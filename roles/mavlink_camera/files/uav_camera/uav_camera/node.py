"""ROS 2 native-DDS RGB consumer and MAVLink camera application."""
import os
import collections
import json
import logging
from array import array
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
try:
    from foxglove_msgs.msg import CompressedVideo
except ImportError:  # ros-<distro>-foxglove-msgs missing: the video topic is disabled
    CompressedVideo = None

from .clock import ClockMapping
from .media import MediaManager
from .protocol import CameraProtocol

DEFAULTS = {
    'image_topic': '/realsense/D555_261622302751_Color',
    'camera_info_topic': '/realsense/D555_261622302751_Color/camera_info',
    'codec': 'h265', 'bitrate': 4000000, 'fps': 30, 'rotation_degrees': 180,
    'rtsp_bind': '0.0.0.0', 'rtsp_port': 8554, 'rtsp_path': '/rgb',
    'rtsp_uri': 'rtsp://192.168.144.1:8554/rgb',
    'storage_dir': '/home/john/camera-media',
    'frame_timeout_s': 2.0, 'photo_timeout_s': 3.0,
    'mavlink_endpoint': 'tcp:127.0.0.1:5760',
    'system_id': 1, 'component_id': 100,
    'camera_name': 'D555 RGB', 'vendor_name': 'RealSense',
    # Encoded access units as foxglove_msgs/CompressedVideo ('' disables). The
    # same NVENC output as RTSP, so Foxglove costs no second encode and never
    # touches the D555's own compressed streams (see README: they throttle the
    # imager to ~3.5 fps and replay stale buffers).
    'video_topic': '/d555/color/video', 'video_frame_id': 'camera_color_optical_frame',
    'clock_topic': '/d555/clock',
    # Every colour frame as the camera sent it, republished as sensor_msgs/Image
    # with its header mapped to UTC and the canonical frame id ('' disables): the
    # bag's lossless colour record. The camera unicasts a copy per reader, so the
    # local copy has to come from this node, its one colour reader.
    'raw_topic': '',
    # Transmitter buttons from the autopilot's RC_CHANNELS (channel 0 = off).
    # 'button' acts on a momentary press, 'toggle' on every flip of a latching one.
    'rc_photo_channel': 0, 'rc_video_channel': 0, 'rc_button_mode': 'button', 'rc_rate_hz': 20.0,
    # Further RTSP streams of this camera (stream_id 2, 3, ...), served by other
    # processes (the LiDAR map view), as a JSON list: see CameraProtocol._parse_streams.
    'extra_streams': '',
}


# sensor_msgs/Image encoding -> (bytes per pixel, GStreamer raw format). The
# frame goes to the encoder pipeline as it arrives; nvvidconv converts it on the
# VIC (YUY2 is what the D555 sends: 2 bytes per pixel, no CPU colour conversion).
NATIVE_FORMATS = {'yuv422_yuy2': (2, 'YUY2'), 'yuv422': (2, 'UYVY'), 'rgb8': (3, 'RGB'),
                  'rgba8': (4, 'RGBA'), 'bgra8': (4, 'BGRx'), 'mono8': (1, 'GRAY8'), 'bgr8': (3, 'BGR')}


def restamped_image(msg, stamp_ns, frame_id):
    """The same Image with a new header. rclpy's generated setter copies the
    pixel array (Jazzy builds messages with check_fields off), so a republish
    costs one 2 MB memcpy plus the serialization: measured as a few percent of a
    core at 30 Hz. A zero-copy version of this path is a C++ node passing
    serialized buffers, or NITROS: the roadmap's Stage 2 compute item."""
    out = Image()
    out.header.stamp.sec, out.header.stamp.nanosec = divmod(int(stamp_ns), 1000000000)
    out.header.frame_id = frame_id
    out.height, out.width, out.encoding = msg.height, msg.width, msg.encoding
    out.is_bigendian, out.step = msg.is_bigendian, msg.step
    out.data = msg.data
    return out


def raw_frame(msg):
    """(pixel bytes, GStreamer format) of an Image, stride trimmed, no conversion
    and no rotation (nvvidconv flips in hardware). One copy when the stride
    carries padding, none otherwise."""
    enc = msg.encoding.lower()
    if enc not in NATIVE_FORMATS:
        raise ValueError(f'Unsupported RGB source encoding {msg.encoding!r}')
    channels, fmt = NATIVE_FORMATS[enc]
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels:
        raise ValueError('Invalid image dimensions or stride')
    if len(msg.data) != msg.step * msg.height:
        raise ValueError('Image data length does not match stride and height')
    if msg.step == msg.width * channels:
        return bytes(msg.data), fmt
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return rows[:, :msg.width * channels].tobytes(), fmt


def rgb_bytes(msg, rotation_degrees=0):
    """Respect sensor_msgs/Image row stride; convert in the worker, never callback."""
    enc = msg.encoding.lower()
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4,
                'mono8': 1, 'yuv422_yuy2': 2, 'yuv422': 2}.get(enc)
    if channels is None:
        raise ValueError(f'Unsupported RGB source encoding {msg.encoding!r}')
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels:
        raise ValueError('Invalid image dimensions or stride')
    if len(msg.data) != msg.step * msg.height:
        raise ValueError('Image data length does not match stride and height')
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
    code = {'bgr8': cv2.COLOR_BGR2RGB, 'rgba8': cv2.COLOR_RGBA2RGB,
            'bgra8': cv2.COLOR_BGRA2RGB, 'mono8': cv2.COLOR_GRAY2RGB,
            'yuv422_yuy2': cv2.COLOR_YUV2RGB_YUY2,
            'yuv422': cv2.COLOR_YUV2RGB_UYVY}.get(enc)
    if code is not None:
        pixels = cv2.cvtColor(pixels, code)
    if rotation_degrees == 180:
        pixels = pixels[::-1, ::-1]
    elif rotation_degrees != 0:
        raise ValueError('rotation_degrees must be 0 or 180')
    return pixels.tobytes()


class CameraNode(Node):
    def __init__(self):
        super().__init__('uav_camera')
        self.declare_parameters('', list(DEFAULTS.items()))
        self.config = {k: self.get_parameter(k).value for k in DEFAULTS}
        if self.config['rotation_degrees'] not in (0, 180):
            raise ValueError('rotation_degrees must be 0 or 180')
        self.log = logging.getLogger('uav_camera')
        self._video_pub = None
        self._video_published = 0
        if self.config['video_topic']:
            if CompressedVideo is None:
                self.log.error('video_topic %s disabled: foxglove_msgs is not installed',
                               self.config['video_topic'])
            else:
                # Best effort on purpose: a reliable writer can block in publish()
                # while a slow or departing reader is in play, and this publisher
                # is fed from the encoder's streaming thread. A dropped frame is
                # far cheaper than a stalled encoder (measured: a reliable reader
                # coming and going stalled NVENC for >3 s and forced a rebuild).
                video_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                       reliability=ReliabilityPolicy.BEST_EFFORT,
                                       durability=DurabilityPolicy.VOLATILE)
                self._video_pub = self.create_publisher(CompressedVideo, self.config['video_topic'], video_qos)
        # The raw colour record for the bag: published from the image callback,
        # before the frame is handed to the encoder thread, so every frame the
        # camera delivers is republished whether or not the encoder keeps up.
        self._raw_pub = None
        self._raw_published = self._raw_capture_stamped = self._raw_receipt_stamped = 0
        if self.config.get('raw_topic'):
            raw_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
            self._raw_pub = self.create_publisher(Image, self.config['raw_topic'], raw_qos)
        # Encoded access units are queued by the GStreamer thread and published
        # from the executor, so DDS can never hold up the encoder.
        self._video_queue = collections.deque(maxlen=8)
        self._video_dropped = 0
        if self._video_pub:
            self._video_timer = self.create_timer(0.005, self._drain_video)
        self.media = MediaManager(self.config, logger=self.log,
                                  on_encoded=self._encoded if self._video_pub else None)
        self.protocol = CameraProtocol(self.media, self.config, logger=self.log)
        self._condition = threading.Condition()
        self._pending = None
        self._info = None
        self._stopping = False
        self._received = 0
        self._dropped = 0
        self._converted = 0
        self._conversion_error = ''
        self._last_arrival = None
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._image_sub = self.create_subscription(Image, self.config['image_topic'], self._image, qos)
        self._info_sub = self.create_subscription(CameraInfo, self.config['camera_info_topic'], self._camera_info, qos)
        # D555 clock model (published latched by the D555 adapter): maps each
        # frame's device stamp to UTC capture time for video and photos.
        self.clock = ClockMapping()
        self._video_capture_stamped = 0
        self._video_receipt_stamped = 0
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._clock_sub = (self.create_subscription(String, self.config['clock_topic'],
                                                    lambda m: self.clock.update(m.data), latched)
                           if self.config.get('clock_topic') else None)
        self._status_pub = self.create_publisher(String, '~/status', 1)
        self._timer = self.create_timer(1.0, self._status)
        self.media.start()
        self._worker = threading.Thread(target=self._convert, name='rgb-converter', daemon=True)
        self._worker.start()
        self.protocol.start()
        self.log.info('Listening to %s; advertised video %s; video topic %s; raw topic %s',
                      self.config['image_topic'], self.config['rtsp_uri'],
                      self.config['video_topic'] if self._video_pub else 'disabled',
                      self.config['raw_topic'] if self._raw_pub else 'disabled')

    def _image(self, msg):
        received_ns = time.time_ns()
        if self._raw_pub is not None:
            self._publish_raw(msg, received_ns)
        with self._condition:
            self._received += 1
            self._last_arrival = time.monotonic()
            if self._pending is not None:
                self._dropped += 1
            self._pending = (msg, self._info, received_ns, time.monotonic_ns())
            self._condition.notify()

    def _publish_raw(self, msg, received_ns):
        """The frame as the camera sent it, header in UTC (the device stamp through
        the /d555/clock model, receipt time while no model is valid) and the
        canonical frame id."""
        capture_ns = self.clock.to_utc_ns(msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)
        if capture_ns is not None:
            self._raw_capture_stamped += 1
        else:
            capture_ns = received_ns
            self._raw_receipt_stamped += 1
        frame_id = self.config['video_frame_id'] or msg.header.frame_id.rstrip('\x00')
        self._raw_pub.publish(restamped_image(msg, capture_ns, frame_id))
        self._raw_published += 1

    def _encoded(self, data, keyframe, meta):
        """Runs on the GStreamer streaming thread: queue only, never publish
        here. One Annex B access unit per message, keyframes self-contained
        (VPS/SPS/PPS inserted)."""
        if len(self._video_queue) == self._video_queue.maxlen:
            self._video_dropped += 1
        self._video_queue.append((data, keyframe, meta))

    def _drain_video(self):
        """Executor side: publish what the encoder queued. Stamped with the
        frame's capture time in UTC (its device stamp through the /d555/clock
        model), or host receive time while no valid model is available.
        Foxglove schemas are flat (timestamp + frame_id, no std_msgs Header)."""
        while self._video_queue:
            data, keyframe, meta = self._video_queue.popleft()
            msg = CompressedVideo()
            capture_ns = self.clock.to_utc_ns(meta[0]) if meta else None
            if capture_ns is not None:
                utc_us = capture_ns // 1000
                self._video_capture_stamped += 1
            else:
                utc_us = meta[2] if meta else time.time_ns() // 1000
                self._video_receipt_stamped += 1
            msg.timestamp.sec = utc_us // 1000000
            msg.timestamp.nanosec = (utc_us % 1000000) * 1000
            msg.frame_id = self.config['video_frame_id'] or (meta[1].rstrip('\x00') if meta else '')
            msg.format = self.media.codec
            msg.data = array('B', data)  # array fast path: no per-byte range check
            self._video_pub.publish(msg)
            self._video_published += 1

    def _camera_info(self, msg):
        self._info = {
            'width': msg.width, 'height': msg.height, 'frame_id': msg.header.frame_id,
            'stamp_ns': msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec,
            'distortion_model': msg.distortion_model, 'd': list(msg.d),
            'k': list(msg.k), 'r': list(msg.r), 'p': list(msg.p),
            'binning_x': msg.binning_x, 'binning_y': msg.binning_y,
            'roi': {'x_offset': msg.roi.x_offset, 'y_offset': msg.roi.y_offset,
                    'height': msg.roi.height, 'width': msg.roi.width, 'do_rectify': msg.roi.do_rectify},
            'source_clock': 'camera_device_clock_unmapped',
            'pose_available': False,
        }

    def _convert(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._pending is not None or self._stopping)
                if self._stopping:
                    return
                msg, info, received_utc_ns, received_monotonic_ns = self._pending
                self._pending = None
            try:
                rotation = self.config['rotation_degrees']
                valid_info = (info and info.get('width') == msg.width and
                              info.get('height') == msg.height and
                              info.get('frame_id') == msg.header.frame_id)
                info = dict(info) if valid_info else {'calibration_available': False}
                info['host_ros_callback_utc_ns'] = received_utc_ns
                capture_ns = self.clock.to_utc_ns(msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)
                info['capture_utc_us'] = capture_ns // 1000 if capture_ns is not None else None
                info['capture_clock'] = ('d555_device_clock_mapped_to_utc' if capture_ns is not None
                                         else f'unavailable: {self.clock.reason}')
                info['host_ros_callback_monotonic_ns'] = received_monotonic_ns
                info['output_rotation_degrees'] = rotation
                info['intrinsics_reference'] = 'original_unrotated_source_pixels'
                info['source_to_saved_pixel_transform'] = (
                    [[-1, 0, msg.width-1], [0, -1, msg.height-1], [0, 0, 1]]
                    if rotation == 180 else [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
                pixels, fmt = raw_frame(msg)
                self.media.submit_frame(pixels, msg.width, msg.height,
                                        msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec,
                                        msg.header.frame_id, info, fmt)
                self._converted += 1
                self._conversion_error = ''
            except Exception as exc:
                if str(exc) != self._conversion_error:
                    self.log.exception('RGB conversion failed')
                self._conversion_error = str(exc)

    def _status(self):
        status = self.media.status()
        status.update(received_frames=self._received, converted_frames=self._converted,
                      dropped_before_conversion=self._dropped,
                      source_age_s=None if self._last_arrival is None else time.monotonic()-self._last_arrival,
                      conversion_error=self._conversion_error,
                      video_topic=self.config['video_topic'] if self._video_pub else '',
                      video_frames_published=self._video_published, video_frames_dropped=self._video_dropped,
                      video_stamped_capture_utc=self._video_capture_stamped,
                      video_stamped_receipt=self._video_receipt_stamped,
                      raw_topic=self.config['raw_topic'] if self._raw_pub else '',
                      raw_frames_published=self._raw_published,
                      raw_stamped_capture_utc=self._raw_capture_stamped,
                      raw_stamped_receipt=self._raw_receipt_stamped,
                      clock_model=self.clock.reason, rc_buttons=self.protocol.rc_status(),
                      source_timestamp_clock='camera_device_clock, mapped to UTC via /d555/clock')
        self._status_pub.publish(String(data=json.dumps(status, default=str)))

    def close(self):
        self.protocol.close()
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._worker.join(timeout=5)
        self.media.close()


def _spin(node):
    """rclpy's default executor, or the experimental EventsExecutor when
    UAV_EVENTS_EXECUTOR=1 (uav_ansible: ros_events_executor), the A/B of the
    autonomy roadmap's compute-recovery item."""
    import rclpy     # some nodes import it inside main()
    if os.environ.get("UAV_EVENTS_EXECUTOR", "0") == "1":
        try:
            from rclpy.experimental.events_executor import EventsExecutor
        except ImportError:
            EventsExecutor = None
        if EventsExecutor is not None:
            executor = EventsExecutor()
            executor.add_node(node)
            try:
                executor.spin()
            finally:
                executor.remove_node(node)
                executor.shutdown()
            return
    rclpy.spin(node)

def main(args=None):
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        _spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
