"""Small MAVLink Camera v2 server for a ROS-backed camera.

Only camera commands are handled. All transport I/O is performed by one thread;
blocking media operations use a single separate worker. ``handle_message`` and
``tick`` are intentionally transport-independent for testing.

Photo pose is unavailable in this initial implementation. CAMERA_IMAGE_CAPTURED
uses INT32_MAX for all integer pose fields and NaN quaternion entries. The common
message has no standard invalid sentinel for integer pose fields: consumers must
not interpret those application sentinels as a geotag. No body attitude is passed
off as calibrated camera attitude. Image-event replay is limited to the most
recent 256 captures in this process; media files are retained by the backend.
"""
from __future__ import annotations

from collections import OrderedDict, deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
import logging
import math
import select
import socket
import threading
import time
from urllib.parse import urlsplit

from pymavlink.dialects.v20 import common as mav

from .rc import BUTTON, RcButtons

# Deprecated command removed from the generated enum; retained for older GCSs.
_REQUEST_IMAGE_CAPTURE = 2002


@dataclass
class _Transaction:
    message: object
    signature: tuple
    wire_sequence: int
    started: float
    result: int = mav.MAV_RESULT_IN_PROGRESS
    last_ack: float = 0.0


class CameraProtocol:
    """Expose ``backend`` using MAVLink 2 over the existing router's TCP port.

    Config keys: system_id, component_id, mavlink_endpoint, rtsp_uri,
    codec, camera_name, vendor_name, hfov_deg, min_photo_interval_s,
    rc_photo_channel, rc_video_channel, rc_button_mode, rc_rate_hz. Defaults target the D555
    on jethawk. The backend status method must return quickly and be thread-safe.

    Transmitter buttons (``rc_*``, channel 0 = off) are read from the autopilot's
    RC_CHANNELS through the same router connection and act like the GCS
    commands, with nobody to acknowledge: photo takes one picture, video
    starts or stops recording. While they are enabled the protocol keeps
    RC_CHANNELS at ``rc_rate_hz`` with SET_MESSAGE_INTERVAL (PX4 streams it at
    5 Hz, slow enough to miss a short press, and forgets the rate on reboot).
    """

    def __init__(self, backend, config: dict, logger=None):
        self.backend = backend
        self.config = config
        self.log = logger or logging.getLogger(__name__)
        self.system_id = int(config.get("system_id", 1))
        self.component_id = int(config.get("component_id", 100))
        if not (1 <= self.system_id <= 255 and 7 <= self.component_id <= 255):
            raise ValueError("Invalid MAVLink system/component ID")
        endpoint = str(config.get("mavlink_endpoint", "tcp:127.0.0.1:5760"))
        if not endpoint.startswith("tcp:"):
            raise ValueError("mavlink_endpoint must be tcp:host:port")
        self.host, port = endpoint[4:].rsplit(":", 1)
        self.host = self.host.removeprefix("//")
        self.port = int(port)
        if not self.host or not 1 <= self.port <= 65535:
            raise ValueError("Invalid MAVLink TCP endpoint")
        self.uri = str(config.get("rtsp_uri", "rtsp://192.168.144.1:8554/rgb"))
        if len(self.uri.encode()) > 159:
            raise ValueError("RTSP URI exceeds MAVLink's 159-byte string capacity")
        codec = str(config.get("codec", "h265")).lower()
        if codec not in ("h264", "h265"):
            raise ValueError("codec must be h264 or h265")
        self.encoding = mav.VIDEO_STREAM_ENCODING_H265 if codec == "h265" else mav.VIDEO_STREAM_ENCODING_H264
        self.extra_streams = self._parse_streams(config.get("extra_streams", ""))
        self._probes = {}
        self._mav = mav.MAVLink(None, srcSystem=self.system_id, srcComponent=self.component_id)
        self._mav.robust_parsing = True
        self._socket = None
        self._thread = None
        self._stop = threading.Event()
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="camera-media-command")
        self._pending = None
        self._transactions = OrderedDict()
        self._photo_sequences = OrderedDict()
        self._images = OrderedDict()
        self._sequence = None
        self._mode = mav.CAMERA_MODE_VIDEO
        self._last_heartbeat = -math.inf
        self._last_status = -math.inf
        self._reported_backend_error = ""
        self._status_period = 1.0
        self._closed = False
        self.min_photo_interval = max(0.5, float(config.get("min_photo_interval_s", 1.0)))
        mode = str(config.get("rc_button_mode", BUTTON))
        buttons = {name: (int(config.get(f"rc_{name}_channel", 0) or 0), mode) for name in ("photo", "video")}
        buttons = {name: spec for name, spec in buttons.items() if spec[0] > 0}
        self.rc = RcButtons(buttons) if buttons else None
        self.rc_rate_hz = float(config.get("rc_rate_hz", 20.0))
        if self.rc and not 1.0 <= self.rc_rate_hz <= 50.0:
            raise ValueError("rc_rate_hz must be between 1 and 50")
        self._rc_frames = deque(maxlen=256)
        self._rc_rate_requested = -math.inf
        self._known_commands = {
            mav.MAV_CMD_REQUEST_MESSAGE,
            mav.MAV_CMD_REQUEST_CAMERA_INFORMATION,
            mav.MAV_CMD_REQUEST_CAMERA_SETTINGS,
            mav.MAV_CMD_REQUEST_STORAGE_INFORMATION,
            mav.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS,
            _REQUEST_IMAGE_CAPTURE,
            mav.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION,
            mav.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS,
            mav.MAV_CMD_SET_CAMERA_MODE,
            mav.MAV_CMD_IMAGE_START_CAPTURE,
            mav.MAV_CMD_IMAGE_STOP_CAPTURE,
            mav.MAV_CMD_VIDEO_START_CAPTURE,
            mav.MAV_CMD_VIDEO_STOP_CAPTURE,
            mav.MAV_CMD_VIDEO_START_STREAMING,
            mav.MAV_CMD_VIDEO_STOP_STREAMING,
        }

    @staticmethod
    def _parse_streams(value):
        """Further RTSP streams served beside the RGB one, as stream_id 2, 3, ... (the LiDAR
        map view): a list of {name, uri, codec, width, height, fps, bitrate, hfov_deg,
        thermal, probe}, or that list as JSON (a ROS parameter cannot hold a list of dicts).
        Each is served by its own process; RUNNING is set while `probe` (host:port, by
        default the URI's) accepts connections. A `thermal` stream is the one QGC overlays
        on the main stream (picture-in-picture, blend, full) instead of listing it."""
        if isinstance(value, str):
            value = json.loads(value) if value.strip() else []
        streams = []
        for spec in value or []:
            uri = str(spec["uri"])
            if not uri.startswith("rtsp://") or len(uri.encode()) > 159:
                raise ValueError(f"Extra stream URI must be rtsp:// and at most 159 bytes: {uri}")
            codec = str(spec.get("codec", "h265")).lower()
            if codec not in ("h264", "h265"):
                raise ValueError("Extra stream codec must be h264 or h265")
            host, _, port = str(spec.get("probe") or urlsplit(uri).netloc).rpartition(":")
            streams.append({
                "name": str(spec.get("name") or f"Stream {len(streams) + 2}")[:31],
                "uri": uri,
                "encoding": mav.VIDEO_STREAM_ENCODING_H265 if codec == "h265" else mav.VIDEO_STREAM_ENCODING_H264,
                "width": int(spec.get("width", 0)), "height": int(spec.get("height", 0)),
                "fps": float(spec.get("fps", 0)), "bitrate": int(spec.get("bitrate", 0)),
                "hfov": int(spec.get("hfov_deg", 0)), "thermal": bool(spec.get("thermal", False)),
                "probe": (host or "127.0.0.1", int(port or 554)),
            })
        return streams

    @staticmethod
    def _boot_ms():
        # Linux monotonic time is measured from boot, not process start.
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

    def start(self):
        if self._closed:
            raise RuntimeError("CameraProtocol is closed")
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="mavlink-camera", daemon=True)
            self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)
        self._disconnect()
        self._sequence = None
        self._worker.shutdown(wait=True, cancel_futures=True)
        self._closed = True

    def _disconnect(self):
        if self._socket:
            try:
                self._socket.close()
            except OSError:
                pass
        self._socket = None

    def _run(self):
        next_connect = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if self._socket is None and now >= next_connect:
                try:
                    self._socket = socket.create_connection((self.host, self.port), timeout=1.0)
                    self._socket.settimeout(0.2)
                    self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self._mav = mav.MAVLink(None, srcSystem=self.system_id, srcComponent=self.component_id)
                    self._mav.robust_parsing = True
                    self._last_heartbeat = -math.inf
                    self._reported_backend_error = ""
                    self.log.info(f"Camera connected to MAVLink router {self.host}:{self.port}")
                except OSError as exc:
                    self.log.warning(f"Camera MAVLink connection unavailable: {exc}")
                    self._disconnect()
                    next_connect = now + 5.0
            try:
                self.tick(now)
                if self._socket is None:
                    self._stop.wait(0.05)
                    continue
                ready, _, _ = select.select([self._socket], [], [], 0.05)
                if ready:
                    data = self._socket.recv(16384)
                    if not data:
                        raise ConnectionError("Router disconnected")
                    for message in self._mav.parse_buffer(data) or ():
                        self.handle_message(message)
            except (OSError, ValueError) as exc:
                self.log.warning(f"Camera MAVLink transport error: {exc}")
                self._disconnect()
                next_connect = time.monotonic() + 1.0
            except Exception as exc:
                # A malformed packet or backend status error must not silently
                # terminate the camera's only heartbeat/command thread.
                self.log.error(f"Camera protocol error: {exc}")
                self._stop.wait(0.05)

    def _send(self, message):
        if self._socket is None:
            return
        packet = message.pack(self._mav)
        self._mav.seq = (self._mav.seq + 1) & 255
        try:
            self._socket.sendall(packet)
        except OSError as exc:
            # Transport loss cannot turn an already-saved photo into a failed
            # capture. Keep its completed transaction/event for retry replay.
            self.log.warning(f"Camera MAVLink send failed: {exc}")
            self._disconnect()

    def _ack(self, message, result):
        self._send(mav.MAVLink_command_ack_message(
            int(message.command), result, 255 if result == mav.MAV_RESULT_IN_PROGRESS else 0,
            0, message.get_srcSystem(), message.get_srcComponent()))

    def _status_text(self, text):
        self.log.warning(text)
        self._send(mav.MAVLink_statustext_message(mav.MAV_SEVERITY_WARNING, text.encode()[:50]))

    def _status(self):
        return self.backend.status()

    @staticmethod
    def _integer(value, lower=0, upper=0x7FFFFFFF):
        value = float(value)
        if not math.isfinite(value) or not value.is_integer() or not lower <= value <= upper:
            raise ValueError("Invalid integer parameter")
        return int(value)

    @staticmethod
    def _signature(message):
        # Canonical NaN spelling makes reserved-NaN parameters compare equal.
        params = tuple(float(getattr(message, f"param{i}")).hex() for i in range(1, 8))
        return (message.get_srcSystem(), message.get_srcComponent(), int(message.command), params)

    def _remember(self, message, now):
        signature = self._signature(message)
        transaction = _Transaction(message, signature, message.get_seq(), now)
        self._transactions[signature] = transaction
        self._transactions.move_to_end(signature)
        while len(self._transactions) > 256:
            self._transactions.popitem(last=False)
        return transaction

    def _finish(self, transaction, result):
        if transaction:
            transaction.result = result
            transaction.last_ack = time.monotonic()
            self._ack(transaction.message, result)

    def handle_message(self, message):
        kind = message.get_type()
        if kind in ("RC_CHANNELS", "SYS_STATUS"):
            if self.rc and (message.get_srcSystem(), message.get_srcComponent()) == (
                    self.system_id, mav.MAV_COMP_ID_AUTOPILOT1):
                self._handle_rc(message, time.monotonic())
            return
        if kind != "COMMAND_LONG":
            return
        if message.target_system not in (0, self.system_id) or message.target_component not in (0, self.component_id):
            return
        if message.get_srcSystem() == self.system_id and message.get_srcComponent() == self.component_id:
            return
        command = int(message.command)
        if command not in self._known_commands:
            # Never acknowledge another component's broadcast flight commands.
            if message.target_component == self.component_id:
                self._ack(message, mav.MAV_RESULT_UNSUPPORTED)
            return
        now = time.monotonic()
        # Information requests are harmless and must regenerate their response on
        # every retry (the response, rather than its ACK, may have been lost).
        if command in self._request_commands():
            self._handle_request(message)
            return
        signature = self._signature(message)
        if self._pending and self._pending[2] and self._pending[2].signature == signature:
            self._ack(message, mav.MAV_RESULT_IN_PROGRESS)
            return
        previous = self._transactions.get(signature)
        if previous and now - previous.started < 60 and (
            message.confirmation > 0 or (message.get_seq() == previous.wire_sequence and now - previous.started < 2)
        ):
            self._ack(message, previous.result)
            return
        transaction = self._remember(message, now)
        try:
            if command == mav.MAV_CMD_SET_CAMERA_MODE:
                mode = self._integer(message.param2, 0, 1)
                self._mode = mode
                self._finish(transaction, mav.MAV_RESULT_ACCEPTED)
                self._send(self._settings())
                return
            if command in (mav.MAV_CMD_IMAGE_START_CAPTURE, mav.MAV_CMD_IMAGE_STOP_CAPTURE):
                if self._integer(message.param1, 0, 255) != 0:
                    self._finish(transaction, mav.MAV_RESULT_DENIED)
                    return
            elif command in (mav.MAV_CMD_VIDEO_START_STREAMING, mav.MAV_CMD_VIDEO_STOP_STREAMING):
                stream = self._integer(message.param1, 0, 255)
                if stream > 1 + len(self.extra_streams):
                    self._finish(transaction, mav.MAV_RESULT_DENIED)
                    return
                if stream >= 2:
                    # QGC sends START for the stream it switches to. An extra stream is
                    # served on demand by its own process: nothing to switch here, and
                    # the RGB stream stays as it is.
                    self._finish(transaction, mav.MAV_RESULT_ACCEPTED)
                    self._send(self._stream_status(stream))
                    return
            else:
                if self._integer(message.param1, 0, 255) not in (0, 1):
                    self._finish(transaction, mav.MAV_RESULT_DENIED)
                    return
            if command == mav.MAV_CMD_IMAGE_STOP_CAPTURE:
                self._sequence = None  # A frame already being saved finishes.
                self._finish(transaction, mav.MAV_RESULT_ACCEPTED)
                self._send(self._capture_status())
                return
            if self._pending is not None:
                self._finish(transaction, mav.MAV_RESULT_TEMPORARILY_REJECTED)
                return
            if command == mav.MAV_CMD_IMAGE_START_CAPTURE:
                count = self._integer(message.param3, 0, 1000000)
                interval = float(message.param2)
                sequence = self._integer(message.param4)
                if not math.isfinite(interval) or interval < 0 or (count != 1 and interval < self.min_photo_interval):
                    raise ValueError(f"Photo interval must be at least {self.min_photo_interval:g}s")
                if count != 1 and sequence:
                    raise ValueError("Capture sequence ID is only valid for single photos")
                key = (message.get_srcSystem(), message.get_srcComponent(), sequence)
                if count == 1 and sequence and key in self._photo_sequences:
                    earlier = self._photo_sequences[key]
                    self._finish(transaction, earlier.result)
                    return
                if self._sequence is not None:
                    self._finish(transaction, mav.MAV_RESULT_TEMPORARILY_REJECTED)
                    return
                if not self._status().get("ready", False):
                    self._finish(transaction, mav.MAV_RESULT_TEMPORARILY_REJECTED)
                    self._status_text("D555: no fresh RGB frame for photo")
                    return
                if count == 1 and sequence:
                    self._photo_sequences[key] = transaction
                    while len(self._photo_sequences) > 512:
                        self._photo_sequences.popitem(last=False)
                if count != 1:
                    self._sequence = {"remaining": count or None, "interval": interval, "next": now + interval}
                self._submit("photo", self.backend.capture_photo, transaction, now)
                return
            if command == mav.MAV_CMD_VIDEO_START_CAPTURE:
                rate = float(message.param2)
                if not math.isfinite(rate) or not 0 <= rate <= 10:
                    raise ValueError("Capture status frequency must be between 0 and 10Hz")
                self._status_period = 1.0 / rate if rate else math.inf
                if self._status().get("recording", False):
                    self._finish(transaction, mav.MAV_RESULT_ACCEPTED)
                elif not self._status().get("ready", False):
                    self._finish(transaction, mav.MAV_RESULT_TEMPORARILY_REJECTED)
                else:
                    self._submit("record_start", self.backend.start_recording, transaction, now)
            elif command == mav.MAV_CMD_VIDEO_STOP_CAPTURE:
                if not self._status().get("recording", False):
                    self._finish(transaction, mav.MAV_RESULT_ACCEPTED)
                else:
                    self._submit("record_stop", self.backend.stop_recording, transaction, now)
            elif command == mav.MAV_CMD_VIDEO_STOP_STREAMING:
                # Acknowledged, not obeyed. QGC sends STOP for the stream it leaves
                # and START for the one it shows, so honouring STOP would cut the
                # RGB for every other viewer (the GCS, another QGC) the moment one
                # switches to the LiDAR view, until someone switched back. And it
                # saves nothing: RTSP sends only to clients that PLAY, and the
                # encoder runs regardless (recordings, Foxglove).
                self._finish(transaction, mav.MAV_RESULT_ACCEPTED)
                self._send(self._stream_status())
            elif command == mav.MAV_CMD_VIDEO_START_STREAMING:
                # Also requests a keyframe, so the viewer that switched starts at once.
                self._submit("stream", lambda: self.backend.set_streaming(True), transaction, now)
        except (ValueError, OverflowError) as exc:
            self._finish(transaction, mav.MAV_RESULT_DENIED)
            self._status_text(f"Camera: {exc}")
        except Exception as exc:
            self._finish(transaction, mav.MAV_RESULT_FAILED)
            self._status_text(f"Camera command failed: {exc}")

    def _handle_rc(self, message, now):
        if message.get_type() == "SYS_STATUS":
            self.rc.set_present(message.onboard_control_sensors_present & mav.MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
            return
        self._rc_frames.append(now)
        values = [getattr(message, f"chan{i}_raw") for i in range(1, 19)]
        for name in self.rc.update(values, now):
            self._rc_press(name, now)

    def _rc_press(self, name, now):
        """A transmitter button: the GCS command's checks, without a requester to ACK."""
        status = self._status()
        if self._pending is not None:
            self._status_text(f"Camera busy, RC {name} ignored")
        elif name == "photo":
            if self._sequence is not None:
                self._status_text("Photo sequence running, RC photo ignored")
            elif not status.get("ready", False):
                self._status_text("D555: no fresh RGB frame for photo")
            else:
                self.log.info("RC button: photo")
                self._submit("photo", self.backend.capture_photo, None, now)
        elif name == "video":
            if status.get("recording", False):
                self.log.info("RC button: stop recording")
                self._submit("record_stop", self.backend.stop_recording, None, now)
            elif not status.get("ready", False):
                self._status_text("D555: no fresh RGB frame, RC record ignored")
            else:
                self.log.info("RC button: start recording")
                self._submit("record_start", self.backend.start_recording, None, now)

    def _keep_rc_rate(self, now):
        # Ask again whenever the last 5 s fell short of the rate (PX4 reboot,
        # router reconnect); at most every 10 s.
        if self._socket is None or now - self._rc_rate_requested < 10.0:
            return
        recent = sum(1 for t in self._rc_frames if now - t <= 5.0)
        if recent >= 0.75 * 5.0 * self.rc_rate_hz:
            return
        self._rc_rate_requested = now
        self._send(mav.MAVLink_command_long_message(
            self.system_id, mav.MAV_COMP_ID_AUTOPILOT1, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mav.MAVLINK_MSG_ID_RC_CHANNELS, 1e6 / self.rc_rate_hz, 0, 0, 0, 0, 0))

    def rc_status(self):
        if not self.rc:
            return None
        now = time.monotonic()
        return {"buttons": {name: {"channel": channel, "mode": mode} for name, (channel, mode) in self.rc.buttons.items()},
                "rc_present": self.rc.present, "presses": dict(self.rc.presses), "suppressed": self.rc.suppressed,
                "rate_hz": round(sum(1 for t in self._rc_frames if now - t <= 5.0) / 5.0, 1)}

    @staticmethod
    def _request_commands():
        return {
            mav.MAV_CMD_REQUEST_MESSAGE, mav.MAV_CMD_REQUEST_CAMERA_INFORMATION,
            mav.MAV_CMD_REQUEST_CAMERA_SETTINGS, mav.MAV_CMD_REQUEST_STORAGE_INFORMATION,
            mav.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, _REQUEST_IMAGE_CAPTURE,
            mav.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, mav.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS,
        }

    def _handle_request(self, message):
        legacy = {
            mav.MAV_CMD_REQUEST_CAMERA_INFORMATION: mav.MAVLINK_MSG_ID_CAMERA_INFORMATION,
            mav.MAV_CMD_REQUEST_CAMERA_SETTINGS: mav.MAVLINK_MSG_ID_CAMERA_SETTINGS,
            mav.MAV_CMD_REQUEST_STORAGE_INFORMATION: mav.MAVLINK_MSG_ID_STORAGE_INFORMATION,
            mav.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS: mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS,
            _REQUEST_IMAGE_CAPTURE: mav.MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED,
            mav.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION: mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,
            mav.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS: mav.MAVLINK_MSG_ID_VIDEO_STREAM_STATUS,
        }
        try:
            generic = message.command == mav.MAV_CMD_REQUEST_MESSAGE
            msg_id = self._integer(message.param1) if generic else legacy[message.command]
            selector = message.param2 if generic else message.param1
            if msg_id in (mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, mav.MAVLINK_MSG_ID_VIDEO_STREAM_STATUS):
                # Stream 0 is all of them: QGC asks so, then once per stream still missing.
                stream = self._integer(selector)
                if stream > 1 + len(self.extra_streams):
                    self._ack(message, mav.MAV_RESULT_DENIED)
                    return
                build = (self._stream_information if msg_id == mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION
                         else self._stream_status)
                responses = [build(i) for i in (range(1, 2 + len(self.extra_streams)) if stream == 0 else (stream,))]
                self._ack(message, mav.MAV_RESULT_ACCEPTED)
                for response in responses:
                    self._send(response)
                return
            if msg_id == mav.MAVLINK_MSG_ID_STORAGE_INFORMATION and self._integer(selector) not in (0, 1):
                self._ack(message, mav.MAV_RESULT_DENIED)
                return
            builders = {
                mav.MAVLINK_MSG_ID_CAMERA_INFORMATION: self._information,
                mav.MAVLINK_MSG_ID_CAMERA_SETTINGS: self._settings,
                mav.MAVLINK_MSG_ID_STORAGE_INFORMATION: self._storage,
                mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: self._capture_status,
                mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: self._stream_information,
                mav.MAVLINK_MSG_ID_VIDEO_STREAM_STATUS: self._stream_status,
            }
            if msg_id == mav.MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
                index = self._integer(selector, -1)
                end = self._integer(message.param3, -1) if generic else 0
                selected = [event for i, event in self._images.items()
                            if index == -1 or (i >= index and (end == -1 or i <= (end or index)))]
                if not selected:
                    self._ack(message, mav.MAV_RESULT_DENIED)
                    return
                self._ack(message, mav.MAV_RESULT_ACCEPTED)
                for event in selected:
                    self._send(event)
            elif msg_id in builders:
                response = builders[msg_id]()
                self._ack(message, mav.MAV_RESULT_ACCEPTED)
                self._send(response)
            else:
                # Ignore unrelated broadcast message requests.
                if message.target_component != 0:
                    self._ack(message, mav.MAV_RESULT_UNSUPPORTED)
        except (ValueError, OverflowError):
            self._ack(message, mav.MAV_RESULT_DENIED)
        except Exception as exc:
            self._ack(message, mav.MAV_RESULT_FAILED)
            self.log.error(f"Camera information request failed: {exc}")

    def _submit(self, kind, callback, transaction, now):
        future = self._worker.submit(callback)
        self._pending = (future, kind, transaction)
        if transaction:
            transaction.last_ack = now
            self._ack(transaction.message, mav.MAV_RESULT_IN_PROGRESS)

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        if self.rc:
            self._keep_rc_rate(now)
        if self._pending:
            future, kind, transaction = self._pending
            if future.done():
                self._pending = None
                try:
                    result = future.result()
                    if kind == "photo":
                        if not result.get("success", False):
                            raise RuntimeError("Photo backend did not save an image")
                        event = self._image_captured(result)
                        self._images[event.image_index] = event
                        while len(self._images) > 256:
                            self._images.popitem(last=False)
                        self._finish(transaction, mav.MAV_RESULT_ACCEPTED)
                        self._send(event)
                        if self._sequence and self._sequence["remaining"] is not None:
                            self._sequence["remaining"] -= 1
                            if self._sequence["remaining"] <= 0:
                                self._sequence = None
                    else:
                        self._finish(transaction, mav.MAV_RESULT_ACCEPTED)
                    self._send(self._capture_status())
                    self._send(self._stream_status())
                except Exception as exc:
                    self._sequence = None if kind == "photo" else self._sequence
                    self._finish(transaction, mav.MAV_RESULT_FAILED)
                    self._status_text(f"Camera {kind} failed: {exc}")
                    self._send(self._capture_status())
            elif transaction and now - transaction.last_ack >= 1.0:
                transaction.last_ack = now
                self._ack(transaction.message, mav.MAV_RESULT_IN_PROGRESS)
        if self._sequence and not self._pending and now >= self._sequence["next"]:
            if not self._status().get("ready", False):
                self._sequence = None
                self._status_text("Photo sequence stopped: RGB frames unavailable")
            else:
                self._sequence["next"] = now + self._sequence["interval"]
                self._submit("photo", self.backend.capture_photo, None, now)
        status = self._status()
        error = str(status.get("last_error") or "")
        if error != self._reported_backend_error:
            self._reported_backend_error = error
            if error:
                # Recording/encoder failures can occur asynchronously, after the
                # command's successful start ACK. Surface them to the GCS as
                # well as through local logs/ROS status. Repeat after reconnect.
                self._status_text(f"D555: {error}")
        if now - self._last_heartbeat >= 1.0:
            self._last_heartbeat = now
            status = self._status()
            self._send(mav.MAVLink_heartbeat_message(
                mav.MAV_TYPE_CAMERA, mav.MAV_AUTOPILOT_INVALID, 0, 0,
                mav.MAV_STATE_ACTIVE if status.get("ready", False) else mav.MAV_STATE_STANDBY, 3))
        status_period = self._status_period if self._status().get("recording", False) else 1.0
        if now - self._last_status >= status_period:
            self._last_status = now
            self._send(self._capture_status())
            self._send(self._stream_status())

    def _information(self):
        status = self._status()
        flags = (mav.CAMERA_CAP_FLAGS_CAPTURE_VIDEO | mav.CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
                 mav.CAMERA_CAP_FLAGS_HAS_MODES | mav.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
                 mav.CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE | mav.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM)
        def name(value):
            return list(str(value).encode()[:31].ljust(32, b"\0"))
        # Optical dimensions are intentionally unknown; use CameraInfo, not
        # made-up focal length or sensor size, for the calibrated ROS camera.
        return mav.MAVLink_camera_information_message(
            self._boot_ms(), name(self.config.get("vendor_name", "RealSense")),
            name(self.config.get("camera_name", "D555 RGB")), 1, 0.0, 0.0, 0.0,
            int(status.get("width", 0)), int(status.get("height", 0)), 0, flags, 0, b"")

    def _settings(self):
        return mav.MAVLink_camera_settings_message(self._boot_ms(), self._mode, math.nan, math.nan)

    def _storage(self):
        status = self._status()
        total = float(status.get("total_mib", 0))
        free = float(status.get("available_mib", 0))
        return mav.MAVLink_storage_information_message(
            self._boot_ms(), 1, 1, mav.STORAGE_STATUS_READY, total, max(0.0, total - free), free,
            0.0, 0.0, mav.STORAGE_TYPE_HD, b"Jetson NVMe")

    def _capture_status(self):
        status = self._status()
        saving = self._pending is not None and self._pending[1] == "photo"
        image_status = (2 if self._sequence else 0) + int(saving)
        started = status.get("recording_started_mono")
        elapsed = max(0, int((time.monotonic() - started) * 1000)) if started and status.get("recording") else 0
        return mav.MAVLink_camera_capture_status_message(
            self._boot_ms(), image_status, int(bool(status.get("recording", False))),
            float(self._sequence["interval"]) if self._sequence else 0.0,
            min(elapsed, 0xFFFFFFFF), float(status.get("available_mib", 0)), int(status.get("image_count", 0)))

    def _stream_values(self):
        status = self._status()
        running = status.get("streaming", False) and status.get("ready", False)
        return (
            mav.VIDEO_STREAM_STATUS_FLAGS_RUNNING if running else 0,
            float(status.get("fps", 0)), int(status.get("width", 0)), int(status.get("height", 0)),
            int(status.get("bitrate", 0)), 0, int(float(self.config.get("hfov_deg", 90))))

    def _reachable(self, stream, now=None):
        """Whether an extra stream's RTSP server accepts connections (cached for 5 s; the
        probe is a local address, so it connects or is refused at once)."""
        now = time.monotonic() if now is None else now
        cached = self._probes.get(stream["probe"])
        if cached and now - cached[0] < 5.0:
            return cached[1]
        try:
            with socket.create_connection(stream["probe"], timeout=0.25):
                up = True
        except OSError:
            up = False
        self._probes[stream["probe"]] = (now, up)
        return up

    def _extra_values(self, stream):
        flags = mav.VIDEO_STREAM_STATUS_FLAGS_RUNNING if self._reachable(stream) else 0
        if stream["thermal"]:
            flags |= mav.VIDEO_STREAM_STATUS_FLAGS_THERMAL
        return flags, stream["fps"], stream["width"], stream["height"], stream["bitrate"], 0, stream["hfov"]

    def _stream_information(self, stream_id=1):
        if stream_id == 1:
            flags, fps, width, height, bitrate, rotation, hfov = self._stream_values()
            name, uri, encoding = b"D555 RGB", self.uri.encode(), self.encoding
        else:
            stream = self.extra_streams[stream_id - 2]
            flags, fps, width, height, bitrate, rotation, hfov = self._extra_values(stream)
            name, uri, encoding = stream["name"].encode(), stream["uri"].encode(), stream["encoding"]
        return mav.MAVLink_video_stream_information_message(
            stream_id, 1 + len(self.extra_streams), mav.VIDEO_STREAM_TYPE_RTSP, flags, fps, width, height,
            bitrate, rotation, hfov, name, uri, encoding)

    def _stream_status(self, stream_id=1):
        values = self._stream_values() if stream_id == 1 else self._extra_values(self.extra_streams[stream_id - 2])
        return mav.MAVLink_video_stream_status_message(stream_id, *values)

    def _image_captured(self, result):
        unknown = 0x7FFFFFFF
        return mav.MAVLink_camera_image_captured_message(
            int(result.get("time_boot_ms", self._boot_ms())) & 0xFFFFFFFF,
            int(result.get("time_utc_us", 0)), 0, unknown, unknown, unknown, unknown,
            [math.nan] * 4, int(result["index"]), 1, b"")
