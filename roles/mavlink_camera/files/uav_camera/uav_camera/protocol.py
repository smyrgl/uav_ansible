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

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import logging
import math
import select
import socket
import threading
import time

from pymavlink.dialects.v20 import common as mav

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
    codec, camera_name, vendor_name, hfov_deg, min_photo_interval_s. Defaults target the D555
    on jethawk. The backend status method must return quickly and be thread-safe.
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
        if message.get_type() != "COMMAND_LONG":
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
            elif command in (mav.MAV_CMD_VIDEO_START_STREAMING, mav.MAV_CMD_VIDEO_STOP_STREAMING):
                enabled = command == mav.MAV_CMD_VIDEO_START_STREAMING
                self._submit("stream", lambda: self.backend.set_streaming(enabled), transaction, now)
        except (ValueError, OverflowError) as exc:
            self._finish(transaction, mav.MAV_RESULT_DENIED)
            self._status_text(f"Camera: {exc}")
        except Exception as exc:
            self._finish(transaction, mav.MAV_RESULT_FAILED)
            self._status_text(f"Camera command failed: {exc}")

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
            if msg_id in (mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, mav.MAVLINK_MSG_ID_VIDEO_STREAM_STATUS,
                          mav.MAVLINK_MSG_ID_STORAGE_INFORMATION):
                if self._integer(selector) not in (0, 1):
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

    def _stream_information(self):
        flags, fps, width, height, bitrate, rotation, hfov = self._stream_values()
        return mav.MAVLink_video_stream_information_message(
            1, 1, mav.VIDEO_STREAM_TYPE_RTSP, flags, fps, width, height, bitrate, rotation, hfov,
            b"D555 RGB", self.uri.encode(), self.encoding)

    def _stream_status(self):
        return mav.MAVLink_video_stream_status_message(1, *self._stream_values())

    def _image_captured(self, result):
        unknown = 0x7FFFFFFF
        return mav.MAVLink_camera_image_captured_message(
            int(result.get("time_boot_ms", self._boot_ms())) & 0xFFFFFFFF,
            int(result.get("time_utc_us", 0)), 0, unknown, unknown, unknown, unknown,
            [math.nan] * 4, int(result["index"]), 1, b"")
