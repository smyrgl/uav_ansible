"""Bounded ROS RGB -> Jetson NVENC -> RTSP / local capture media service.

GStreamer is loaded at start(), so source freshness and photo handling can be
tested without Jetson plugins. Media timestamps use a monotonic clock; original
ROS timestamps and CameraInfo are retained in capture sidecars.
"""

from __future__ import annotations

import copy
import json
import os
import shutil
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class _Frame:
    rgb: bytes
    width: int
    height: int
    stamp_ns: int
    frame_id: str
    camera_info: dict | None
    sequence: int
    received_mono: float
    received_utc_us: int


class MediaManager:
    def __init__(self, config: dict, logger=None, on_encoded=None):
        self.config = dict(config)
        self.logger = logger
        # Optional consumer of every encoded access unit: hook(data, keyframe,
        # meta) with meta = (source stamp_ns, frame_id, host receive UTC us) or
        # None. The ROS node publishes these as a video topic; one NVENC pass
        # feeds RTSP, recordings and the dashboard alike.
        self._on_encoded_hook = on_encoded
        self._pts_index: dict[int, tuple[int, str, int]] = {}
        self._pts_index_limit = 128
        self.encoded_hook_calls = 0
        self.codec = str(config.get("codec", "h265")).lower()
        if self.codec not in {"h264", "h265"}:
            raise ValueError("codec must be h264 or h265")
        self.fps = int(config.get("fps", 30))
        self.bitrate = int(config.get("bitrate", 4_000_000))
        if not 1 <= self.fps <= 120 or not 100_000 <= self.bitrate <= 100_000_000:
            raise ValueError("invalid encoding fps or bitrate")
        self.storage = Path(config.get("storage_dir", "/home/john/camera-media"))
        self.frame_timeout = float(config.get("frame_timeout_s", 2.0))
        self.photo_timeout = float(config.get("photo_timeout_s", 3.0))
        self.encoder_timeout = float(config.get("encoder_timeout_s", 3.0))
        if min(self.frame_timeout, self.photo_timeout, self.encoder_timeout) <= 0:
            raise ValueError("frame, photo, and encoder timeouts must be positive")
        self._condition = threading.Condition(threading.RLock())
        self._record_control = threading.RLock()
        self._latest: _Frame | None = None
        self._pending: _Frame | None = None
        self._sequence = 0
        self._image_count = 0
        self._last_error = ""
        self._streaming = True
        self._stop = threading.Event()
        self._worker = None
        self._loop_thread = None
        self._pipeline = None
        self._encoder = None
        self._source = None
        self._shape = None
        self._encoded_caps = None
        self._rtsp_sources: dict[int, dict] = {}
        self._record: dict | None = None
        self._last_record_path = ""
        self._last_encoded_mono = 0.0
        self._encoder_started_mono = 0.0
        self._start_mono_ns = 0
        self._last_pts = 0
        self._server_source_id = 0
        self._bus_watch_id = 0

    def _error(self, message):
        self._last_error = str(message)
        if self.logger:
            self.logger.error(str(message))

    def start(self):
        if self._worker:
            return
        import gi
        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtspServer", "1.0")
        from gi.repository import GLib, Gst, GstRtspServer
        self.Gst, self.GLib = Gst, GLib
        Gst.init(None)
        for name in ("appsrc", "appsink", "videoconvert", "nvvidconv",
                     f"nvv4l2{self.codec}enc", f"{self.codec}parse",
                     f"rtp{self.codec}pay", "matroskamux"):
            if not Gst.ElementFactory.find(name):
                raise RuntimeError(f"required GStreamer plugin unavailable: {name}")
        self.storage.mkdir(parents=True, exist_ok=True)
        self._loop = GLib.MainLoop()
        self._server = GstRtspServer.RTSPServer.new()
        self._server.connect("client-connected", self._on_client_connected)
        self._server.set_address(str(self.config.get("rtsp_bind", "0.0.0.0")))
        self._server.set_service(str(int(self.config.get("rtsp_port", 8554))))
        self._factory = GstRtspServer.RTSPMediaFactory.new()
        self._factory.set_shared(True)
        self._factory.set_eos_shutdown(False)
        self._factory.set_suspend_mode(GstRtspServer.RTSPSuspendMode.NONE)
        self._factory.set_launch(
            f"( appsrc name=video_source is-live=true format=time block=false "
            f"max-bytes=2097152 leaky-type=downstream "
            f"caps=\"video/x-{self.codec},stream-format=byte-stream,alignment=au\" "
            f"! {self.codec}parse config-interval=-1 "
            f"! rtp{self.codec}pay name=pay0 pt=96 config-interval=-1 mtu=1200 )"
        )
        self._factory.connect("media-configure", self._on_media_configure)
        mount = str(self.config.get("rtsp_path", "/rgb"))
        if not mount.startswith("/"):
            raise ValueError("rtsp_path must start with /")
        self._server.get_mount_points().add_factory(mount, self._factory)
        # Default RTSP protocols allow both unicast RTP/UDP and interleaved TCP.
        self._server_source_id = self._server.attach(None)
        if not self._server_source_id:
            raise RuntimeError("cannot bind RTSP server (address or port in use)")
        self._loop_thread = threading.Thread(target=self._loop.run, daemon=True,
                                             name="camera-glib")
        self._loop_thread.start()
        self._stop.clear()
        self._worker = threading.Thread(target=self._run, daemon=True,
                                        name="camera-encode")
        self._worker.start()

    def submit_frame(self, rgb: bytes, width: int, height: int, stamp_ns: int,
                     frame_id: str, camera_info: dict | None):
        if width < 2 or height < 2 or width % 2 or height % 2:
            self._error("NVENC requires positive, even image dimensions")
            return
        if len(rgb) != width * height * 3:
            self._error("RGB frame byte count does not match dimensions")
            return
        with self._condition:
            self._sequence += 1
            frame = _Frame(bytes(rgb), width, height, int(stamp_ns), str(frame_id),
                           copy.deepcopy(camera_info), self._sequence,
                           time.monotonic(), time.time_ns() // 1000)
            self._latest = self._pending = frame
            self._condition.notify_all()

    def status(self) -> dict:
        now = time.monotonic()
        with self._condition:
            frame = self._latest
            record = self._record
            fresh = bool(frame and now - frame.received_mono < self.frame_timeout)
            encoded = bool(self._last_encoded_mono and
                           now - self._last_encoded_mono < self.frame_timeout)
            state = dict(
                ready=fresh and encoded, streaming=self._streaming and fresh and encoded,
                recording=bool(record and record["started"].is_set()),
                recording_started_mono=(record["start_mono"] if record else 0.0),
                image_count=self._image_count, width=frame.width if frame else 0,
                height=frame.height if frame else 0, fps=self.fps,
                bitrate=self.bitrate, last_error=self._last_error,
                codec=self.codec,
            )
        try:
            usage = shutil.disk_usage(self.storage)
            state.update(total_mib=usage.total / 1048576,
                         available_mib=usage.free / 1048576)
        except OSError:
            state.update(total_mib=0.0, available_mib=0.0)
        return state

    def _new_path(self, suffix: str) -> Path:
        name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        return self.storage / (name + suffix)

    def capture_photo(self) -> dict:
        # A frame already cached when a capture arrives never satisfies it.
        with self._condition:
            after = self._sequence
            deadline = time.monotonic() + self.photo_timeout
            while not self._stop.is_set():
                if self._latest and self._latest.sequence > after:
                    frame = self._latest
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._error("photo timed out waiting for a fresh RGB frame")
                    return {"path": "", "index": self._image_count,
                            "time_boot_ms": 0, "time_utc_us": 0, "success": False}
                self._condition.wait(remaining)
            else:
                return {"path": "", "index": self._image_count,
                        "time_boot_ms": 0, "time_utc_us": 0, "success": False}
        path = self._new_path(".jpg")
        try:
            self._write_photo(path, frame)
        except Exception as exc:
            self._error(f"photo save failed: {exc}")
            return {"path": str(path), "index": self._image_count,
                    "time_boot_ms": int(frame.received_mono * 1000) & 0xffffffff,
                    "time_utc_us": frame.received_utc_us, "success": False}
        with self._condition:
            index = self._image_count
            self._image_count += 1
        return {"path": str(path), "index": index,
                "time_boot_ms": int(frame.received_mono * 1000) & 0xffffffff,
                "time_utc_us": frame.received_utc_us, "success": True}

    def _write_photo(self, path: Path, frame: _Frame):
        import cv2
        import numpy as np
        rgb = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(
            frame.height, frame.width, 3)
        ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                   [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            raise RuntimeError("JPEG encoder failed")
        sidecar = path.with_suffix(".json")
        metadata = dict(source_stamp_ns=frame.stamp_ns, frame_id=frame.frame_id,
                        source_clock=(frame.camera_info or {}).get(
                            "source_clock", "camera_device_clock_unmapped"),
                        received_utc_us=frame.received_utc_us,
                        received_monotonic_ns=int(frame.received_mono * 1e9),
                        width=frame.width, height=frame.height,
                        encoding="rgb8", camera_info=frame.camera_info,
                        timestamp_note="source_stamp_ns is the original ROS header stamp; "
                                       "received_utc_us is host receipt time")
        jpeg_tmp = path.with_suffix(".jpg.tmp")
        json_tmp = path.with_suffix(".json.tmp")
        try:
            self._atomic_bytes(jpeg_tmp, encoded.tobytes())
            self._atomic_bytes(json_tmp, json.dumps(metadata, indent=2).encode())
            os.replace(json_tmp, sidecar)
            os.replace(jpeg_tmp, path)
        except Exception:
            for candidate in (jpeg_tmp, json_tmp, path, sidecar):
                candidate.unlink(missing_ok=True)
            raise

    @staticmethod
    def _atomic_bytes(path: Path, payload: bytes):
        with path.open("xb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())

    def _pipeline_description(self, width, height):
        # CPU RGB conversion is one bounded copy; NV12 upload + encode use Jetson.
        profile = "0" if self.codec == "h265" else "4"  # Main / High
        return (
            "appsrc name=raw_source is-live=true format=time block=false "
            "max-buffers=2 max-bytes=0 leaky-type=downstream "
            f"caps=video/x-raw,format=RGB,width={width},height={height},framerate={self.fps}/1 "
            "! videoconvert ! video/x-raw,format=NV12 "
            "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
            f"! nvv4l2{self.codec}enc name=encoder bitrate={self.bitrate} "
            f"control-rate=1 num-B-Frames=0 iframeinterval={self.fps} "
            f"idrinterval={self.fps} insert-sps-pps=true preset-level=1 profile={profile} "
            f"! {self.codec}parse config-interval=-1 "
            f"! video/x-{self.codec},stream-format=byte-stream,alignment=au "
            "! appsink name=encoded_sink emit-signals=true sync=false "
            "max-buffers=2 drop=true"
        )

    def _build_encoder(self, frame):
        Gst = self.Gst
        self._last_encoded_mono = 0.0
        if self._pipeline:
            with self._record_control:
                if self._record:
                    self.stop_recording()
            self._pipeline.set_state(Gst.State.NULL)
        self._pipeline = Gst.parse_launch(self._pipeline_description(frame.width, frame.height))
        self._source = self._pipeline.get_by_name("raw_source")
        self._encoder = self._pipeline.get_by_name("encoder")
        self._pipeline.get_by_name("encoded_sink").connect("new-sample", self._on_encoded)
        self._shape = (frame.width, frame.height)
        if not self._start_mono_ns:
            self._start_mono_ns = time.monotonic_ns()
        with self._condition:
            for stream in self._rtsp_sources.values():
                stream["wait_keyframe"] = True
        self._encoder_started_mono = time.monotonic()
        if self._pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("NVENC pipeline could not enter PLAYING")

    def _encoder_stalled(self, now=None):
        """Detect silent stalls only while a running encoder has fresh input."""
        now = time.monotonic() if now is None else now
        with self._condition:
            if (self._pipeline is None or not self._latest or
                    now - self._latest.received_mono >= self.frame_timeout):
                return False
            last_progress = max(self._encoder_started_mono, self._last_encoded_mono)
            return now - last_progress >= self.encoder_timeout

    def _run(self):
        next_frame_at = 0.0
        last_encoded_sequence = 0
        while not self._stop.is_set():
            try:
                self._check_bus()
                if self._encoder_stalled():
                    raise RuntimeError("NVENC produced no output despite fresh RGB input "
                                       f"for {self.encoder_timeout:g} seconds")
                with self._condition:
                    delay = max(0.0, next_frame_at - time.monotonic())
                    if self._pending is None or delay:
                        self._condition.wait(min(delay or 0.1, 0.1))
                        frame = None
                    else:
                        frame, self._pending = self._pending, None
                if frame is None:
                    if (self._record and self._latest and
                            time.monotonic() - self._latest.received_mono >= self.frame_timeout):
                        self.stop_recording()
                        self._error("recording stopped because RGB source became stale")
                    continue
                if frame.sequence == last_encoded_sequence:
                    continue
                if self._shape != (frame.width, frame.height):
                    self._build_encoder(frame)
                payload = self.Gst.Buffer.new_allocate(None, len(frame.rgb), None)
                payload.fill(0, frame.rgb)
                # Never put wall/ROS time directly on a live GStreamer clock.
                pts = max(time.monotonic_ns() - self._start_mono_ns, self._last_pts + 1)
                payload.pts = payload.dts = pts
                payload.duration = self.Gst.SECOND // self.fps
                self._last_pts = pts
                self._remember_frame(pts, frame)
                result = self._source.emit("push-buffer", payload)
                if result != self.Gst.FlowReturn.OK:
                    raise RuntimeError(f"NVENC source rejected frame: {result}")
                next_frame_at = time.monotonic() + 1 / self.fps
                last_encoded_sequence = frame.sequence
            except Exception as exc:
                self._error(f"media pipeline failed: {exc}")
                self._last_encoded_mono = 0.0
                if self._record:
                    self._abort_record(self._record, f"recording interrupted: {exc}")
                if self._pipeline:
                    self._pipeline.set_state(self.Gst.State.NULL)
                    self._pipeline = None
                self._shape = None
                self._stop.wait(1.0)
        if self._pipeline:
            self._pipeline.set_state(self.Gst.State.NULL)
            self._pipeline = None

    def _check_bus(self):
        Gst = self.Gst
        if self._pipeline:
            message = self._pipeline.get_bus().pop_filtered(Gst.MessageType.ERROR)
            if message:
                error, debug = message.parse_error()
                raise RuntimeError(f"{error.message}: {debug}")
        record = self._record
        if record:
            message = record["pipeline"].get_bus().pop_filtered(Gst.MessageType.ERROR)
            if message:
                error, _ = message.parse_error()
                self._abort_record(record, f"recording failed: {error.message}")

    def _request_idr(self):
        if self._encoder:
            try:
                self._encoder.emit("force-IDR")
            except Exception as exc:
                self._error(f"keyframe request failed: {exc}")

    def _on_client_connected(self, _server, client):
        # media-configure runs once for a shared stream, but every new receiver
        # needs a recovery point. Do not reset timestamps for existing clients.
        client.connect("play-request", self._on_client_play)

    def _on_client_play(self, _client, _context):
        self._request_idr()

    def _on_media_configure(self, _factory, media):
        source = media.get_element().get_by_name("video_source")
        source.set_property("min-latency", 0)
        key = id(media)
        with self._condition:
            self._rtsp_sources[key] = dict(source=source, base_pts=None, wait_keyframe=True)
        media.connect("unprepared", self._on_media_unprepared, key)
        self._request_idr()

    def _on_media_unprepared(self, _media, key):
        with self._condition:
            self._rtsp_sources.pop(key, None)

    def _copy_rebased(self, buffer, base_pts):
        result = buffer.copy_deep()
        result.pts = max(0, int(buffer.pts) - base_pts)
        result.dts = (max(0, int(buffer.dts) - base_pts)
                      if buffer.dts != self.Gst.CLOCK_TIME_NONE else result.pts)
        return result

    def _remember_frame(self, pts: int, frame: _Frame):
        """Encoder output keeps the input PTS (no B-frames), so PTS maps an
        access unit back to its source frame. Bounded: a dropped frame leaves
        an orphan entry, which eviction reclaims."""
        with self._condition:
            self._pts_index[int(pts)] = (frame.stamp_ns, frame.frame_id, frame.received_utc_us)
            while len(self._pts_index) > self._pts_index_limit:
                self._pts_index.pop(next(iter(self._pts_index)))

    def _recall_frame(self, pts: int):
        with self._condition:
            return self._pts_index.pop(int(pts), None)

    def _dispatch_encoded(self, data: bytes, keyframe: bool, pts: int):
        """Hand one access unit to the hook; a failing hook must never take
        the encoder, RTSP or a recording down with it."""
        hook = self._on_encoded_hook
        if hook is None:
            return
        meta = self._recall_frame(pts)
        try:
            hook(data, keyframe, meta)
            self.encoded_hook_calls += 1
        except Exception as exc:
            message = f"encoded-frame hook failed: {exc}"
            if message != self._last_error:
                self._error(message)

    def _on_encoded(self, sink):
        Gst = self.Gst
        sample = sink.emit("pull-sample")
        if not sample:
            return Gst.FlowReturn.OK
        buffer = sample.get_buffer()
        keyframe = not buffer.has_flags(Gst.BufferFlags.DELTA_UNIT)
        with self._condition:
            self._last_encoded_mono = time.monotonic()
            self._encoded_caps = sample.get_caps().copy()
            streams = list(self._rtsp_sources.values()) if self._streaming else []
        for stream in streams:
            if stream["wait_keyframe"]:
                if not keyframe:
                    continue
                stream["wait_keyframe"] = False
            if stream["base_pts"] is None:
                stream["base_pts"] = int(buffer.pts)
            stream["source"].emit("push-buffer", self._copy_rebased(buffer, stream["base_pts"]))
        if self._on_encoded_hook is not None:
            self._dispatch_encoded(buffer.extract_dup(0, buffer.get_size()), keyframe, int(buffer.pts))
        # Serialize the nonblocking push with detachment/EOS. Otherwise a stop
        # racing this callback can push after EOS and falsely report a disk error.
        with self._condition:
            record = self._record
            if record and not record["failed"]:
                if record["base_pts"] is None:
                    if not keyframe:
                        return Gst.FlowReturn.OK
                    record["base_pts"] = int(buffer.pts)
                    record["start_mono"] = time.monotonic()
                # A stalled disk must fail explicitly instead of growing memory.
                if record["source"].get_property("current-level-bytes") > 16 * 1024 * 1024:
                    record["failed"] = True
                    self.GLib.idle_add(self._abort_record, record, "recording disk queue exceeded 16 MiB")
                    return Gst.FlowReturn.OK
                flow = record["source"].emit("push-buffer", self._copy_rebased(buffer, record["base_pts"]))
                if flow == Gst.FlowReturn.OK:
                    record["started"].set()
                    record["frames"] += 1
                else:
                    record["failed"] = True
                    self.GLib.idle_add(self._abort_record, record, f"recording write failed: {flow}")
        return Gst.FlowReturn.OK

    def start_recording(self) -> str:
        with self._record_control:
            if self._record:
                return str(self._record["path"])
            if not self.status()["ready"] or not self._encoded_caps:
                raise RuntimeError("cannot record: no fresh encoded RGB stream")
            if shutil.disk_usage(self.storage).free < 64 * 1024 * 1024:
                raise RuntimeError("cannot record: less than 64 MiB free storage")
            path = self._new_path(".mkv")
            pipeline = self.Gst.parse_launch(
                "appsrc name=record_source is-live=true format=time block=false "
                f"max-bytes=16777216 ! {self.codec}parse "
                "! matroskamux name=mux ! filesink name=record_file sync=false"
            )
            source = pipeline.get_by_name("record_source")
            source.set_property("caps", self._encoded_caps)
            pipeline.get_by_name("record_file").set_property("location", str(path))
            record = dict(path=path, pipeline=pipeline, source=source, base_pts=None,
                          started=threading.Event(), start_mono=0.0, frames=0,
                          failed=False, first_frame=self._latest)
            if pipeline.set_state(self.Gst.State.PLAYING) == self.Gst.StateChangeReturn.FAILURE:
                pipeline.set_state(self.Gst.State.NULL)
                raise RuntimeError("recording pipeline could not start")
            with self._condition:
                self._record = record
            self._request_idr()
            if not record["started"].wait(3.0) or record["failed"]:
                self._abort_record(record, "recording timed out waiting for an encoded keyframe")
                raise RuntimeError(self._last_error)
            return str(path)

    def _abort_record(self, record, reason):
        record["failed"] = True
        with self._condition:
            if self._record is record:
                self._record = None
        self._error(reason)
        record["pipeline"].set_state(self.Gst.State.NULL)
        return False  # GLib idle callback is one-shot.

    def stop_recording(self) -> str:
        with self._record_control:
            with self._condition:
                record, self._record = self._record, None
            if not record:
                return self._last_record_path
            pipeline = record["pipeline"]
            record["source"].emit("end-of-stream")
            message = pipeline.get_bus().timed_pop_filtered(
                8 * self.Gst.SECOND, self.Gst.MessageType.EOS | self.Gst.MessageType.ERROR)
            pipeline.set_state(self.Gst.State.NULL)
            if not message or message.type != self.Gst.MessageType.EOS:
                reason = "recording finalization timed out"
                if message:
                    error, _ = message.parse_error()
                    reason = f"recording finalization failed: {error.message}"
                self._error(reason)
                raise RuntimeError(reason)
            first = record["first_frame"]
            metadata = dict(codec=self.codec, bitrate=self.bitrate, nominal_fps=self.fps,
                            frames=record["frames"], source_stamp_ns_at_request=first.stamp_ns,
                            source_clock=(first.camera_info or {}).get(
                                "source_clock", "camera_device_clock_unmapped"),
                            frame_id=first.frame_id, camera_info=first.camera_info,
                            start_monotonic_s=record["start_mono"],
                            duration_s=max(0.0, time.monotonic() - record["start_mono"]),
                            timestamp_note="video PTS uses host monotonic time; "
                                           "source_stamp_ns_at_request is the ROS header stamp")
            self._atomic_bytes(record["path"].with_suffix(".json"),
                               json.dumps(metadata, indent=2).encode())
            self._last_record_path = str(record["path"])
            return self._last_record_path

    def set_streaming(self, enabled: bool):
        with self._condition:
            self._streaming = bool(enabled)
            if enabled:
                for stream in self._rtsp_sources.values():
                    stream["wait_keyframe"] = True
        if enabled:
            self._request_idr()

    def close(self):
        try:
            self.stop_recording()
        except Exception as exc:
            self._error(str(exc))
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._worker:
            self._worker.join(timeout=5)
            self._worker = None
        if self._server_source_id:
            self.GLib.source_remove(self._server_source_id)
            self._server_source_id = 0
        if self._loop_thread:
            self._loop.quit()
            self._loop_thread.join(timeout=2)
            self._loop_thread = None
