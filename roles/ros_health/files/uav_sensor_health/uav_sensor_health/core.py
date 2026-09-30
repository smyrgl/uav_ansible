"""Bounded metadata-only checks; receipt freshness uses monotonic time."""

from collections import deque
from dataclasses import dataclass
import math

OK, WARN, ERROR, STALE = 0, 1, 2, 3
EPOCH_FLOOR_SEC = 946684800  # 2000-01-01; distinguishes this bench's device uptime.


@dataclass(frozen=True)
class ImageMetadata:
    """Per-frame facts. From an Image every field is known; from a CameraInfo
    header (same stamp, frame and size as the image it accompanies, at a few
    hundred bytes instead of a megabyte per frame) step, encoding and byte
    count are None and their checks are skipped. The D555 sends every
    subscriber its own unicast copy of each image, so the health observer
    reads CameraInfo and leaves the image bandwidth to the consumers."""
    stamp_ns: int
    width: int
    height: int
    frame_id: str
    step: int | None = None
    encoding: str | None = None
    data_bytes: int | None = None

    @property
    def header_only(self):
        return self.data_bytes is None

    def layout_error(self):
        if self.width <= 0 or self.height <= 0:
            return "Invalid image dimensions"
        if self.step is not None and self.step < self.width:
            return "Invalid image row stride"
        if self.encoding is not None and not self.encoding:
            return "Image encoding is empty"
        if self.data_bytes is not None and self.step is not None and self.data_bytes != self.height * self.step:
            return "Image byte count does not match height times row stride"
        return None


@dataclass(frozen=True)
class Assessment:
    level: int
    message: str
    values: dict


def classify_clock(stamp_ns, wall_sec, tolerance_sec):
    """Epoch proximity is observable; synchronization is not proven by it."""
    stamp_sec = stamp_ns / 1e9
    if stamp_ns <= 0:
        return "unset", "Missing or invalid image timestamp"
    if stamp_sec < EPOCH_FLOOR_SEC:
        return "device_clock", "Device/non-epoch clock; UTC synchronization unverified"
    age = wall_sec - stamp_sec
    if age < -tolerance_sec:
        return "future", "Image timestamp is ahead of host wall clock"
    if age > tolerance_sec:
        return "past", "Image timestamp is behind host wall clock"
    return "epoch_compatible", "Near host wall time; synchronization unverified"


class StreamMonitor:
    def __init__(self, started_mono, *, timeout_sec=2.0, startup_grace_sec=10.0,
                 min_rate_hz=15.0, rate_window_sec=5.0, clock_tolerance_sec=2.0):
        limits = (timeout_sec, startup_grace_sec, min_rate_hz,
                  rate_window_sec, clock_tolerance_sec)
        if not all(math.isfinite(v) for v in limits):
            raise ValueError("Health thresholds must be finite")
        if min(timeout_sec, rate_window_sec, clock_tolerance_sec) <= 0:
            raise ValueError("Timeout, rate window and clock tolerance must be positive")
        if min(startup_grace_sec, min_rate_hz) < 0:
            raise ValueError("Grace period and minimum rate must be nonnegative")
        self.started_mono = started_mono
        self.timeout_sec = timeout_sec
        self.startup_grace_sec = startup_grace_sec
        self.min_rate_hz = min_rate_hz
        self.rate_window_sec = rate_window_sec
        self.clock_tolerance_sec = clock_tolerance_sec
        self.receipts = deque(maxlen=4096)
        self.last = None
        self.last_received = None
        self.last_stamp_advance = None
        self.last_regression = None
        self.regressions = 0
        self.total = 0

    def observe(self, metadata, received_mono):
        if self.last is None:
            self.last_stamp_advance = received_mono
        elif metadata.stamp_ns > self.last.stamp_ns:
            self.last_stamp_advance = received_mono
        elif metadata.stamp_ns < self.last.stamp_ns:
            self.regressions += 1
            self.last_regression = received_mono
            self.last_stamp_advance = received_mono
        self.last = metadata
        self.last_received = received_mono
        self.total += 1
        self.receipts.append(received_mono)
        self._prune(received_mono)

    def _prune(self, now_mono):
        while self.receipts and self.receipts[0] < now_mono - self.rate_window_sec:
            self.receipts.popleft()

    def assess(self, now_mono, wall_sec):
        self._prune(now_mono)
        values = {"received_total": self.total, "timeout_sec": self.timeout_sec,
                  "freshness_clock": "monotonic receipt", "content_validated": False}
        if self.last is None:
            in_grace = now_mono - self.started_mono < self.startup_grace_sec
            level = WARN if in_grace else STALE
            message = "Waiting for first image" if in_grace else "No image received"
            return (Assessment(level, message, values),
                    Assessment(level, "No image timestamp to inspect", {}))

        age = max(0.0, now_mono - self.last_received)
        span = now_mono - self.receipts[0] if self.receipts else 0.0
        rate = (len(self.receipts) - 1) / span if span > 0 else 0.0
        values.update(receipt_age_sec=round(age, 3), rate_hz=round(rate, 2),
                      rate_window_sec=self.rate_window_sec, min_rate_hz=self.min_rate_hz,
                      width=self.last.width, height=self.last.height,
                      frame_id=self.last.frame_id,
                      source="camera_info header" if self.last.header_only else "image")
        if not self.last.header_only:
            values.update(encoding=self.last.encoding, row_step=self.last.step,
                          image_bytes=self.last.data_bytes)
        noun = "frame headers" if self.last.header_only else "images"
        layout_error = self.last.layout_error()
        if age > self.timeout_sec:
            stream = Assessment(STALE, "Image stream stale", values)
        elif layout_error:
            stream = Assessment(WARN, layout_error, values)
        elif not self.last.frame_id:
            stream = Assessment(WARN, f"Receiving {noun}; frame_id is empty", values)
        elif span >= 1.0 and rate < self.min_rate_hz:
            stream = Assessment(WARN, f"Receiving {noun} below expected rate", values)
        else:
            stream = Assessment(OK, f"Receiving fresh {noun}", values)

        state, message = classify_clock(self.last.stamp_ns, wall_sec,
                                        self.clock_tolerance_sec)
        clock_values = {"classification": state, "header_stamp_ns": self.last.stamp_ns,
                        "host_minus_header_sec": round(wall_sec - self.last.stamp_ns / 1e9, 6),
                        "clock_tolerance_sec": self.clock_tolerance_sec,
                        "timestamp_regressions": self.regressions,
                        "synchronization_verified": False}
        if age > self.timeout_sec:
            clock = Assessment(STALE, "No fresh image timestamp", clock_values)
        elif now_mono - self.last_stamp_advance > self.timeout_sec:
            clock = Assessment(WARN, "Images arriving but timestamp is not advancing", clock_values)
        elif self.last_regression is not None and now_mono - self.last_regression < self.timeout_sec:
            clock = Assessment(WARN, "Image timestamp moved backwards; clock reset or reordering", clock_values)
        else:
            clock = Assessment(WARN, message, clock_values)
        return stream, clock
