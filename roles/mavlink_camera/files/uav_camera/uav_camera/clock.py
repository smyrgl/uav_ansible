"""D555 capture times in UTC, from the clock model the D555 adapter publishes.

The adapter (foxglove role, d555_clock.py) fits the camera's hardware clock to
UTC from the camera's own IMU stream and publishes the model as JSON on
/d555/clock (latched). This module applies it to a frame's header stamp,
which is mid-exposure in the device clock. A model that is invalid, or older
than max_age_s, maps nothing, and callers fall back to host receive time.
No ROS imports, so it is testable anywhere.
"""
import json
import time


def unwrap_near(device_ns, reference_ns, wrap_ns):
    """The representation of a possibly wrapped device time nearest reference."""
    if not wrap_ns:
        return device_ns
    return device_ns + round((reference_ns - device_ns) / wrap_ns) * wrap_ns


class ClockMapping:
    def __init__(self, max_age_s=5.0):
        self.max_age_ns = int(max_age_s * 1e9)
        self.model = None
        self.reason = "no /d555/clock model received"

    def update(self, text):
        try:
            model = json.loads(text)
        except ValueError:
            self.model, self.reason = None, "unreadable /d555/clock message"
            return
        if model.get("valid"):
            self.model, self.reason = model, "ok"
        else:
            self.model, self.reason = None, str(model.get("reason", "model not valid"))

    def to_utc_ns(self, device_ns, now_ns=None):
        model = self.model
        if not model or not device_ns:
            return None
        now = time.time_ns() if now_ns is None else now_ns
        if now - int(model["computed_utc_ns"]) > self.max_age_ns:
            self.reason = "clock model stale"
            return None
        ref = int(model["device_ref_ns"])
        d = unwrap_near(int(device_ns), ref, int(model.get("wrap_ns", 0)))
        return d + int(model["offset_ref_ns"]) + round(float(model["skew_ppm"]) * 1e-6 * (d - ref))
