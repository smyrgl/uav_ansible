#!/usr/bin/env python3
"""D555 hardware clock to UTC, estimated host-side from the camera's IMU stream.

The D555 stamps every sample in its own hardware clock (time since power-on).
Over PoE it has no PTP client (firmware 7.58 sends no PTP at all) and hardware
sync is USB-only, so the clock relation is estimated on the host, as
librealsense's "global time" does. Each IMU sample (100 Hz, a few hundred
bytes) carries two timestamps that matter here:

  * the DDS source timestamp: the camera's clock when its writer sent it;
  * the DDS receive timestamp: host UTC (chrony, locked to the GNSS PPS).

receive - send = (UTC - device offset) + transport latency, and latency only
ever adds. So the smallest difference within each one-second bin is the
closest look at the offset, and a line through those bin minima over a
sliding window gives offset and skew. What it cannot see is the minimum
one-way latency itself (the camera's DDS writer to the host's reader, of the
order of half the 0.3 ms ping round trip): mapped times are late by about
that much, never early.

Device time may wrap: librealsense treats D555 device time as a 32-bit
microsecond counter (period 4294.967296 s). A jump from just below the
period to just above zero is unwrapped; any other backwards jump is a camera
restart and starts a new model.
"""
from collections import deque
import math

NS_PER_S = 1_000_000_000
WRAP_32BIT_US_NS = (1 << 32) * 1000
_WRAP_MARGIN_NS = 60 * NS_PER_S


def unwrap_near(device_ns, reference_ns, wrap_ns):
    """The representation of a possibly wrapped device time nearest reference."""
    if not wrap_ns:
        return device_ns
    return device_ns + round((reference_ns - device_ns) / wrap_ns) * wrap_ns


def map_device_ns(model, device_ns):
    """UTC nanoseconds for a device time, from a published model dictionary."""
    d = unwrap_near(int(device_ns), model["device_ref_ns"], model["wrap_ns"])
    return d + model["offset_ref_ns"] + round(model["skew_ppm"] * 1e-6 * (d - model["device_ref_ns"]))


def _linear_fit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return my, 0.0
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - slope * mx, slope


class ClockModel:
    """Lower-envelope fit of (receive - send) against send time."""

    def __init__(self, window_s=60.0, bin_s=1.0, min_bins=10, max_residual_s=0.0005,
                 max_skew_ppm=200.0, stale_s=3.0, reset_jump_s=2.0, wrap_ns=WRAP_32BIT_US_NS):
        if not (window_s > bin_s > 0 and min_bins >= 3 and max_residual_s > 0 and stale_s > 0):
            raise ValueError("invalid clock model configuration")
        self.bin_ns = int(bin_s * NS_PER_S)
        self.window_bins = int(window_s / bin_s)
        self.min_bins = int(min_bins)
        self.max_residual_ns = max_residual_s * NS_PER_S
        self.max_skew_ppm = float(max_skew_ppm)
        self.stale_ns = int(stale_s * NS_PER_S)
        self.reset_jump_ns = int(reset_jump_s * NS_PER_S)
        self.wrap_ns = int(wrap_ns or 0)
        self.samples = 0
        self.wraps = 0
        self.resets = 0
        self._new_epoch()

    def _new_epoch(self):
        self._bins = deque()     # (bin index, device_ns, delay_ns): per-bin minimum delay
        self._unwrap_ns = 0
        self._last_raw = None
        self._last_device = None
        self._last_receive = None
        self.fit = None
        self.reason = "no IMU samples yet"

    def add(self, device_ns, receive_utc_ns):
        """One sample: device send time (device clock) and host receive time (UTC)."""
        device_ns, receive_utc_ns = int(device_ns), int(receive_utc_ns)
        if device_ns <= 0 or receive_utc_ns <= 0:
            return False
        d = device_ns + self._unwrap_ns
        if self._last_device is not None:
            if (self.wrap_ns and self._last_raw > self.wrap_ns - _WRAP_MARGIN_NS
                    and device_ns < _WRAP_MARGIN_NS):
                self._unwrap_ns += self.wrap_ns
                self.wraps += 1
                d = device_ns + self._unwrap_ns
            elif d < self._last_device - self.reset_jump_ns:
                self.resets += 1
                self._new_epoch()
                d = device_ns
        self._last_raw, self._last_device, self._last_receive = device_ns, d, receive_utc_ns
        delay = receive_utc_ns - d
        index = d // self.bin_ns
        if self._bins and self._bins[-1][0] == index:
            if delay < self._bins[-1][2]:
                self._bins[-1] = (index, d, delay)
        elif not self._bins or index > self._bins[-1][0]:
            self._bins.append((index, d, delay))
        while self._bins and self._bins[0][0] <= index - self.window_bins:
            self._bins.popleft()
        self.samples += 1
        return True

    def solve(self, now_utc_ns):
        """Refit from the current window. Returns the model dictionary."""
        self.fit = None
        if self._last_receive is None:
            self.reason = "no IMU samples yet"
        elif now_utc_ns - self._last_receive > self.stale_ns:
            self.reason = "IMU samples stale"
        elif len(self._bins) < self.min_bins:
            self.reason = f"collecting: {len(self._bins)} of {self.min_bins} one-second bins"
        else:
            self._fit()
        return self.as_dict(now_utc_ns)

    def _fit(self):
        bins = list(self._bins)
        d_ref, delay_ref = bins[-1][1], bins[-1][2]
        xs = [(b[1] - d_ref) / NS_PER_S for b in bins]
        ys = [float(b[2] - delay_ref) for b in bins]
        keep = list(range(len(bins)))
        for _ in range(3):
            c0, c1 = _linear_fit([xs[i] for i in keep], [ys[i] for i in keep])
            res = [ys[i] - (c0 + c1 * xs[i]) for i in range(len(bins))]
            spread = sorted(abs(res[i]) for i in keep)[len(keep) // 2] * 1.4826
            # A bin with no fast sample (a congested second) sits high above the
            # envelope; nothing can sit far below it.
            limit = 3.0 * max(spread, 20_000.0)
            new_keep = [i for i in range(len(bins)) if abs(res[i]) <= limit]
            if len(new_keep) < self.min_bins or new_keep == keep:
                break
            keep = new_keep
        rms = math.sqrt(sum(res[i] ** 2 for i in keep) / len(keep))
        skew_ppm = c1 / 1e3   # ns per s -> parts per million
        if rms > self.max_residual_ns:
            self.reason = f"fit residual {rms / 1e3:.0f} us exceeds {self.max_residual_ns / 1e3:.0f} us"
            return
        if abs(skew_ppm) > self.max_skew_ppm:
            self.reason = f"skew {skew_ppm:.1f} ppm is implausible"
            return
        # Seat the line on the low side of the bin minima: the envelope, not their mean.
        low = sorted(res[i] for i in keep)[len(keep) // 10]
        self.fit = {"device_ref_ns": d_ref, "offset_ref_ns": int(round(delay_ref + c0 + low)),
                    "skew_ppm": skew_ppm, "residual_rms_us": rms / 1e3,
                    "bins": len(bins), "bins_used": len(keep)}
        self.reason = "ok"

    def to_utc_ns(self, device_ns):
        if not self.fit:
            return None
        return map_device_ns({**self.fit, "wrap_ns": self.wrap_ns}, device_ns)

    def as_dict(self, now_utc_ns):
        out = {"valid": self.fit is not None, "reason": self.reason, "wrap_ns": self.wrap_ns,
               "samples": self.samples, "wraps": self.wraps, "resets": self.resets,
               "bins": len(self._bins), "computed_utc_ns": int(now_utc_ns),
               "source": "D555 IMU: DDS source timestamp (device clock) vs DDS receive timestamp (UTC)",
               "bias_note": "mapped times are late by the minimum one-way transport latency (sub-millisecond)"}
        if self.fit:
            out.update(self.fit)
        return out
