"""Sensor summaries. Only explicit loss of connection produces ERROR.

Missing monitoring evidence is UNKNOWN/WARN; missing/degraded data and all
timing limitations are WARN. These are observability rules, not fusion gates.
"""

from collections import deque
from dataclasses import dataclass, field
import math

from .core import Assessment, OK, WARN, ERROR


def check(good, message, /, **values):
    return Assessment(OK if good else WARN, message, values)


def grouped(connection, message, sections):
    values = {"Connection/state": {True: "connected", False: "disconnected",
                                   None: "unknown"}[connection]}
    warnings = []
    for name, assessment in sections.items():
        # Raw diagnostics may use ERROR for invalid timing. Never propagate it
        # to a connected sensor's summary.
        level = OK if assessment.level == OK else WARN
        values[f"{name}/status"] = "OK" if level == OK else "WARN"
        values[f"{name}/message"] = assessment.message
        values.update({f"{name}/{key}": val for key, val in assessment.values.items()})
        if level != OK:
            warnings.append(f"{name}: {assessment.message}")
    values["Attention"] = "; ".join(warnings) if warnings else "None"
    level = ERROR if connection is False else WARN if connection is None or warnings else OK
    return Assessment(level, message, values)


@dataclass
class Sample:
    """Bounded receipt and stamp-progress evidence; no sensor payload retained."""
    received: float = -1.0
    progress: float = -1.0
    stamp: object = None
    values: dict = field(default_factory=dict)
    receipts: deque = field(default_factory=lambda: deque(maxlen=512))

    def observe(self, now, values, stamp=None):
        if self.received < 0 or stamp is None or stamp != self.stamp:
            self.progress = now
        self.received, self.stamp, self.values = now, stamp, dict(values)
        self.receipts.append(now)

    def age(self, now):
        return None if self.received < 0 else max(0.0, now - self.received)

    def fresh(self, now, timeout=3.0):
        age = self.age(now)
        return age is not None and age <= timeout

    def advancing(self, now, timeout=3.0):
        return self.progress >= 0 and now - self.progress <= timeout

    def metrics(self, now):
        while self.receipts and now - self.receipts[0] > 5:
            self.receipts.popleft()
        span = now - self.receipts[0] if self.receipts else 0
        rate = max(0, len(self.receipts) - 1) / span if span > 0 else 0
        age = self.age(now)
        return {**self.values, "receipt_age_sec": round(age, 3) if age is not None else "never",
                "observed_rate_hz": round(rate, 2)}


def number(values, key, default=-1):
    try:
        value = float(values[key])
        return value if math.isfinite(value) else default
    except (KeyError, ValueError, TypeError):
        return default


def lidar_health(kind, cloud, driver, clock, now, timeout=3.0):
    live_cloud, live_driver = cloud.fresh(now, timeout), driver.fresh(now, timeout)
    v = driver.values if live_driver else {}
    if kind == "avia":
        connection = True if live_cloud else (v.get("connected") in ("1", "true") if "connected" in v else None)
        timing_good = False  # Current driver intentionally stamps host receipt.
        timing_message = "Host receipt timestamps; hardware sync unverified"
        timing_values = {k: val for k, val in clock.metrics(now).items() if k not in ("message", "level")} if clock.fresh(now, timeout) else {}
    else:
        packet_ages = [number(v, k) for k in ("msop_age_sec", "difop_age_sec")]
        any_packets = any(0 <= age <= timeout for age in packet_ages)
        connection = True if live_cloud or any_packets or v.get("connected") in ("1", "true") else False if live_driver and v else None
        timing_good = (live_driver and number(v, "difop_time_mode") == 3
                       and number(v, "difop_sync_status") == 1
                       and number(v, "msop_time_mode") == 3
                       and number(v, "utc_offset_verified") == 1
                       and all(0 <= age <= timeout for age in packet_ages)
                       and driver.values.get("level") == OK)
        timing_message = "gPTP valid; UTC offset verified" if timing_good else v.get("message", "No current gPTP evidence")
        timing_values = {k: val for k, val in v.items() if "time" in k or "offset" in k or "sync" in k}
    points = number(cloud.values, "points", 0)
    shape_ok = cloud.values.get("layout_valid", False)
    data_good = live_cloud and points > 0 and shape_ok and bool(cloud.values.get("frame_id"))
    data_message = "Fresh nonempty point clouds" if data_good else "No fresh point clouds" if not live_cloud else "Empty or malformed point cloud"
    if data_good and driver.values.get("level") not in (None, OK) and live_driver:
        data_good, data_message = False, driver.values.get("message", "Driver reports degraded data")
    message = "Disconnected" if connection is False else "Connection unknown; driver evidence missing" if connection is None else (
        f"Point clouds live; {cloud.metrics(now)['observed_rate_hz']} Hz" if data_good else "Connected; " + data_message.lower())
    if connection is True and not timing_good:
        message += "; timing warning"
    return grouped(connection, message, {
        "Data": check(data_good, data_message, **cloud.metrics(now)),
        "Timing": check(timing_good, timing_message, **timing_values),
        "Driver": check(live_driver and driver.values.get("level") == OK,
                        driver.values.get("message", "No driver diagnostics") if live_driver else "Driver diagnostics missing or stale",
                        **{k: val for k, val in driver.metrics(now).items() if k not in ("message", "good")}),
    })


def px4_health(samples, transport, now, started, grace=10.0):
    heartbeat = samples.get("HEARTBEAT", Sample())
    live = heartbeat.fresh(now, 3)
    # A router TCP connection or camera heartbeat does NOT prove PX4 is present.
    connected = True if live else None if now - started < grace else False
    return grouped(connected, "Autopilot heartbeat live" if live else "No PX4 heartbeat through MAVLink router", {
        "MAVLink": check(live, "PX4 1/1 heartbeat received" if live else "PX4 link unavailable; camera traffic does not count",
                         **heartbeat.metrics(now)),
        "Router": check(transport.get("connected", False), transport.get("message", "Starting observer")),
    })


def hflow_health(flow, distance, driver, now, quality_min=20):
    """H-Flow as seen by the Jetson's own DroneCAN listener (uav-hflow), not via PX4.

    flow: /hflow/sensor_optical_flow samples; distance: /hflow/range samples;
    driver: the listener's hflow/driver diagnostics (CAN state, rates).
    """
    live_flow = flow.fresh(now, 4) and flow.advancing(now, 4)
    live_distance = distance.fresh(now, 4) and distance.advancing(now, 4)
    flow_ok = (live_flow and number(flow.values, "quality", 0) >= quality_min
               and number(flow.values, "integration_timespan_us", 0) > 0
               and bool(flow.values.get("pixel_flow_finite", False)))
    range_m = number(distance.values, "range_m", math.nan)
    range_ok = (live_distance and math.isfinite(range_m)
                and number(distance.values, "min_range", 0) <= range_m <= number(distance.values, "max_range", 0))
    driver_live = driver.fresh(now, 4)
    can_up = str(driver.values.get("can_operstate", "")) == "up"
    if live_flow or live_distance:
        connection = True
    elif driver_live and not can_up:
        connection = False          # the listener is running and says the bus is down
    else:
        connection = None           # no evidence either way
    text = ("Flow + range live" if flow_ok and range_ok else "Connected; flow or range degraded" if connection
            else "CAN interface down" if connection is False else "Unknown; no fresh flow/range frames on the CAN listener")
    if range_ok:
        text += f"; range {range_m:.2f} m"
    bus_text = ("%s up, listen-only; flow %s Hz, range %s Hz" % (driver.values.get("can_interface", "can0"),
                driver.values.get("flow_hz", "?"), driver.values.get("range_hz", "?")) if driver_live and can_up
                else "CAN listener diagnostics missing or interface down")
    return grouped(connection, text, {
        "Optical flow": check(flow_ok, "Fresh flow; quality acceptable" if flow_ok else "Missing, frozen or low quality flow",
                              **flow.metrics(now), minimum_quality=quality_min),
        "Range": check(range_ok, "Fresh in-range measurement" if range_ok else "Missing, frozen or invalid range", **distance.metrics(now)),
        "Bus": check(driver_live and can_up, bus_text, **{k: v for k, v in driver.values.items() if k.startswith(("can_", "node_", "other_node"))}),
        "Timing": check(False, "Host receipt timestamps; the H-Flow sends no DroneCAN time (no bus time sync)"),
        "Identity": check(True, str(driver.values.get("hardware_id", "H-Flow DroneCAN node")) + "; observed by uav-hflow on the FC's CAN2 bus, the same frames PX4 fuses"),
    })
