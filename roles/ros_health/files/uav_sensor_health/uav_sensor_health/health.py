"""Sensor summaries. Only explicit loss of connection produces ERROR.

Missing monitoring evidence is UNKNOWN/WARN; missing/degraded data and all
timing limitations are WARN. These are observability rules, not fusion gates.
"""

from collections import deque
from dataclasses import dataclass, field
import math
import re

from .core import STALE, Assessment, OK, WARN, ERROR


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


def d555_timing(clock, model, now, timeout=3.0):
    """A D555 stream's timing section, given the adapter's d555/clock diagnostic.

    The /d555 relays carry UTC stamps; they are verified only while the
    adapter reports a valid clock model AND the stamps really sit within the
    monitor's tolerance of host UTC (classification "epoch_compatible")."""
    live = model.fresh(now, timeout)
    v = model.values if live else {}
    valid = bool(live and v.get("level") == OK and v.get("timing_validated") == "true")
    if not valid or clock.values.get("classification") != "epoch_compatible" or clock.level == STALE:
        if live and not valid:
            note = v.get("message", "D555 clock model not valid")
            return Assessment(clock.level, f"{clock.message}; {note}",
                              {**clock.values, "clock_model": v.get("reason", "not valid")})
        return clock
    return Assessment(OK, "UTC capture time from the D555 IMU clock model",
                      {**clock.values, "synchronization_verified": True,
                       "clock_model_skew_ppm": v.get("skew_ppm"),
                       "clock_model_residual_us": v.get("residual_rms_us")})


def lidar_health(kind, cloud, driver, clock, now, timeout=3.0):
    live_cloud, live_driver = cloud.fresh(now, timeout), driver.fresh(now, timeout)
    v = driver.values if live_driver else {}
    if kind == "avia":
        connection = True if live_cloud else (v.get("connected") in ("1", "true") if "connected" in v else None)
        # The driver validates its own time base (PPS + pushed UTC, receipt-latency
        # bounds) and says so in avia/clock; anything less is a timing warning.
        live_clock = clock.fresh(now, timeout)
        cv = clock.values if live_clock else {}
        timing_good = bool(live_clock and cv.get("level") == OK and cv.get("timing_validated") == "true")
        timing_message = cv.get("message", "No Avia clock diagnostic") if live_clock else "Avia clock diagnostic missing or stale"
        timing_values = {k: val for k, val in clock.metrics(now).items() if k not in ("message", "level")} if live_clock else {}
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
    sections = {
        "MAVLink": check(live, "PX4 1/1 heartbeat received" if live else "PX4 link unavailable; camera traffic does not count",
                         **heartbeat.metrics(now)),
        "Router": check(transport.get("connected", False), transport.get("message", "Starting observer")),
    }
    # FC barometer (SCALED_PRESSURE): bay air temperature and static pressure.
    # Informational: absent from the stream is not a fault, stale is.
    baro = samples.get("SCALED_PRESSURE")
    if baro is not None:
        temp_c, press = number(baro.values, "temperature") / 100.0, number(baro.values, "press_abs")
        sections["Barometer"] = check(baro.fresh(now, 5), f"FC baro {temp_c:.1f} C, {press:.1f} hPa" if baro.fresh(now, 5)
                                      else "SCALED_PRESSURE stale", temperature_c=round(temp_c, 2), pressure_hpa=round(press, 2),
                                      **baro.metrics(now))
    return grouped(connected, "Autopilot heartbeat live" if live else "No PX4 heartbeat through MAVLink router", sections)


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
        # Informational, not a warning: the H-Flow puts no time in its DroneCAN
        # messages, so host receipt is the best stamp available (accepted
        # 2026-10-02). A flow sample's integration window ends a few ms before it.
        "Timing": check(True, "Host receipt timestamps (the H-Flow sends no DroneCAN time; best available)"),
        "Identity": check(True, str(driver.values.get("hardware_id", "H-Flow DroneCAN node")) + "; observed by uav-hflow on the FC's CAN2 bus, the same frames PX4 fuses"),
    })


def _parse_number(text, default=None):
    found = re.search(r"[-+]?\d+(?:\.\d+)?", str(text))
    return float(found.group()) if found else default


def jetson_identity(entries):
    """Module name from jetson_stats/board/Config, for the row's source id."""
    config = entries.get("jetson_stats/board/Config")
    if config:
        values = config[3]
        return str(values.get("Module") or values.get("Model") or config[4] or "Jetson")
    return "Jetson"


def jetson_health(entries, now, timeout=5.0, temp_warn=84.0, temp_crit=100.0):
    """Fold isaac_ros_jetson_stats rows into one summary.

    entries: name -> (received_mono, level, message, values, hardware_id) as
    captured from /diagnostics. Thermal zones drive the level, judged from the
    reported temperature against the upstream thresholds (84 C WARN, 100 C
    ERROR) rather than the upstream level: NVMe SMART zones carry a bogus
    -256 C critical threshold, which makes the upstream node flag them ERROR
    at room temperature. A genuinely critical zone is the one case where a
    connected device reports ERROR, because it is about to throttle.
    """
    fresh = {name: e for name, e in entries.items() if now - e[0] <= timeout}
    if not fresh:
        if entries:
            age = now - max(e[0] for e in entries.values())
            return grouped(None, f"jetson_stats rows stale for {age:.0f} s (uav-jetson-stats down?)",
                           {"Publisher": check(False, "jetson_stats diagnostics stale")})
        return grouped(None, "No jetson_stats rows observed", {"Publisher": check(False, "jetson_stats diagnostics never received")})

    def group(prefix):
        return {name[len(prefix):]: e for name, e in fresh.items() if name.startswith(prefix)}

    def levels_ok(rows):
        return all(e[1] == OK for e in rows.values())

    temps_raw = group("jetson_stats/temp/")
    temps = {zone: _parse_number(e[2]) for zone, e in temps_raw.items() if not str(e[2]).startswith("Offline")}
    temps = {zone: t for zone, t in temps.items() if t is not None}
    if "tj" in temps:
        hottest = ("tj", temps["tj"])
    elif temps:
        hottest = max(temps.items(), key=lambda kv: kv[1])
    else:
        hottest = None
    critical = any(t >= temp_crit for t in temps.values())
    thermal = check(bool(temps) and all(t < temp_warn for t in temps.values()),
                    ("THERMAL CRITICAL: " if critical else "") + (f"{hottest[0]} {hottest[1]:.1f} C" if hottest else "no thermal zones"),
                    **{f"{zone}_c": round(t, 1) for zone, t in sorted(temps.items())})

    power = group("jetson_stats/power/")
    total = None
    for rail in ("VDD_IN", "TOT", "POM_5V_IN", "VDD_GPU_SOC"):
        if rail in power:
            total = (rail, power[rail]); break
    if total is None and power:
        total = next(iter(power.items()))
    def watts(entry):
        text = str(entry[3].get("Power", entry[2]))
        value = _parse_number(text)
        return None if value is None else (value / 1000.0 if "mW" in text else value)
    total_w = watts(total[1]) if total else None
    power_section = check(bool(power) and levels_ok(power), f"{total[0]} {total_w:.1f} W" if total_w is not None else "no power rails",
                          **{rail: str(e[3].get("Power", e[2])).strip() for rail, e in sorted(power.items())})

    cpus = group("jetson_stats/cpu/")
    loads = [v for v in (_parse_number(e[2]) for e in cpus.values() if not str(e[2]).strip().startswith("OFF")) if v is not None]
    gpus = group("jetson_stats/gpu/")
    gpu_load = next((v for v in (_parse_number(e[2]) for e in gpus.values()) if v is not None), None)
    compute_text = ", ".join(part for part in (
        f"CPU {sum(loads) / len(loads):.0f} % x{len(loads)}" if loads else None,
        f"GPU {gpu_load:.0f} %" if gpu_load is not None else None) if part)
    compute = check(bool(cpus or gpus) and levels_ok(cpus) and levels_ok(gpus), compute_text or "no load rows",
                    **{f"cpu{n}": str(e[2]).strip() for n, e in sorted(cpus.items())},
                    **{f"gpu_{n}": str(e[2]).strip() for n, e in gpus.items()})

    memory = group("jetson_stats/mem/")
    mem = check(levels_ok(memory), str(memory["RAM"][2]).strip() if "RAM" in memory else (
        str(memory["ram"][2]).strip() if "ram" in memory else "no memory rows"), **{k: str(e[2]).strip() for k, e in sorted(memory.items())})

    fans = group("jetson_stats/fan/")
    fan_text = ", ".join(f"{name} {str(e[2]).strip()}" for name, e in sorted(fans.items())) or "no fan rows"
    fan = check(levels_ok(fans), fan_text, **{f"{name}/{v}": val for name, e in fans.items() for v, val in e[3].items()})

    board = fresh.get("jetson_stats/board/Status")
    config = fresh.get("jetson_stats/board/Config")
    board_values = {**({k: v for k, v in board[3].items()} if board else {}),
                    **({k: config[3][k] for k in ("Model", "Module", "Jetpack", "L4T") if k in config[3]} if config else {})}
    mode = board[3].get("NV Power-Mode") if board else None
    board_section = check(board is not None and board[1] == OK, board[2] if board else "no board status", **board_values)

    summary = ", ".join(part for part in (
        thermal.message, power_section.message if total_w is not None else None,
        f"fan {str(next(iter(fans.values()))[2]).strip()}" if fans else None,
        compute_text or None, mode) if part)
    result = grouped(True, summary, {"Thermal": thermal, "Power": power_section, "Compute": compute,
                                     "Memory": mem, "Fan": fan, "Board": board_section})
    if critical:
        try:
            result.level = ERROR
        except Exception:
            result = Assessment(ERROR, result.message, result.values)
    return result
