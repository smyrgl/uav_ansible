"""Bounded, non-blocking-to-ROS observers for existing read-only interfaces.

MAVLink requests only current telemetry (REQUEST_MESSAGE), never sets message
rates, parameters, modes or actuator state. GNSS uses its read-only broker,
never the receiver's serial device.
"""

import binascii
import copy
import datetime
import json
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time

from .health import Sample, check, grouped


PVT_MODES = {0: "no PVT", 1: "stand-alone", 2: "differential", 3: "fixed location", 4: "RTK fixed",
             5: "RTK float", 6: "SBAS aided", 7: "moving-base RTK fixed", 8: "moving-base RTK float", 10: "PPP"}
RTK_FIXED_MODES = (4, 7)


def parse_pvt_geodetic(block):
    """SBF PVTGeodetic (4007), revision-2 layout: the fields the health row
    needs. Do-Not-Use values (65535) become None. Returns (values, stamp)."""
    mode_byte, error = block[14], block[15]
    mode = mode_byte & 15
    values = {"mode": mode, "mode_text": PVT_MODES.get(mode, f"mode {mode}"), "error": error,
              "satellites": block[74], "fix_2d": bool(mode_byte & 0x80)}
    reference_id, corr_age = struct.unpack_from("<HH", block, 76)
    values["reference_id"] = reference_id
    values["mean_corr_age_sec"] = None if corr_age == 65535 else round(corr_age / 100.0, 2)
    if len(block) >= 94:
        h_acc, v_acc = struct.unpack_from("<HH", block, 90)
        values["h_accuracy_m"] = None if h_acc == 65535 else round(h_acc / 100.0, 2)
        values["v_accuracy_m"] = None if v_acc == 65535 else round(v_acc / 100.0, 2)
    return values, struct.unpack_from("<IH", block, 8)


def sbf_blocks(buf):
    """Recover complete CRC-valid SBF blocks; bound incomplete frames to 8192 B."""
    blocks, i = [], 0
    while True:
        i = buf.find(b"$@", i)
        if i < 0:
            return blocks, buf[-1:]
        if len(buf) < i + 8:
            return blocks, buf[i:]
        crc, bid, size = struct.unpack_from("<HHH", buf, i + 2)
        if size < 8 or size > 8192 or size % 4:
            i += 2
            continue
        if len(buf) < i + size:
            return blocks, buf[i:]
        block = buf[i:i + size]
        if binascii.crc_hqx(block[4:], 0) == crc:
            blocks.append((bid & 0x1fff, block))
            i += size
        else:
            i += 2


def chrony_assessment(tracking, sources, wall):
    v = {k.strip(): val.strip() for line in tracking.splitlines()
         if ":" in line for k, val in [line.split(":", 1)]}
    try:
        ref = datetime.datetime.strptime(v["Ref time (UTC)"], "%a %b %d %H:%M:%S %Y").replace(tzinfo=datetime.timezone.utc).timestamp()
        age = wall - ref
        offset = abs(float(v["System time"].split()[0]))
        dispersion = float(v["Root dispersion"].split()[0])
        locked = ("(PPS)" in v.get("Reference ID", "") and v.get("Leap status") == "Normal"
                  and any(line.split()[:2] == ["#*", "PPS"] for line in sources.splitlines())
                  and 0 <= age < 120 and offset < .01 and dispersion < .01)
        return check(locked, "GNSS PPS selected; host clock disciplined" if locked else "PPS not selected, stale or outside 10 ms bench tolerance",
                     reference=v.get("Reference ID", "unknown"), leap=v.get("Leap status", "unknown"),
                     reference_age_sec=round(age, 2), system_offset_sec=offset,
                     root_dispersion_sec=dispersion)
    except (KeyError, ValueError):
        return check(False, "Unable to interpret chrony status")


def ptp_assessment(data, boot_id, mono_ns):
    age = (mono_ns - int(data.get("updated_monotonic_ns", 0))) / 1e9
    current = (data.get("schema_version") == 1 and data.get("boot_id") == boot_id and 0 <= age < 5)
    verified = current and data.get("utc_offset_verified") is True and data.get("ptp_timescale") is True
    return check(verified, "PHC/UTC offset verified" if verified else "PHC/UTC verification missing, stale or invalid",
                 evidence_age_sec=round(age, 3), port_state=data.get("port_state"),
                 utc_offset=data.get("current_utc_offset"),
                 utc_offset_advertised_valid=data.get("utc_offset_valid"),
                 reason=data.get("reason", ""))


class Observers:
    def __init__(self, params):
        self.params = params
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.samples = {}
        self.transport = {"connected": False, "message": "Starting MAVLink observer"}
        self.gnss_transport = "Starting GNSS observer"
        self.timing = {}
        self.threads = [threading.Thread(target=fn, daemon=True) for fn in (self._mavlink, self._gnss, self._time)]
        for thread in self.threads:
            thread.start()

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=3)

    def snapshot(self):
        with self.lock:
            return copy.deepcopy((self.samples, self.transport, self.gnss_transport, self.timing))

    def _record(self, key, values, stamp=None):
        with self.lock:
            self.samples.setdefault(key, Sample()).observe(time.monotonic(), values, stamp)

    def _mavlink(self):
        from pymavlink.dialects.v20 import common as mavlink
        class Writer:
            def __init__(self, sock):
                self.sock = sock
            def write(self, data):
                self.sock.sendall(data)
        wanted = {"HEARTBEAT", "GPS_RAW_INT", "SCALED_PRESSURE"}   # H-Flow is observed on CAN; SCALED_PRESSURE = FC baro (bay air)
        while not self.stop.is_set():
            link = None
            try:
                _, host, port = self.params["mavlink_endpoint"].split(":")
                link = socket.create_connection((host, int(port)), timeout=2)
                link.settimeout(.2)
                protocol = mavlink.MAVLink(Writer(link), srcSystem=self.params["mavlink_source_system"], srcComponent=191)
                protocol.robust_parsing = True
                with self.lock:
                    self.transport = {"connected": True, "message": "Router TCP connected"}
                last_hb = -1.0
                while not self.stop.is_set():
                    try:
                        data = link.recv(65536)
                        if not data:
                            raise ConnectionError("Router TCP closed")
                        messages = protocol.parse_buffer(data) or []
                    except socket.timeout:
                        messages = []
                    now = time.monotonic()
                    for msg in messages:
                        if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                            continue
                        kind = msg.get_type()
                        if kind == "HEARTBEAT":
                            last_hb = now
                        if kind in wanted:
                            values = msg.to_dict()
                            values.pop("mavpackettype", None)
                            self._record(kind, values, values.get("time_usec", values.get("time_boot_ms")))
            except (OSError, ValueError) as exc:
                with self.lock:
                    self.transport = {"connected": False, "message": str(exc)}
            finally:
                if link is not None:
                    link.close()
            self.stop.wait(2)

    def _gnss(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection(("127.0.0.1", self.params["gnss_broker_port"]), timeout=2) as src:
                    src.settimeout(.5)
                    buf = b""
                    with self.lock:
                        self.gnss_transport = "Read-only GNSS broker connected"
                    while not self.stop.is_set():
                        try:
                            data = src.recv(65536)
                        except socket.timeout:
                            continue
                        if not data:
                            raise ConnectionError("GNSS broker closed")
                        blocks, buf = sbf_blocks(buf + data)
                        if blocks:
                            self._record("GNSS", {"source": "CRC-valid SBF via read-only broker"})
                        for bid, block in blocks:
                            if bid == 4007 and len(block) >= 80:
                                self._record("GNSS_PVT", *parse_pvt_geodetic(block))
                            elif bid == 5914 and len(block) >= 24:
                                self._record("GNSS_TIME", {"sync_level": block[21]}, struct.unpack_from("<IH", block, 8))
            except OSError as exc:
                with self.lock:
                    self.gnss_transport = str(exc)
            self.stop.wait(2)

    def _time(self):
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        while not self.stop.is_set():
            checks = {}
            try:
                outputs = [subprocess.run(["chronyc", "-n", arg], capture_output=True,
                            text=True, timeout=2, check=True, env={"PATH": "/usr/bin:/bin", "LC_ALL": "C"}).stdout
                           for arg in ("tracking", "sources")]
                checks["PPS"] = chrony_assessment(*outputs, time.time())
            except (OSError, subprocess.SubprocessError) as exc:
                checks["PPS"] = check(False, f"Chrony evidence unavailable: {exc}")
            try:
                data = json.loads(Path(self.params["ptp_status_path"]).read_text())
                checks["PTP"] = ptp_assessment(data, boot_id, time.monotonic_ns())
            except (OSError, ValueError, TypeError) as exc:
                checks["PTP"] = check(False, f"PTP evidence unavailable: {exc}")
            with self.lock:
                self.timing = {"received": time.monotonic(), "checks": checks}
            self.stop.wait(2)


def gnss_health(samples, timing, transport, now, require_rtk_fixed=True):
    """The position section is OK only at RTK fixed when require_rtk_fixed:
    this airframe navigates on RTK, so float, DGNSS or stand-alone is a
    warning even though the PVT is valid."""
    receiver = samples.get("GNSS", Sample())
    pvt = samples.get("GNSS_PVT", Sample())
    live = receiver.fresh(now, 3)
    connection = True if live else False if receiver.received >= 0 else None
    mode = pvt.values.get("mode", 0)
    mode_text = pvt.values.get("mode_text", PVT_MODES.get(mode, f"mode {mode}"))
    pvt_good = pvt.fresh(now, 3) and pvt.advancing(now, 3) and mode != 0 and pvt.values.get("error", 255) == 0
    rtk_fixed = mode in RTK_FIXED_MODES
    corr_age = pvt.values.get("mean_corr_age_sec")
    h_acc, v_acc = pvt.values.get("h_accuracy_m"), pvt.values.get("v_accuracy_m")
    if pvt_good:
        parts = [mode_text, f"{pvt.values.get('satellites', '?')} SVs",
                 "no corrections" if corr_age is None else f"corrections {corr_age:.1f} s old"]
        if h_acc is not None and v_acc is not None:
            parts.append(f"H {h_acc:.2f} m V {v_acc:.2f} m")
        position_text = "; ".join(parts)
    else:
        position_text = "Missing, stale or invalid PVT solution"
    position_ok = pvt_good and (rtk_fixed or not require_rtk_fixed)
    clock = timing.get("checks", {}).get("PPS", check(False, "No chrony evidence"))
    if now - timing.get("received", -100) > 6:
        clock = check(False, "Chrony observation stale")
    message = "Receiver data live" if live else "GNSS data connection lost" if connection is False else "Unknown; no receiver evidence"
    message += f"; {mode_text}" if pvt_good else "; position fix unavailable"
    if pvt_good and require_rtk_fixed and not rtk_fixed:
        message += " (RTK fixed required)"
    if clock.level == 0:
        message += "; PPS locked"
    metrics = {key: ("n/a" if value is None else value) for key, value in pvt.metrics(now).items()}
    return grouped(connection, message, {
        "Receiver": check(live, transport, **receiver.metrics(now)),
        "Position": check(position_ok, position_text, **metrics),
        "Timing": clock,
    })
