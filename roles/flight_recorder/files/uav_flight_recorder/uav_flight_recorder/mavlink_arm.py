"""The autopilot's armed state and one transmitter switch, from mavlink-router's TCP server.

The recorder keys on the HEARTBEAT rather than on /px4/armed (XRCE-DDS over the
FC's Ethernet). The FMU module's whole Ethernet interface (RMII, plus the
management bus and power enable of the PHY on the baseboard) runs through
the second of its two board-to-board connectors (PAB X2); TELEM2, which feeds
mavlink-router, and the module's power run through the first (X1). With the
module marginally seated (2026-10-02) the Ethernet stayed down for whole
boots while TELEM2 kept working.

The switch comes from RC_CHANNELS, which PX4 forwards raw and without any
loss or SBUS-failsafe marker (failsafe positions pass straight through). It
counts only while SYS_STATUS lists the RC receiver as present, which PX4
clears whenever manual control is invalid.

Read-only: nothing is ever sent. Only system 1, component 1 counts, so the
camera component's own heartbeat (system 1, component 100) can never stand
in for the autopilot's.
"""
import math
import socket
import threading
import time

MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536


def armed_from_base_mode(base_mode):
    return bool(int(base_mode) & MAV_MODE_FLAG_SAFETY_ARMED)


class AutopilotParser:
    """Feed raw router bytes; returns what the autopilot said in them, in order:
    ("armed", bool) per HEARTBEAT, ("rc", raw microseconds of channels 1-18) per
    RC_CHANNELS and ("rc_present", bool) per SYS_STATUS."""

    def __init__(self, autopilot=(1, 1)):
        from pymavlink.dialects.v20 import common as mavlink
        self._mav = mavlink.MAVLink(None)   # parse only; never packs or sends
        self._mav.robust_parsing = True
        self.autopilot = tuple(autopilot)

    def feed(self, data):
        events = []
        for msg in self._mav.parse_buffer(data) or []:
            if (msg.get_srcSystem(), msg.get_srcComponent()) != self.autopilot:
                continue
            kind = msg.get_type()
            if kind == "HEARTBEAT":
                events.append(("armed", armed_from_base_mode(msg.base_mode)))
            elif kind == "RC_CHANNELS":
                events.append(("rc", tuple(getattr(msg, "chan%d_raw" % i) for i in range(1, 19))))
            elif kind == "SYS_STATUS":
                events.append(("rc_present", bool(msg.onboard_control_sensors_present & MAV_SYS_STATUS_SENSOR_RC_RECEIVER)))
        return events


class RcSwitch:
    """One transmitter channel as a debounced on/off level. ``level`` is None
    while the RC receiver is absent, after a gap of ``stale_s`` and until the
    next frame, which becomes the new baseline. ``presses`` counts debounced
    off-to-on changes after a baseline."""

    def __init__(self, channel, threshold_us=1500, debounce_frames=2, stale_s=1.0):
        if not 1 <= int(channel) <= 18:
            raise ValueError("rc channel must be 1-18")
        self.channel = int(channel)
        self.threshold = float(threshold_us)
        self.debounce = max(1, int(debounce_frames))
        self.stale_s = float(stale_s)
        self.present = False
        self.level = None
        self.presses = 0
        self._candidate = None
        self._last = -math.inf

    def set_present(self, present):
        self.present = bool(present)
        if not self.present:
            self.level, self._candidate = None, None

    def frame(self, values, now):
        if now - self._last > self.stale_s:
            self.level, self._candidate = None, None
        self._last = now
        raw = values[self.channel - 1] if self.channel <= len(values) else None
        if not self.present or raw is None or not 0 < raw < 65535:
            self.level, self._candidate = None, None
            return
        level = raw > self.threshold
        if self.level is None or level == self.level:
            self.level, self._candidate = level, None
            return
        seen, count = self._candidate or (level, 0)
        count = count + 1 if seen == level else 1
        if count < self.debounce:
            self._candidate = (level, count)
            return
        self.level, self._candidate = level, None
        if level:
            self.presses += 1


class AutopilotLink:
    """Background reader of the autopilot's armed flag and, if ``rc_channel``
    is set, that transmitter channel. Reconnects on its own."""

    def __init__(self, endpoint, rc_channel=0, autopilot=(1, 1)):
        scheme, host, port = endpoint.split(":")
        if scheme != "tcp":
            raise ValueError("mavlink_endpoint must be tcp:host:port")
        self.address = (host, int(port))
        self.autopilot = tuple(autopilot)
        self.rc = RcSwitch(rc_channel) if int(rc_channel) else None
        self._lock = threading.Lock()
        self._armed = None
        self._last_heartbeat = None
        self._heartbeats = 0
        self._connected = False
        self._error = ""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mavlink-autopilot", daemon=True)

    def start(self):
        self._thread.start()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=3)

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            age = None if self._last_heartbeat is None else now - self._last_heartbeat
            snap = {"armed": self._armed, "heartbeat_age_s": age, "heartbeats": self._heartbeats,
                    "connected": self._connected, "error": self._error}
            if self.rc:
                snap.update(rc_level=self.rc.level, rc_presses=self.rc.presses, rc_present=self.rc.present)
            return snap

    def apply(self, events, now):
        """Fold parser events into the state (the reader thread's job; public for tests)."""
        with self._lock:
            for kind, value in events:
                if kind == "armed":
                    self._armed, self._last_heartbeat = value, now
                    self._heartbeats += 1
                elif self.rc and kind == "rc_present":
                    self.rc.set_present(value)
                elif self.rc and kind == "rc":
                    self.rc.frame(value, now)

    def _run(self):
        while not self._stop.is_set():
            link = None
            try:
                parser = AutopilotParser(self.autopilot)
                link = socket.create_connection(self.address, timeout=2)
                link.settimeout(0.5)
                with self._lock:
                    self._connected, self._error = True, ""
                while not self._stop.is_set():
                    try:
                        data = link.recv(65536)
                    except socket.timeout:
                        continue
                    if not data:
                        raise ConnectionError("router closed the connection")
                    events = parser.feed(data)
                    if events:
                        self.apply(events, time.monotonic())
            except (OSError, ValueError, ImportError) as exc:
                with self._lock:
                    self._connected, self._error = False, str(exc)
            finally:
                if link is not None:
                    link.close()
            self._stop.wait(2)
