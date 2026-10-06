#!/usr/bin/env python3
"""Block the Wi-Fi radio while the autopilot is armed, keyed on its MAVLink HEARTBEAT.

The uplink dongle is a permanent fixture on the airframe, and the UniRC 7 Pro
link (RC and MAVLink) shares 2.4 and 5.8 GHz with it. So the radio goes dark
for the whole armed interval and comes back after landing: while armed the
Jetson's egress is the GCS tunnel (gcs_gateway), which the route metrics
already prefer once the wifi default route is gone.

Reads mavlink-router's TCP server, like the flight recorder does, and never
sends anything. No pymavlink: only HEARTBEAT (message 0) is decoded, with the
frame checksum verified, and only from the autopilot (system 1, component 1)
so no other component's heartbeat can stand in for it.

Policy
  armed HEARTBEAT              -> rfkill block <type> at once
  disarmed for unblock_delay_s -> rfkill unblock <type>
  no autopilot HEARTBEAT for stale_s (FC off, router down) -> unblock: nothing
                                  on the bench must be able to strand the box
  service stop                 -> the unit's ExecStopPost unblocks as well
The radio is left alone at start-up until the first HEARTBEAT decides, so a
restart in flight does not open the radio for a second.
"""
import argparse
import json
import socket
import subprocess
import sys
import time

MAV_MODE_FLAG_SAFETY_ARMED = 0x80
HEARTBEAT_ID = 0
HEARTBEAT_CRC_EXTRA = 50
HEARTBEAT_LEN = 9
MAVLINK1 = 0xFE
MAVLINK2 = 0xFD


def x25(data, crc=0xFFFF):
    """MAVLink's CRC-16/MCRF4XX, as used by both protocol versions."""
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class HeartbeatParser:
    """Feed raw bytes; get (system, component, armed) per valid HEARTBEAT.

    Other messages are stepped over by their declared length, accepted only
    when the byte after them is again a frame marker (their CRC_EXTRA is not
    known here); a candidate that fails that lookahead, or a HEARTBEAT whose
    checksum fails, costs one byte and the scan resumes."""

    def __init__(self):
        self.buf = bytearray()
        self.frames = 0
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while self.buf:
            i = 0
            while i < len(self.buf) and self.buf[i] not in (MAVLINK1, MAVLINK2):
                i += 1
            if i:
                del self.buf[:i]
                if not self.buf:
                    break
            magic = self.buf[0]
            if len(self.buf) < 2:
                break
            plen = self.buf[1]
            if magic == MAVLINK2:
                if len(self.buf) < 10:
                    break
                incompat = self.buf[2]
                total = 10 + plen + 2 + (13 if incompat & 1 else 0)
                if len(self.buf) < total:
                    break
                sysid, compid = self.buf[5], self.buf[6]
                msgid = self.buf[7] | (self.buf[8] << 8) | (self.buf[9] << 16)
                payload = bytes(self.buf[10:10 + plen])
                crc_rx = self.buf[10 + plen] | (self.buf[11 + plen] << 8)
                body = bytes(self.buf[1:10 + plen])
            else:
                if len(self.buf) < 6:
                    break
                total = 6 + plen + 2
                if len(self.buf) < total:
                    break
                sysid, compid, msgid = self.buf[3], self.buf[4], self.buf[5]
                payload = bytes(self.buf[6:6 + plen])
                crc_rx = self.buf[6 + plen] | (self.buf[7 + plen] << 8)
                body = bytes(self.buf[1:6 + plen])
            if msgid == HEARTBEAT_ID:
                if x25(bytes([HEARTBEAT_CRC_EXTRA]), x25(body)) != crc_rx:
                    self.bad += 1
                    del self.buf[0]
                    continue
                self.frames += 1
                full = payload + bytes(HEARTBEAT_LEN - len(payload)) if len(payload) < HEARTBEAT_LEN else payload
                out.append((sysid, compid, bool(full[6] & MAV_MODE_FLAG_SAFETY_ARMED)))
                del self.buf[:total]
                continue
            if len(self.buf) == total:
                break                       # need the lookahead byte before trusting the length
            if self.buf[total] not in (MAVLINK1, MAVLINK2):
                self.bad += 1
                del self.buf[0]
                continue
            del self.buf[:total]
        return out


class Policy:
    """Decides block/unblock from (armed, now); pure, so it is testable."""

    def __init__(self, unblock_delay_s, stale_s):
        self.unblock_delay = float(unblock_delay_s)
        self.stale = float(stale_s)
        self.blocked = None          # None: untouched since start
        self.armed = None
        self.last_heartbeat = None
        self.disarmed_since = None

    def heartbeat(self, armed, now):
        self.last_heartbeat = now
        if armed != self.armed:
            self.armed = armed
            self.disarmed_since = None if armed else now

    def wanted(self, now):
        """Return True to block, False to unblock, None to leave the radio alone."""
        if self.last_heartbeat is None:
            return None
        if now - self.last_heartbeat > self.stale:
            return False
        if self.armed:
            return True
        if self.blocked is None:
            return False                # first word from the autopilot is "disarmed": open
        if self.disarmed_since is not None and now - self.disarmed_since >= self.unblock_delay:
            return False
        return self.blocked

    def step(self, now):
        """Return 'block' / 'unblock' when the radio state must change, else None."""
        want = self.wanted(now)
        if want is None or want == self.blocked:
            return None
        self.blocked = want
        return "block" if want else "unblock"


def rfkill(action, rtype):
    subprocess.run(["rfkill", action, rtype], check=False)


def log(msg):
    print(time.strftime("%H:%M:%S"), msg, flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--endpoint", default="tcp:127.0.0.1:5760", help="mavlink-router TCP server")
    ap.add_argument("--type", default="wlan", help="rfkill device type to block")
    ap.add_argument("--unblock-delay", type=float, default=5.0, help="seconds disarmed before unblocking")
    ap.add_argument("--stale", type=float, default=10.0, help="seconds without a HEARTBEAT before unblocking")
    ap.add_argument("--autopilot", default="1:1", help="system:component whose HEARTBEAT counts")
    ap.add_argument("--state-file", default="/run/uav/wifi-rfkill.json", help="status for other tools ('' to disable)")
    args = ap.parse_args()
    scheme, host, port = args.endpoint.split(":")
    if scheme != "tcp":
        sys.exit("endpoint must be tcp:host:port")
    autopilot = tuple(int(v) for v in args.autopilot.split(":"))
    policy = Policy(args.unblock_delay, args.stale)
    parser = HeartbeatParser()
    log("wifi rfkill: autopilot %d:%d via %s, type %s, unblock after %.0f s disarmed or %.0f s silence"
        % (*autopilot, args.endpoint, args.type, args.unblock_delay, args.stale))
    sock = None
    last_status = 0.0
    while True:
        now = time.monotonic()
        if sock is None:
            try:
                sock = socket.create_connection((host, int(port)), timeout=3)
                sock.settimeout(1.0)
                parser.buf.clear()
                log("connected to %s" % args.endpoint)
            except OSError as e:
                sock = None
                if now - last_status > 30:
                    log("router unreachable (%s); retrying" % e)
                    last_status = now
                time.sleep(2)
        else:
            try:
                data = sock.recv(4096)
                if not data:
                    raise OSError("router closed the connection")
                for sysid, compid, armed in parser.feed(data):
                    if (sysid, compid) == autopilot:
                        policy.heartbeat(armed, time.monotonic())
            except socket.timeout:
                pass
            except OSError as e:
                log("link lost: %s" % e)
                sock.close()
                sock = None
        now = time.monotonic()
        action = policy.step(now)
        if action:
            rfkill(action, args.type)
            log("%s %s (autopilot %s, heartbeat %s)" % (
                action, args.type, "armed" if policy.armed else "disarmed",
                "fresh" if policy.last_heartbeat and now - policy.last_heartbeat <= args.stale else "stale"))
        if args.state_file and now - last_status > 5:
            last_status = now
            try:
                with open(args.state_file, "w") as f:
                    json.dump({"armed": policy.armed, "blocked": policy.blocked,
                               "heartbeat_age_s": None if policy.last_heartbeat is None else round(now - policy.last_heartbeat, 1),
                               "frames": parser.frames, "bad_frames": parser.bad, "connected": sock is not None}, f)
            except OSError:
                pass


if __name__ == "__main__":
    main()
