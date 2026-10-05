#!/usr/bin/env python3
"""Configure and watch an ArduRemoteID broadcast module from the ground.

The module hangs off a PX4 TELEM port and only speaks MAVLink (no USB console,
no parameter editor in its web page). Everything here goes through the Jetson's
mavlink-router: PX4 forwards messages addressed to component 236 onto the
module's port and the module's replies back, because the TELEM2 and TELEM3
MAVLink instances both have forwarding on (MAV_x_FORWARD, MAV_HB_FORW_EN).

  watch        heartbeats / arm status from the module, PX4 Remote ID messages
  params       the module's numeric parameters (strings are not readable over MAVLink)
  set-param    PARAM_SET on the module, e.g. WEBSERVER_EN 0, BT4_POWER 12, BAUDRATE 115200
  set-id       send OPEN_DRONE_ID_BASIC_ID: a module whose UAS_ID/UAS_ID_TYPE/UAS_TYPE are
               still unset persists it into those parameters (the only way to set the
               string over MAVLink); a module that already has one broadcasts it as BasicID 2
  fc-params    show, and with --apply set, the PX4 side for the chosen TELEM port

--direct talks to the module alone on a serial adapter (3.3 V, module RX = ESP32 GPIO2,
module TX = GPIO3) and plays the autopilot's heartbeat so the module adopts system id 1.
"""
import argparse
import os
import struct
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

RID_COMP = 236           # MAV_COMP_ID_ODID_TXRX_1, what ArduRemoteID answers as
MAV_TYPE_ODID = mavutil.mavlink.MAV_TYPE_ODID   # 34
STATES = {0: "UNINIT (stock ArduRemoteID: PX4 calls this 'not ready')", 3: "STANDBY", 4: "ACTIVE (healthy for PX4)",
          5: "CRITICAL (module: data missing or invalid)"}
ARM = {0: "GOOD_TO_ARM", 1: "PRE_ARM_FAIL_GENERIC"}
PX4_INT_TYPES = (mavutil.mavlink.MAV_PARAM_TYPE_INT32, mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
                 mavutil.mavlink.MAV_PARAM_TYPE_INT8, mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
                 mavutil.mavlink.MAV_PARAM_TYPE_INT16, mavutil.mavlink.MAV_PARAM_TYPE_UINT16)


def px4_value(msg):
    """PX4 sends integer parameters as the integer's bit pattern in the float field."""
    if msg.param_type in PX4_INT_TYPES:
        return struct.unpack("<i", struct.pack("<f", msg.param_value))[0]
    return msg.param_value


def px4_int_as_float(v):
    return struct.unpack("<f", struct.pack("<i", int(v)))[0]


class Link:
    def __init__(self, args):
        self.args = args
        self.direct = args.direct
        if self.direct:
            self.m = mavutil.mavlink_connection(args.link, baud=args.baud, source_system=1, source_component=1)
        else:
            self.m = mavutil.mavlink_connection(args.link, source_system=args.source_system, source_component=191)
        self.sysid = 1
        self.last_hb = 0.0
        if not self.direct:
            self.wait_fc()

    def wait_fc(self, timeout=10.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if hb and hb.get_srcComponent() == 1 and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.sysid = hb.get_srcSystem()
                return
        sys.exit("no autopilot heartbeat on %s" % self.args.link)

    def heartbeat(self):
        """Keep the router (and, in --direct mode, the module) aware of us: once a second."""
        if time.time() - self.last_hb < 1.0:
            return
        self.last_hb = time.time()
        if self.direct:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_AUTOPILOT_PX4, 0, 0,
                                      mavutil.mavlink.MAV_STATE_STANDBY)
        else:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def recv(self, types, timeout=0.5):
        self.heartbeat()
        return self.m.recv_match(type=types, blocking=True, timeout=timeout)

    # --- module parameters (ArduRemoteID answers every numeric parameter as REAL32) ---
    def rid_param_read(self, name, timeout=2.0):
        for _ in range(3):
            self.m.mav.param_request_read_send(self.sysid, RID_COMP, name.encode(), -1)
            t_end = time.time() + timeout
            while time.time() < t_end:
                r = self.recv("PARAM_VALUE")
                if r and r.get_srcComponent() == RID_COMP and r.param_id.rstrip("\x00") == name:
                    return r.param_value
        return None

    def rid_param_set(self, name, value):
        for _ in range(3):
            self.m.mav.param_set_send(self.sysid, RID_COMP, name.encode(), float(value),
                                      mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t_end = time.time() + 2.0
            while time.time() < t_end:
                r = self.recv("PARAM_VALUE")
                if r and r.get_srcComponent() == RID_COMP and r.param_id.rstrip("\x00") == name:
                    return r.param_value
        return None

    def rid_param_list(self):
        self.m.mav.param_request_list_send(self.sysid, RID_COMP)
        seen, count, idle = {}, None, time.time()
        while time.time() - idle < 3.0 and (count is None or len(seen) < count):
            r = self.recv("PARAM_VALUE")
            if r and r.get_srcComponent() == RID_COMP:
                seen[r.param_id.rstrip("\x00")] = r.param_value
                count, idle = r.param_count, time.time()
        return seen, count

    # --- PX4 parameters ---
    def px4_param_read(self, name, timeout=2.0):
        for _ in range(3):
            self.m.mav.param_request_read_send(self.sysid, 1, name.encode(), -1)
            t_end = time.time() + timeout
            while time.time() < t_end:
                r = self.recv("PARAM_VALUE")
                if r and r.get_srcComponent() == 1 and r.param_id.rstrip("\x00") == name:
                    return px4_value(r), r.param_type
        return None, None

    def px4_param_set_int(self, name, value):
        for _ in range(3):
            self.m.mav.param_set_send(self.sysid, 1, name.encode(), px4_int_as_float(value),
                                      mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            t_end = time.time() + 2.0
            while time.time() < t_end:
                r = self.recv("PARAM_VALUE")
                if r and r.get_srcComponent() == 1 and r.param_id.rstrip("\x00") == name:
                    return px4_value(r)
        return None


def cmd_watch(link, args):
    t_end = time.time() + args.seconds
    print("watching for %.0f s (module = system %d component %d) ..." % (args.seconds, link.sysid, RID_COMP))
    while time.time() < t_end:
        msg = link.recv(["HEARTBEAT", "OPEN_DRONE_ID_ARM_STATUS", "STATUSTEXT"])
        if not msg:
            continue
        stamp = time.strftime("%H:%M:%S")
        if msg.get_type() == "HEARTBEAT" and msg.get_srcComponent() == RID_COMP:
            print("%s module heartbeat: type %d%s, system_status %d = %s" % (
                stamp, msg.type, " (ODID)" if msg.type == MAV_TYPE_ODID else "", msg.system_status,
                STATES.get(msg.system_status, "?")))
        elif msg.get_type() == "OPEN_DRONE_ID_ARM_STATUS":
            print("%s module arm status: %d = %s %s" % (stamp, msg.status, ARM.get(msg.status, "?"),
                                                        msg.error.rstrip("\x00")))
        elif msg.get_type() == "STATUSTEXT":
            text = msg.text.rstrip("\x00")
            if args.all or any(k in text for k in ("Remote ID", "Drone ID", "Preflight", "ODID")):
                print("%s PX4 (severity %d): %s" % (stamp, msg.severity, text))


def cmd_params(link, args):
    seen, count = link.rid_param_list()
    if not seen:
        sys.exit("no parameter replies from component %d (module not powered, not on a forwarded port, "
                 "or the baud rates differ)" % RID_COMP)
    print("%d of %s parameters:" % (len(seen), count))
    for name in sorted(seen):
        print("  %-16s %g" % (name, seen[name]))


def cmd_set_param(link, args):
    got = link.rid_param_set(args.name, args.value)
    if got is None:
        sys.exit("no reply from the module")
    print("%s = %g (module echo)" % (args.name, got))
    if args.name == "BAUDRATE":
        print("takes effect when the module next powers up (an FC reboot power-cycles the TELEM ports)")


def cmd_set_id(link, args):
    uas = args.uas_id.encode("ascii")
    if not 1 <= len(uas) <= 20:
        sys.exit("UAS id must be 1..20 ASCII characters")
    before = (link.rid_param_read("UAS_ID_TYPE"), link.rid_param_read("UAS_TYPE"))
    print("module before: UAS_ID_TYPE %s UAS_TYPE %s" % before)
    if before[0] and before[1] and not args.force:
        sys.exit("the module already has a Basic ID in its parameters; it would broadcast this one as "
                 "BasicID 2 instead of replacing it. Reset it first (set-param TO_DEFAULTS 1) or pass --force "
                 "to send anyway.")
    for _ in range(args.repeat):
        link.m.mav.open_drone_id_basic_id_send(link.sysid, RID_COMP, bytes(20), args.id_type, args.ua_type,
                                               uas.ljust(20, b"\0"))
        t_end = time.time() + 1.0
        while time.time() < t_end:
            link.recv("HEARTBEAT", timeout=0.2)
    after = (link.rid_param_read("UAS_ID_TYPE"), link.rid_param_read("UAS_TYPE"))
    print("module after:  UAS_ID_TYPE %s UAS_TYPE %s" % after)
    if after == (float(args.id_type), float(args.ua_type)):
        print("stored. The id string itself is not readable over MAVLink: confirm it on the module's status page "
              "or with a Remote ID phone app.")
    else:
        sys.exit("the module did not persist the Basic ID")


def cmd_fc_params(link, args):
    want = {"SER_TEL%d_BAUD" % args.telem: args.baud, "MAV_%d_RATE" % args.instance: 0,
            "MAV_%d_MODE" % args.instance: 0, "MAV_%d_FORWARD" % args.instance: 1,
            "MAV_%d_CONFIG" % args.instance: 100 + args.telem, "COM_ARM_ODID": args.com_arm_odid}
    changes = []
    for name, target in want.items():
        value, ptype = link.px4_param_read(name)
        mark = "" if value == target else "  -> %s" % target
        print("  %-14s %s%s" % (name, value, mark))
        if value != target:
            changes.append((name, target))
    if not changes:
        print("FC already matches."); return
    if not args.apply:
        print("dry run; pass --apply to set the marked parameters (SER_*_BAUD and MAV_*_CONFIG need a reboot)")
        return
    for name, target in changes:
        got = link.px4_param_set_int(name, target)
        print("  set %-14s -> %s" % (name, got))
        if got != target:
            sys.exit("PX4 did not accept %s" % name)
    if args.reboot:
        link.m.mav.command_long_send(link.sysid, 1, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                     1, 0, 0, 0, 0, 0, 0)
        ack = link.recv("COMMAND_ACK", timeout=3.0)
        print("reboot: %s" % ("accepted" if ack and ack.result == 0 else "not acknowledged (armed?)"))
    else:
        print("reboot the FC to apply the serial changes")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default="tcp:jethawk:5760", help="pymavlink connection string (router TCP, or a serial device with --direct)")
    ap.add_argument("--baud", type=int, default=57600, help="serial baud for --direct")
    ap.add_argument("--direct", action="store_true", help="module alone on a serial adapter; we play the autopilot")
    ap.add_argument("--source-system", type=int, default=246)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("watch"); p.add_argument("--seconds", type=float, default=20.0); p.add_argument("--all", action="store_true")
    sub.add_parser("params")
    p = sub.add_parser("set-param"); p.add_argument("name"); p.add_argument("value", type=float)
    p = sub.add_parser("set-id"); p.add_argument("--uas-id", required=True); p.add_argument("--ua-type", type=int, default=2, help="2 = helicopter or multirotor")
    p.add_argument("--id-type", type=int, default=1, help="1 = serial number (ANSI/CTA-2063-A), 2 = CAA registration id"); p.add_argument("--repeat", type=int, default=3); p.add_argument("--force", action="store_true")
    p = sub.add_parser("fc-params"); p.add_argument("--telem", type=int, default=3); p.add_argument("--instance", type=int, default=0)
    p.add_argument("--baud", dest="baud", type=int, default=57600); p.add_argument("--com-arm-odid", type=int, default=0, help="0 off, 1 warn, 2 deny arming without a healthy module")
    p.add_argument("--apply", action="store_true"); p.add_argument("--reboot", action="store_true")
    args = ap.parse_args()
    link = Link(args)
    {"watch": cmd_watch, "params": cmd_params, "set-param": cmd_set_param, "set-id": cmd_set_id, "fc-params": cmd_fc_params}[args.cmd](link, args)


if __name__ == "__main__":
    main()
