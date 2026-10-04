"""uav-fetch-ulogs: MAVLink FTP from the FC through the router, disarmed only.

    python -m uav_flight_recorder.fetch_ulogs --endpoint tcp:127.0.0.1:5760
        --flights /data/flights --days 3 [--max-age-days N]

Lists /fs/microsd/log/<day> for the recent days, downloads new .ulg files into
<flights>/ulogs/<day>_<name>, records them in <flights>/ulogs/fetched.json and
hard-links each into its bag as fc.ulg (bag matched by armed time). Refuses to
run while the FC is armed: the TELEM2 link is the pilot's telemetry.
"""
import argparse
import fcntl
import os
import socket
import sys
import time

from .ulogs import bag_for, bags_armed_at, load_state, log_start, recent_days, save_state, wanted


def armed(master, timeout=5.0):
    """True/False from the FC's next HEARTBEAT, None without one."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is not None and (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1):
            return bool(msg.base_mode & 128)
    return None


def fresh_ftp(master, pause=3.0):
    """A new MAVFTP object per transfer. Reusing one across downloads leaves the
    FC answering "no sessions available" to the next open (its session was
    released, the library's state was not); a new object after a short pause
    works every time (measured 2026-10-04 against the FC: 3 of 3 files)."""
    from pymavlink import mavftp
    time.sleep(pause)
    return mavftp.MAVFTP(master, target_system=1, target_component=1)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--endpoint", default="tcp:127.0.0.1:5760")
    ap.add_argument("--flights", default="/data/flights")
    ap.add_argument("--days", type=int, default=3)
    ap.add_argument("--tolerance-s", type=float, default=300.0)
    ap.add_argument("--source-system", type=int, default=253)
    args = ap.parse_args(argv)
    ulog_dir = os.path.join(args.flights, "ulogs")
    os.makedirs(ulog_dir, exist_ok=True)
    lock = open(os.path.join(ulog_dir, ".lock"), "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("another fetch is running"); return 0
    from pymavlink import mavutil
    try:
        master = mavutil.mavlink_connection(args.endpoint, source_system=args.source_system, source_component=191)
    except (OSError, socket.error) as exc:
        print("no router: %s" % exc); return 1
    state_armed = armed(master)
    if state_armed is None:
        print("no FC heartbeat; not fetching"); return 1
    if state_armed:
        print("FC armed; not fetching"); return 0
    state_path = os.path.join(ulog_dir, "fetched.json")
    state = load_state(state_path)
    bags = bags_armed_at(args.flights)
    fetched = failed = 0
    for day in recent_days(args.days):
        ftp = fresh_ftp(master, pause=1.0)
        ftp.cmd_list(["/fs/microsd/log/" + day])
        ftp.process_ftp_reply("ListDirectory", timeout=20)
        entries = [(e.name, e.size_b) for e in (ftp.list_result or []) if not e.is_dir]
        for name, size in wanted(entries, day, state):
            remote = "/fs/microsd/log/%s/%s" % (day, name)
            local = os.path.join(ulog_dir, "%s_%s" % (day, name))
            started = time.monotonic()
            print("fetching %s (%.1f MB)" % (remote, size / 1e6), flush=True)
            ftp = fresh_ftp(master)
            try:
                ftp.cmd_get([remote, local])
                ftp.process_ftp_reply("OpenFileRO", timeout=max(600.0, size / 8000.0))   # ~8 kB/s worst case
            except Exception as exc:
                print("  failed: %s" % exc); failed += 1; continue
            if not os.path.isfile(local) or os.path.getsize(local) != size:
                print("  incomplete (%s of %d bytes)" % (os.path.getsize(local) if os.path.isfile(local) else "none", size))
                failed += 1; continue
            seconds = time.monotonic() - started
            start = log_start(day, name)
            bag = bag_for(start, bags, args.tolerance_s) if start else None
            if bag:
                target = os.path.join(bag, "fc.ulg")
                if not os.path.exists(target):
                    os.link(local, target)
            state[day + "/" + name] = {"size": size, "fetched": True, "seconds": round(seconds), "bag": bag}
            save_state(state_path, state)
            fetched += 1
            print("  done in %.0f s (%.0f kB/s)%s" % (seconds, size / 1024 / max(seconds, 1), ", bag " + os.path.basename(bag) if bag else ", no bag within tolerance"))
    print("ulogs: %d fetched, %d failed" % (fetched, failed))
    return 0 if not failed else 3


if __name__ == "__main__":
    sys.exit(main())
