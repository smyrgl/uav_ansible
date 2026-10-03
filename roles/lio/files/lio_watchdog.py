#!/usr/bin/env python3
"""FAST-LIO's watchdog: the /lio/health verdict for the lio nodes, and the restart.

Follows FAST-LIO's log (the uav-lio unit's journal, hence the systemd-journal group) and
its odometry (/Odometry) through a DivergenceMonitor (lio_health), and publishes the
verdict on /lio/health (std_msgs/String, JSON, latched): state ok | degraded | diverged,
the epoch (FAST-LIO restarts within this watchdog's session), the onset and since on the
host's monotonic clock, the reason and the restart bookkeeping. The E1R registration
stops registering and the map freezes and rolls back on DIVERGED; both start over on a
new epoch. When FAST-LIO stays diverged the RestartPolicy restarts it with
`systemctl restart`, which a polkit rule allows this service's user for uav-lio.service
alone. The bridge's /lio/odometry (the EKF's input) is not touched.
Diagnostics: the "lio/watchdog" row.
"""
import argparse
import json
import os
import subprocess
import threading
import time
from datetime import datetime, timezone

import lio_health as health


def follow_journal(unit, on_line, on_error, stop):
    """Feed every new line of `unit`'s journal to on_line(monotonic, text) until stop is set,
    restarting journalctl if it ends; on_error(text) reports why it did."""
    while not stop.is_set():
        try:
            proc = subprocess.Popen(["journalctl", "--follow", "--lines=0", "--output=cat", "--unit", unit],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        except OSError as exc:
            on_error(f"journalctl: {exc}")
            stop.wait(10)
            continue
        for line in proc.stdout:
            on_line(time.monotonic(), line.rstrip("\n"))
            if stop.is_set():
                break
        proc.terminate()
        detail = (proc.stderr.read() or "").strip().splitlines()
        on_error(f"journalctl ended ({proc.wait()}): {detail[-1] if detail else 'no output'}")
        stop.wait(5)


def boot_id():
    try:
        with open("/proc/sys/kernel/random/boot_id") as f:
            return f.read().strip()
    except OSError:
        return ""


def load_restarts(path):
    """Restart times (monotonic s) saved by an earlier watchdog in this boot: the rate limit
    survives the watchdog's own restarts, and a reboot starts it afresh."""
    try:
        with open(path) as f:
            saved = json.load(f)
    except (OSError, ValueError):
        return []
    return [float(t) for t in saved.get("restarts_mono", [])] if saved.get("boot_id") == boot_id() else []


def save_restarts(path, times):
    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"boot_id": boot_id(), "restarts_mono": list(times)}, f)
        os.replace(tmp, path)
    except OSError:
        pass


def journal_readable(unit):
    """Whether this process can read `unit`'s journal (any line at all since boot)."""
    try:
        out = subprocess.run(["journalctl", "--boot", "--lines=1", "--output=cat", "--unit", unit, "--no-pager"],
                             capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return out.returncode == 0 and bool(out.stdout.strip())


def create_watchdog_node(options):
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class Watchdog(Node):
        def __init__(self):
            super().__init__("lio_watchdog")
            self.lock = threading.Lock()
            self.monitor = health.DivergenceMonitor(sustain_s=options.sustain, max_speed_mps=options.max_speed,
                                                    max_leap_m=options.max_leap)
            self.policy = health.RestartPolicy(options.restart, options.restart_after, options.restart_min_interval,
                                               options.restarts_per_hour)
            self.policy.history.extend(load_restarts(options.state_file))
            self.session = time.time_ns()            # a restarted watchdog's epoch 0 is not FAST-LIO restarting
            self.lines = 0
            self.journal_error = None if journal_readable(options.unit) else "cannot read the journal of " + options.unit
            self.restarts, self.restart_failures, self.last_restart_utc = 0, 0, None
            self.published, self.last_key, self.logged_state = 0.0, None, health.OK
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub = self.create_publisher(String, options.health_topic, latched)
            self._String = String
            self.create_subscription(Odometry, options.odometry_topic, self._odom, 50)
            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self._diag = (DiagnosticArray, DiagnosticStatus, KeyValue)
            self.create_timer(0.5, self._tick)
            self.create_timer(1.0, self._diagnose)
            self.stop = threading.Event()
            threading.Thread(target=follow_journal, args=(options.unit, self._line, self._journal_failed, self.stop),
                             daemon=True, name="lio-watchdog-journal").start()
            self.get_logger().info(f"watching {options.unit} (log) and {options.odometry_topic} -> {options.health_topic}; "
                                   f"restart {'after %.0f s of divergence' % options.restart_after if options.restart else 'off'}")

        def _line(self, now, text):
            with self.lock:
                self.lines += 1
                self.journal_error = None
                self.monitor.log_line(now, text)

        def _journal_failed(self, text):
            with self.lock:
                self.journal_error = text

        def _odom(self, message):
            p = message.pose.pose.position
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            with self.lock:
                self.monitor.pose(time.monotonic(), stamp, (p.x, p.y, p.z))

        def _tick(self):
            now = time.monotonic()
            with self.lock:
                self.monitor.update(now)
                verdict = self.monitor.verdict()
                restart = self.policy.due(now, verdict)
                if restart:
                    self.policy.record(now)
                    save_restarts(options.state_file, self.policy.history)
                exhausted, restart_in = self.policy.exhausted(now), self.policy.next_in(now, verdict)
            if verdict["state"] != self.logged_state:    # evidence moves the state between ticks too
                log = self.get_logger().error if verdict["state"] == health.DIVERGED else self.get_logger().info
                log(f"FAST-LIO {verdict['state']}: {verdict['reason'] or 'tracking'} (epoch {verdict['epoch']})")
                self.logged_state = verdict["state"]
            if restart:
                self._restart(verdict)
            key = (verdict["state"], verdict["epoch"], verdict["reason"], exhausted)
            if key != self.last_key or now - self.published >= 2.0:
                self.pub.publish(self._String(data=health.encode(
                    verdict, session=self.session, restarts=self.restarts, exhausted=exhausted,
                    restart_in_s=None if restart_in is None else round(restart_in, 1))))
                self.published, self.last_key = now, key

        def _restart(self, verdict):
            self.get_logger().warning(f"FAST-LIO diverged ({verdict['reason']}): restarting {options.restart_unit}")
            try:
                result = subprocess.run(["systemctl", "restart", "--no-block", options.restart_unit],
                                        capture_output=True, text=True, timeout=30)
                ok, detail = result.returncode == 0, result.stderr.strip()
            except (OSError, subprocess.TimeoutExpired) as exc:
                ok, detail = False, str(exc)
            if ok:
                self.restarts += 1
                self.last_restart_utc = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
            else:
                self.restart_failures += 1
                self.get_logger().error(f"restart of {options.restart_unit} refused: {detail}")

        def _diagnose(self):
            DiagnosticArray_, DiagnosticStatus_, KeyValue_ = self._diag
            now = time.monotonic()
            with self.lock:
                verdict, error = self.monitor.verdict(), self.journal_error
                exhausted, restart_in = self.policy.exhausted(now), self.policy.next_in(now, verdict)
                lines, evidence = self.lines, self.monitor.evidence_total
            if verdict["state"] == health.DIVERGED:
                level = DiagnosticStatus_.ERROR
                if exhausted:
                    text = (f"FAST-LIO diverged ({verdict['reason']}); auto-restart exhausted "
                            f"({options.restarts_per_hour} in an hour): reposition, then restart uav-lio")
                elif restart_in is not None:
                    text = f"FAST-LIO diverged ({verdict['reason']}); restarting it in {restart_in:.0f} s"
                else:
                    text = f"FAST-LIO diverged ({verdict['reason']}); auto-restart off"
            elif error:
                level, text = DiagnosticStatus_.WARN, f"Blind to FAST-LIO's log: {error}"
            elif verdict["state"] == health.DEGRADED:
                level, text = DiagnosticStatus_.WARN, f"FAST-LIO degraded: {verdict['reason']}"
            else:
                level, text = DiagnosticStatus_.OK, f"FAST-LIO tracking (epoch {verdict['epoch']})"
            values = {"state": verdict["state"], "epoch": verdict["epoch"], "reason": verdict["reason"],
                      "diverged_for_s": None if verdict["since_mono"] is None else round(now - verdict["since_mono"], 1),
                      "log_lines_read": lines, "evidence_total": evidence, "restarts": self.restarts,
                      "restart_failures": self.restart_failures, "last_restart_utc": self.last_restart_utc,
                      "restart_exhausted": exhausted, "auto_restart": options.restart}
            out = DiagnosticArray_()
            out.header.stamp = self.get_clock().now().to_msg()
            out.status = [DiagnosticStatus_(level=level, name="lio/watchdog", hardware_id="FAST-LIO divergence watchdog",
                                            message=text, values=[KeyValue_(key=k, value=str(v)) for k, v in values.items()])]
            self.diag_pub.publish(out)

    return Watchdog()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--unit", default="uav-lio.service", help="FAST-LIO's systemd unit (its log)")
    parser.add_argument("--restart-unit", default="uav-lio.service")
    parser.add_argument("--odometry-topic", default="/Odometry")
    parser.add_argument("--health-topic", default="/lio/health")
    parser.add_argument("--sustain", type=float, default=2.0, help="s of unbroken evidence that make a divergence")
    parser.add_argument("--max-speed", type=float, default=30.0, help="m/s no X950 reaches")
    parser.add_argument("--max-leap", type=float, default=5.0, help="m between consecutive poses")
    parser.add_argument("--no-restart", dest="restart", action="store_false", help="report only")
    parser.add_argument("--restart-after", type=float, default=10.0, help="s diverged before a restart")
    parser.add_argument("--restart-min-interval", type=float, default=120.0)
    parser.add_argument("--restarts-per-hour", type=int, default=5)
    parser.add_argument("--state-file", default="/var/lib/uav-ros/lio_watchdog.json",
                        help="restart times kept across the watchdog's own restarts (this boot)")
    options, ros_args = parser.parse_known_args()
    import rclpy
    node = None
    rclpy.init(args=ros_args)
    try:
        node = create_watchdog_node(options)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop.set()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
