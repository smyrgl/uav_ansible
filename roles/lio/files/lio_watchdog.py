#!/usr/bin/env python3
"""FAST-LIO's watchdog: the /lio/health verdict for the lio nodes.

Follows FAST-LIO's log (the uav-lio unit's journal, hence the systemd-journal group) and
its odometry (/Odometry) through a DivergenceMonitor (lio_health), and publishes the
verdict on /lio/health (std_msgs/String, JSON, latched): state ok | degraded | diverged,
the epoch (FAST-LIO restarts within this watchdog's session), the onset and since on the
host's monotonic clock, and the reason. The E1R registration stops registering and the
map freezes and rolls back on DIVERGED; both start over on a new epoch. It restarts
nothing: FAST-LIO, as built here, exits when it is lost and systemd restarts it, as
often as it takes. The bridge's /lio/odometry is not touched.
Diagnostics: the "lio/watchdog" row.
"""
import argparse
import os
import subprocess
import threading
import time

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
            self.session = time.time_ns()            # a restarted watchdog's epoch 0 is not FAST-LIO restarting
            self.lines = 0
            self.journal_error = None if journal_readable(options.unit) else "cannot read the journal of " + options.unit
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
            self.get_logger().info(f"watching {options.unit} (log) and {options.odometry_topic} -> {options.health_topic}")

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
            if verdict["state"] != self.logged_state:    # evidence moves the state between ticks too
                text = f"FAST-LIO {verdict['state']}: {verdict['reason'] or 'tracking'} (epoch {verdict['epoch']})"
                # rclpy caches every logging call site with its severity and raises if the
                # severity changes there (2026-10-03: the process died on degraded -> diverged),
                # so ERROR and INFO get call sites of their own.
                if verdict["state"] == health.DIVERGED:
                    self.get_logger().error(text)
                else:
                    self.get_logger().info(text)
                self.logged_state = verdict["state"]
            key = (verdict["state"], verdict["epoch"], verdict["reason"])
            if key != self.last_key or now - self.published >= 2.0:
                self.pub.publish(self._String(data=health.encode(verdict, session=self.session)))
                self.published, self.last_key = now, key

        def _diagnose(self):
            DiagnosticArray_, DiagnosticStatus_, KeyValue_ = self._diag
            now = time.monotonic()
            with self.lock:
                verdict, error = self.monitor.verdict(), self.journal_error
                lines, evidence = self.lines, self.monitor.evidence_total
            if verdict["state"] == health.DIVERGED:
                level = DiagnosticStatus_.ERROR
                text = f"FAST-LIO diverged ({verdict['reason']}); systemd restarts it"
            elif error:
                level, text = DiagnosticStatus_.WARN, f"Blind to FAST-LIO's log: {error}"
            elif verdict["state"] == health.DEGRADED:
                level, text = DiagnosticStatus_.WARN, f"FAST-LIO degraded: {verdict['reason']}"
            else:
                level, text = DiagnosticStatus_.OK, f"FAST-LIO tracking (epoch {verdict['epoch']})"
            values = {"state": verdict["state"], "epoch": verdict["epoch"], "reason": verdict["reason"],
                      "diverged_for_s": None if verdict["since_mono"] is None else round(now - verdict["since_mono"], 1),
                      "log_lines_read": lines, "evidence_total": evidence,
                      "fastlio_restarts": verdict["epoch"]}
            out = DiagnosticArray_()
            out.header.stamp = self.get_clock().now().to_msg()
            out.status = [DiagnosticStatus_(level=level, name="lio/watchdog", hardware_id="FAST-LIO divergence watchdog",
                                            message=text, values=[KeyValue_(key=k, value=str(v)) for k, v in values.items()])]
            self.diag_pub.publish(out)

    return Watchdog()


def _spin(node):
    """rclpy's default executor, or the experimental EventsExecutor when
    UAV_EVENTS_EXECUTOR=1 (uav_ansible: ros_events_executor), the A/B of the
    autonomy roadmap's compute-recovery item."""
    import rclpy     # some nodes import it inside main()
    if os.environ.get("UAV_EVENTS_EXECUTOR", "0") == "1":
        try:
            from rclpy.experimental.events_executor import EventsExecutor
        except ImportError:
            EventsExecutor = None
        if EventsExecutor is not None:
            executor = EventsExecutor()
            executor.add_node(node)
            try:
                executor.spin()
            finally:
                executor.remove_node(node)
                executor.shutdown()
            return
    rclpy.spin(node)

def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--unit", default="uav-lio.service", help="FAST-LIO's systemd unit (its log)")
    parser.add_argument("--odometry-topic", default="/Odometry")
    parser.add_argument("--health-topic", default="/lio/health")
    parser.add_argument("--sustain", type=float, default=2.0, help="s of unbroken evidence that make a divergence")
    parser.add_argument("--max-speed", type=float, default=30.0, help="m/s no X950 reaches")
    parser.add_argument("--max-leap", type=float, default=5.0, help="m between consecutive poses")
    options, ros_args = parser.parse_known_args()
    import rclpy
    node = None
    rclpy.init(args=ros_args)
    try:
        node = create_watchdog_node(options)
        _spin(node)
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
