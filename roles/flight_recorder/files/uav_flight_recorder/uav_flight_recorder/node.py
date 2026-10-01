"""Flight-gated rosbag2 recorder.

Starts `ros2 bag record` when the vehicle arms (or on a manual request),
keeps it rolling through disarm, and stops once the vehicle is safed. The raw
D555 streams are excluded by regex: the camera unicasts a copy per subscriber
and a second copy would starve the encoder's link; the H.265 video topic and
the throttled depth are recorded instead.
"""
import json
import os
import shutil
import signal
import subprocess
import time
from datetime import datetime, timezone

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .policy import FlightPolicy, IDLE, RECORDING, POST_ROLL

DEFAULTS = {
    "bag_dir": "/data/flights",
    "armed_topic": "/px4/armed", "safety_topic": "/px4/safety_off", "manual_topic": "/flight_recorder/manual",
    "post_roll_sec": 30.0, "safed_grace_sec": 5.0, "min_free_gb": 50.0,
    "exclude_regex": "^/realsense/", "storage_preset": "zstd_fast", "max_cache_size": 268435456,
    "stop_timeout_sec": 45.0,
}


def dir_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


class FlightRecorder(Node):
    def __init__(self):
        super().__init__("uav_flight_recorder")
        self.declare_parameters("", list(DEFAULTS.items()))
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self.policy = FlightPolicy(self.p["post_roll_sec"], self.p["safed_grace_sec"])
        latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, self.p["armed_topic"], lambda m: self._event(armed=m.data), latched)
        self.create_subscription(Bool, self.p["safety_topic"], lambda m: self._event(safety_off=m.data), latched)
        self.create_subscription(Bool, self.p["manual_topic"], lambda m: self._event(manual=m.data), 10)
        self.diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.proc = None
        self.bag = None             # dict(path, started_at, armed_at, reason)
        self.flights = 0
        self.last_flight = None
        self.last_error = None
        self.last_size = 0
        self.size_checked = 0.0
        os.makedirs(self.p["bag_dir"], exist_ok=True)
        self.create_timer(1.0, self._tick)
        self.get_logger().info("flight recorder: %s, post-roll %.0f s, excluding %r" % (
            self.p["bag_dir"], self.p["post_roll_sec"], self.p["exclude_regex"]))

    # --- policy -------------------------------------------------------------
    def _event(self, **kw):
        self._act(self.policy.update(time.monotonic(), **kw))

    def _tick(self):
        now = time.monotonic()
        self._act(self.policy.tick(now))
        if self.proc is not None and self.proc.poll() is not None:
            self.last_error = "ros2 bag record exited with %s while %s" % (self.proc.returncode, self.policy.state)
            self.get_logger().error(self.last_error)
            self._finish("recorder exited")
            if self.policy.state in (RECORDING, POST_ROLL):
                self._start("restart after recorder exit")
        if self.proc is not None and now - self.size_checked > 5:
            self.size_checked = now
            self.last_size = dir_bytes(self.bag["path"])
            if self._free_gb() < float(self.p["min_free_gb"]):
                self.last_error = "stopped: free space below %.0f GB" % self.p["min_free_gb"]
                self.get_logger().error(self.last_error)
                self._finish("disk full guard")
                self.policy.state = IDLE
        self._publish_diagnostics()

    def _act(self, decision):
        if decision.action == "start" and self.proc is None:
            self._start(decision.reason)
        elif decision.action == "stop" and self.proc is not None:
            self._finish(decision.reason)
        elif decision.reason:
            self.get_logger().info(decision.reason)

    # --- recorder process ---------------------------------------------------
    def _start(self, reason):
        if self._free_gb() < float(self.p["min_free_gb"]):
            self.last_error = "not recording: free space below %.0f GB" % self.p["min_free_gb"]
            self.get_logger().error(self.last_error)
            return
        stamp = datetime.now(timezone.utc)
        path = os.path.join(self.p["bag_dir"], stamp.strftime("flight_%Y%m%d_%H%M%SZ"))
        cmd = ["ros2", "bag", "record", "-s", "mcap", "--storage-preset-profile", str(self.p["storage_preset"]),
               "-a", "--exclude-regex", str(self.p["exclude_regex"]), "--max-cache-size", str(int(self.p["max_cache_size"])),
               "-o", path]
        log = open(path + ".log", "ab")
        try:
            self.proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        except OSError as exc:
            self.last_error = "cannot start ros2 bag record: %s" % exc
            self.get_logger().error(self.last_error)
            log.close()
            return
        self.bag = {"path": path, "started_at": stamp.isoformat(), "started_mono": time.monotonic(), "reason": reason,
                    "armed_at": stamp.isoformat() if self.policy.armed else None, "log": log}
        self.last_size = 0
        self.get_logger().info("recording %s (%s)" % (path, reason))

    def _finish(self, reason):
        proc, bag = self.proc, self.bag
        self.proc = None
        if proc is None:
            return
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=float(self.p["stop_timeout_sec"]))
            except subprocess.TimeoutExpired:
                self.get_logger().error("recorder did not close within %.0f s; terminating" % self.p["stop_timeout_sec"])
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        bag["log"].close()
        size = dir_bytes(bag["path"])
        duration = round(time.monotonic() - bag["started_mono"], 1)
        meta = {"bag": bag["path"], "started_at": bag["started_at"], "armed_at": bag["armed_at"],
                "stopped_at": datetime.now(timezone.utc).isoformat(), "duration_sec": duration,
                "bytes": size, "start_reason": bag["reason"], "stop_reason": reason,
                "recorder_exit_code": proc.returncode}
        try:
            with open(os.path.join(bag["path"], "flight.json"), "w") as f:
                json.dump(meta, f, indent=2)
        except OSError as exc:
            self.get_logger().warning("flight.json not written: %s" % exc)
        self.flights += 1
        self.last_flight = meta
        self.bag = None
        self.get_logger().info("stopped %s after %.0f s, %.1f MB (%s)" % (bag["path"], duration, size / 1e6, reason))

    def _free_gb(self):
        try:
            return shutil.disk_usage(self.p["bag_dir"]).free / 1e9
        except OSError:
            return 0.0

    # --- diagnostics ----------------------------------------------------------
    def _publish_diagnostics(self):
        state = self.policy.state
        free = self._free_gb()
        values = {"state": state, "armed": self.policy.armed, "safety_off": self.policy.safety_off,
                  "manual": self.policy.manual, "bag_dir": self.p["bag_dir"], "free_gb": round(free, 1),
                  "flights_this_boot": self.flights, "last_error": self.last_error}
        if self.bag:
            values.update({"bag": self.bag["path"], "bag_mb": round(self.last_size / 1e6, 1),
                           "duration_s": round(time.monotonic() - self.bag["started_mono"]), "start_reason": self.bag["reason"]})
        if self.last_flight:
            values.update({"last_flight/" + k: v for k, v in self.last_flight.items() if k in ("bag", "duration_sec", "bytes", "stop_reason")})
        if self.last_error and (self.proc is None and state != IDLE):
            level, msg = DiagnosticStatus.ERROR, self.last_error
        elif state == RECORDING:
            level, msg = DiagnosticStatus.OK, "Recording %s (%.1f MB)" % (os.path.basename(self.bag["path"]), self.last_size / 1e6)
        elif state == POST_ROLL:
            level, msg = DiagnosticStatus.OK, "Disarmed; post-roll, bag still open"
        elif free < float(self.p["min_free_gb"]):
            level, msg = DiagnosticStatus.WARN, "Idle; %.0f GB free is below the %.0f GB guard, no recording possible" % (free, self.p["min_free_gb"])
        else:
            level, msg = DiagnosticStatus.OK, "Idle; armed-flight recording ready (%.0f GB free)" % free
        d = DiagnosticArray()
        d.header.stamp = self.get_clock().now().to_msg()
        d.status = [DiagnosticStatus(level=level, name="Flight Recorder", message=msg, hardware_id="rosbag2 mcap",
                                     values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])]
        self.diag.publish(d)

    def close(self):
        if self.proc is not None:
            self._finish("recorder shutting down")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlightRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
