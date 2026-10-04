"""Flight-gated rosbag2 recorder.

Starts `ros2 bag record` when the vehicle arms (or on a manual request: the
manual topic or a transmitter switch),
keeps it rolling through disarm, and stops after a post-roll. The armed state
comes from the autopilot's MAVLink HEARTBEAT through mavlink-router (default),
or from the PX4 bridge's /px4/armed and /px4/safety_off, which can also stop
the bag early once the vehicle is safed. The raw
D555 streams are excluded by regex: the camera unicasts a copy per subscriber
and a second copy would starve the encoder's link; the H.265 video topic and
the throttled depth are recorded instead. A bag that stops growing while the
recorder process lives (rosbag2 Jazzy issue #2463) is caught after `stall_sec`:
the recorder is stopped and restarted into a new bag, and the stop reason says so.
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .mavlink_arm import AutopilotLink
from .policy import FlightPolicy, GrowthWatch, IDLE, RECORDING, POST_ROLL

DEFAULTS = {
    "bag_dir": "/data/flights",
    "armed_topic": "/px4/armed", "safety_topic": "/px4/safety_off", "manual_topic": "/flight_recorder/manual",
    "post_roll_sec": 30.0, "safed_grace_sec": 5.0, "min_free_gb": 50.0,
    "exclude_regex": "^/realsense/", "storage_preset": "zstd_fast", "max_cache_size": 268435456,
    "stop_timeout_sec": 45.0,
    # A bag that grows nothing for this long while the recorder lives is a
    # stalled recorder: it is restarted into a new bag. rosbag2 writes in
    # max_cache_size bursts, so this must exceed cache size / data rate.
    "stall_sec": 120.0,
    # Post-flight checks (postflight.py) in a separate low-priority process.
    "postflight_enabled": True,
    "postflight_required_topics": ["/fmu/out/vehicle_odometry", "/avia/points", "/e1r/points", "/gnss/navsatfix"],
    "postflight_gap_topics": ["/fmu/out/vehicle_odometry", "/avia/points", "/e1r/points", "/gnss/navsatfix",
                              "/fmu/out/trajectory_setpoint"],
    "postflight_max_gap_ms": 100.0, "postflight_map_service": "/lio/map/save", "postflight_timeout_sec": 1800.0,
    # Reset the live voxel map when a bag starts, so the map postflight saves
    # covers exactly the bag (what a replay of the bag can reproduce).
    "reset_map_on_start": True, "map_reset_service": "/lio/map/reset",
    # Where the armed state comes from: "mavlink" (the autopilot's HEARTBEAT
    # through mavlink-router, i.e. over TELEM2) or "ros" (/px4/armed and
    # /px4/safety_off from the PX4 bridge, i.e. over the FC's Ethernet).
    "arm_source": "mavlink", "mavlink_endpoint": "tcp:127.0.0.1:5760", "mavlink_stale_sec": 3.0,
    # A transmitter channel (RC_CHANNELS via the same router link, 0 = off) that
    # requests a bag like the manual topic: "switch" records while it is on,
    # "button" starts or stops on each press.
    "rc_channel": 0, "rc_mode": "switch",
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
        self.mav = None
        self._mav_armed = None
        self.rc_channel = int(self.p["rc_channel"])
        if self.rc_channel and self.p["rc_mode"] not in ("switch", "button"):
            raise ValueError("rc_mode must be 'switch' or 'button'")
        self._manual_topic = self._manual_rc = False
        self._rc_presses_seen = 0
        if self.p["arm_source"] == "ros":
            self.create_subscription(Bool, self.p["armed_topic"], lambda m: self._event(armed=m.data), latched)
            self.create_subscription(Bool, self.p["safety_topic"], lambda m: self._event(safety_off=m.data), latched)
        elif self.p["arm_source"] != "mavlink":
            raise ValueError("arm_source must be 'mavlink' or 'ros'")
        if self.p["arm_source"] == "mavlink" or self.rc_channel:
            # HEARTBEAT carries no safety-switch state; the switch is bypassed on
            # this FC anyway (CBRK_IO_SAFETY 22027). The bag closes on the post-roll.
            self.mav = AutopilotLink(self.p["mavlink_endpoint"], self.rc_channel)
            self.mav.start()
            self.create_timer(0.2, self._mavlink_poll)
        self.create_subscription(Bool, self.p["manual_topic"], self._on_manual_topic, 10)
        self.diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.proc = None
        self.bag = None             # dict(path, started_at, armed_at, reason)
        self.flights = 0
        self.last_flight = None
        self.last_error = None
        self.last_size = 0
        self.size_checked = 0.0
        self.growth = GrowthWatch(self.p["stall_sec"])
        self.posts = []             # running post-flight checks: dict(proc, bag, started, log)
        self.map_reset = self.create_client(Trigger, str(self.p["map_reset_service"])) if self.p["reset_map_on_start"] else None
        self.last_postflight = None
        os.makedirs(self.p["bag_dir"], exist_ok=True)
        self.create_timer(1.0, self._tick)
        self.get_logger().info("flight recorder: %s, armed state from %s, post-roll %.0f s, excluding %r%s" % (
            self.p["bag_dir"], "the autopilot HEARTBEAT (%s)" % self.p["mavlink_endpoint"] if self.p["arm_source"] == "mavlink"
            else self.p["armed_topic"], self.p["post_roll_sec"], self.p["exclude_regex"],
            ", bag %s on RC channel %d" % (self.p["rc_mode"], self.rc_channel) if self.rc_channel else ""))

    # --- policy -------------------------------------------------------------
    def _event(self, **kw):
        self._act(self.policy.update(time.monotonic(), **kw))

    def _on_manual_topic(self, msg):
        self._manual_topic = bool(msg.data)
        self._event(manual=self._manual_topic or self._manual_rc)

    def _mavlink_poll(self):
        """Hand changes to the policy, on the executor thread. A stale or lost
        link keeps the last known state: a dropout must never end a flight's
        bag early, and an absent transmitter holds its last request."""
        snap = self.mav.snapshot()
        armed = snap["armed"]
        if self.p["arm_source"] == "mavlink" and armed is not None and armed != self._mav_armed:
            self._mav_armed = armed
            self.get_logger().info("autopilot %s (HEARTBEAT)" % ("armed" if armed else "disarmed"))
            self._event(armed=armed)
        if not self.rc_channel:
            return
        request = self._manual_rc
        if self.p["rc_mode"] == "switch":
            if snap["rc_level"] is not None:
                request = snap["rc_level"]
        elif (snap["rc_presses"] - self._rc_presses_seen) % 2:
            request = not self._manual_rc
        self._rc_presses_seen = snap["rc_presses"]
        if request != self._manual_rc:
            self._manual_rc = request
            self.get_logger().info("bag %s on RC channel %d" % ("requested" if request else "released", self.rc_channel))
            self._event(manual=self._manual_topic or self._manual_rc)

    def _tick(self):
        now = time.monotonic()
        self._act(self.policy.tick(now))
        self._poll_postflight(now)
        if self.proc is not None and self.proc.poll() is not None:
            self.last_error = "ros2 bag record exited with %s while %s" % (self.proc.returncode, self.policy.state)
            self.get_logger().error(self.last_error)
            self._finish("recorder exited")
            if self.policy.state in (RECORDING, POST_ROLL):
                self._start("restart after recorder exit")
        if self.proc is not None and now - self.size_checked > 5:
            self.size_checked = now
            self.last_size = dir_bytes(self.bag["path"])
            self.growth.sample(now, self.last_size)
            if self._free_gb() < float(self.p["min_free_gb"]):
                self.last_error = "stopped: free space below %.0f GB" % self.p["min_free_gb"]
                self.get_logger().error(self.last_error)
                self._finish("disk full guard")
                self.policy.state = IDLE
            elif self.growth.stalled(now):
                self.last_error = "stalled: the bag grew nothing for %.0f s; restarting the recorder" % self.growth.stall_sec
                self.get_logger().error(self.last_error)
                self._finish("stalled bag")
                if self.policy.state in (RECORDING, POST_ROLL):
                    self._start("restart after a stalled bag")
        self._publish_diagnostics()

    def _act(self, decision):
        if decision.action == "start" and self.proc is None:
            self._start(decision.reason)
        elif decision.action == "stop" and self.proc is not None:
            self._finish(decision.reason)
        elif decision.reason:
            self.get_logger().info(decision.reason)
        if self.bag is not None and self.policy.state == POST_ROLL and not self.bag.get("disarmed_at"):
            self.bag["disarmed_at"] = datetime.now(timezone.utc).isoformat()
        elif self.bag is not None and self.policy.state == RECORDING and self.policy.armed and not self.bag.get("armed_at"):
            self.bag["armed_at"] = datetime.now(timezone.utc).isoformat()

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
        self.growth.reset(self.bag["started_mono"])
        self.get_logger().info("recording %s (%s)" % (path, reason))
        if self.map_reset is not None:
            if self.map_reset.service_is_ready():
                self.map_reset.call_async(Trigger.Request())      # fire and forget: the map node logs the reset
                self.get_logger().info("live map reset requested (%s)" % self.p["map_reset_service"])
            else:
                self.get_logger().warning("live map not reset: %s unavailable" % self.p["map_reset_service"])

    def _finish(self, reason):
        proc, bag = self.proc, self.bag
        self.proc = None
        if proc is None:
            return
        if proc.poll() is None:
            try:
                # A stopped recorder (SIGSTOP, a debugger) cannot act on SIGINT:
                # resume it first so it closes the bag instead of being killed
                # 55 s later with the index unwritten.
                os.killpg(os.getpgid(proc.pid), signal.SIGCONT)
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
                "disarmed_at": bag.get("disarmed_at"),
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
        self._postflight(bag["path"])

    # --- post-flight checks -------------------------------------------------
    def _postflight(self, path):
        if not self.p["postflight_enabled"]:
            return
        cmd = [sys.executable, "-m", "uav_flight_recorder.postflight", path,
               "--max-gap-ms", str(float(self.p["postflight_max_gap_ms"])), "--map-timeout", "30"]
        if self.p["postflight_map_service"]:
            cmd += ["--map-service", str(self.p["postflight_map_service"])]
        cmd += ["--required"] + [str(t) for t in self.p["postflight_required_topics"]]
        cmd += ["--gap-topics"] + [str(t) for t in self.p["postflight_gap_topics"]]
        try:
            log = open(path + ".postflight.log", "ab")
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        except OSError as exc:
            self.get_logger().error("post-flight checks not started for %s: %s" % (path, exc))
            return
        self.posts.append({"proc": proc, "bag": path, "started": time.monotonic(), "log": log})
        self.get_logger().info("post-flight checks started for %s" % path)

    def _poll_postflight(self, now):
        for post in list(self.posts):
            code = post["proc"].poll()
            if code is None:
                if now - post["started"] > float(self.p["postflight_timeout_sec"]):
                    os.killpg(os.getpgid(post["proc"].pid), signal.SIGKILL)
                    self.get_logger().error("post-flight checks for %s killed after %.0f s" % (post["bag"], now - post["started"]))
                continue
            post["log"].close()
            self.posts.remove(post)
            result = {"bag": post["bag"], "exit_code": code, "seconds": round(now - post["started"])}
            try:
                with open(os.path.join(post["bag"], "flight.json")) as f:
                    meta = json.load(f)
                verdict = meta.get("signoff", {})
                result["signoff"] = "PASS" if verdict.get("pass") else "FAIL: " + "; ".join(verdict.get("reasons", ["no verdict"]))
                result["map"] = (meta.get("postflight", {}).get("map") or {}).get("message", "not saved")
                if self.last_flight and self.last_flight.get("bag") == post["bag"]:
                    self.last_flight.update({"signoff": verdict, "postflight": meta.get("postflight")})
            except (OSError, ValueError) as exc:
                result["signoff"] = "unknown: %s" % exc
            self.last_postflight = result
            (self.get_logger().info if code in (0, 3) else self.get_logger().error)(
                "post-flight checks for %s: %s (exit %s, %s s)" % (post["bag"], result["signoff"], code, result["seconds"]))

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
                           "duration_s": round(time.monotonic() - self.bag["started_mono"]), "start_reason": self.bag["reason"],
                           "bag_growth_age_s": round(time.monotonic() - self.growth.last_growth)})
        if self.last_flight:
            values.update({"last_flight/" + k: v for k, v in self.last_flight.items() if k in ("bag", "duration_sec", "bytes", "stop_reason")})
        values["postflight/running"] = len(self.posts)
        if self.last_postflight:
            values.update({"postflight/" + k: v for k, v in self.last_postflight.items() if k in ("bag", "signoff", "map", "seconds", "exit_code")})
        values["arm_source"] = self.p["arm_source"]
        link_problem = None
        if self.rc_channel:
            snap = self.mav.snapshot()
            values.update({"rc/channel": self.rc_channel, "rc/mode": self.p["rc_mode"], "rc/present": snap["rc_present"],
                           "rc/level": snap["rc_level"], "manual/rc": self._manual_rc, "manual/topic": self._manual_topic})
        if self.mav and self.p["arm_source"] == "mavlink":
            mav = self.mav.snapshot()
            age = mav["heartbeat_age_s"]
            values.update({"mavlink/connected": mav["connected"], "mavlink/heartbeats": mav["heartbeats"],
                           "mavlink/heartbeat_age_s": None if age is None else round(age, 1),
                           "mavlink/error": mav["error"]})
            if age is None or age > float(self.p["mavlink_stale_sec"]):
                link_problem = "no autopilot HEARTBEAT" + (" (%s)" % mav["error"] if mav["error"] else "")
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
        if link_problem and level == DiagnosticStatus.OK:
            level = DiagnosticStatus.WARN
            msg = (msg + "; " if state != IDLE else "Idle; arming would not be detected: ") + link_problem
        d = DiagnosticArray()
        d.header.stamp = self.get_clock().now().to_msg()
        d.status = [DiagnosticStatus(level=level, name="Flight Recorder", message=msg, hardware_id="rosbag2 mcap",
                                     values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])]
        self.diag.publish(d)

    def close(self):
        if self.mav is not None:
            self.mav.close()
        if self.proc is not None:
            self._finish("recorder shutting down")


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

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlightRecorder()
        _spin(node)
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
