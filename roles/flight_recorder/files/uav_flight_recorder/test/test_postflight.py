import json
import os
import tempfile
import unittest

from uav_flight_recorder.postflight import counts_from_metadata, flight_card, gap_stats, iso_to_ns, run, signoff

METADATA = """rosbag2_bagfile_information:
  version: 9
  storage_identifier: mcap
  duration:
    nanoseconds: 18317584836
  starting_time:
    nanoseconds_since_epoch: 1791136442157368411
  message_count: 44836
  topics_with_message_count:
    - topic_metadata:
        name: /fmu/out/vehicle_odometry
        type: px4_msgs/msg/VehicleOdometry
        serialization_format: cdr
      message_count: 1718
    - topic_metadata:
        name: /fmu/in/trajectory_setpoint
        type: px4_msgs/msg/TrajectorySetpoint
        serialization_format: cdr
      message_count: 3
    - topic_metadata:
        name: /avia/points
        type: sensor_msgs/msg/PointCloud2
        serialization_format: cdr
      message_count: 179
"""


class PostflightTests(unittest.TestCase):
    def test_metadata_counts(self):
        counts, duration, start = counts_from_metadata(METADATA)
        self.assertEqual(counts["/fmu/out/vehicle_odometry"], 1718)
        self.assertEqual(duration, 18317584836)
        self.assertEqual(start, 1791136442157368411)

    def test_gap_stats_window_and_limit(self):
        t = [0, 10, 20, 150, 160, 400]              # ms: a 10 ms stream with a 130 ms hole and a 240 ms hole
        g = gap_stats([x * 1_000_000 for x in t], None, 100.0)
        self.assertEqual((g["count"], g["nominal_ms"], g["limit_ms"], g["gaps_over"], g["max_gap_ms"]), (6, 10.0, 110.0, 2, 240.0))
        g = gap_stats([x * 1_000_000 for x in t], (5_000_000, 165_000_000), 100.0)
        self.assertEqual((g["count"], g["gaps_over"], g["max_gap_ms"]), (4, 1, 130.0))
        lidar = [0, 100, 205, 300, 395, 500, 735, 800]    # 10 Hz with jitter and one missed frame
        g = gap_stats([x * 1_000_000 for x in lidar], None, 100.0)
        self.assertEqual((g["nominal_ms"], g["limit_ms"], g["gaps_over"], g["max_gap_ms"]), (100.0, 200.0, 1, 235.0))
        self.assertEqual(gap_stats([], None)["count"], 0)
        self.assertEqual(gap_stats([5], None)["count"], 1)
        self.assertAlmostEqual(gap_stats([0, 10_000_000, 20_000_000], None)["rate_hz"], 100.0)

    def test_signoff_rules(self):
        counts = {"/fmu/out/vehicle_odometry": 100, "/fmu/in/trajectory_setpoint": 3}
        gaps = {"/fmu/out/vehicle_odometry": {"gaps_over": 2, "max_gap_ms": 250.0, "count": 100, "rate_hz": 99.0,
                                              "nominal_ms": 10.0, "limit_ms": 110.0}}
        verdict = signoff(counts, ["/fmu/out/vehicle_odometry", "/avia/points"], gaps, 100.0)
        self.assertFalse(verdict["pass"])
        self.assertEqual(verdict["fmu_in_messages"], 3)
        self.assertEqual(verdict["reasons"], ["3 message(s) on /fmu/in topics", "no messages on /avia/points",
                                              "2 gap(s) over 110 ms on /fmu/out/vehicle_odometry (nominal 10 ms, worst 250 ms)"])
        self.assertTrue(signoff({"/a": 1}, ["/a"], {"/a": {"gaps_over": 0}}, 100.0)["pass"])

    def test_iso_to_ns(self):
        self.assertEqual(iso_to_ns("2026-10-04T17:43:36+00:00"), 1791135816 * 1_000_000_000)
        self.assertIsNone(iso_to_ns(None))
        self.assertIsNone(iso_to_ns("junk"))

    def test_run_without_mcap_library_still_signs_off_from_metadata(self):
        with tempfile.TemporaryDirectory() as bag:
            with open(os.path.join(bag, "metadata.yaml"), "w") as f:
                f.write(METADATA)
            with open(os.path.join(bag, "flight.json"), "w") as f:
                json.dump({"bag": bag, "started_at": "2026-10-04T17:43:36+00:00", "armed_at": None,
                           "stopped_at": "2026-10-04T17:54:00+00:00", "duration_sec": 18.3, "bytes": 184617818,
                           "stop_reason": "test", "recorder_exit_code": 0}, f)
            meta = run(bag, ["/fmu/out/vehicle_odometry", "/e1r/points"], ["/avia/points"], 100.0, None, 1.0, log=lambda *_: None)
            self.assertFalse(meta["signoff"]["pass"])
            self.assertIn("3 message(s) on /fmu/in topics", meta["signoff"]["reasons"])
            self.assertIn("no messages on /e1r/points", meta["signoff"]["reasons"])
            self.assertEqual(meta["postflight"]["counts_source"], "metadata.yaml")
            self.assertEqual(meta["postflight"]["required"]["/fmu/out/vehicle_odometry"], 1718)
            card = open(os.path.join(bag, "flight_card.md")).read()
            self.assertIn("FAIL", card)
            self.assertIn("| /fmu/in messages | 3 |", card)
            self.assertTrue(flight_card(meta).startswith("# Flight card: "))


if __name__ == "__main__":
    unittest.main()
