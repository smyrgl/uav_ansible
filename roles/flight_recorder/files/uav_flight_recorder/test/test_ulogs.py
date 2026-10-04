import json
import os
import tempfile
import unittest
from datetime import datetime, timezone

from uav_flight_recorder.ulogs import bag_for, bags_armed_at, load_state, log_start, recent_days, save_state, wanted


class UlogTests(unittest.TestCase):
    def test_log_start_from_px4_names(self):
        self.assertEqual(log_start("2026-10-05", "15_42_07.ulg"), datetime(2026, 10, 5, 15, 42, 7, tzinfo=timezone.utc))
        self.assertIsNone(log_start("2026-10-05", "session.ulg"))
        self.assertIsNone(log_start("junk", "15_42_07.ulg"))

    def test_recent_days(self):
        today = datetime(2026, 10, 5, 12, tzinfo=timezone.utc)
        self.assertEqual(recent_days(3, today), ["2026-10-05", "2026-10-04", "2026-10-03"])

    def test_wanted_skips_fetched_at_same_size(self):
        state = {"2026-10-05/15_42_07.ulg": {"size": 100, "fetched": True}}
        entries = [("15_42_07.ulg", 100), ("16_00_00.ulg", 5), ("notes.txt", 1), ("15_42_07.ulg", 150)]
        self.assertEqual(wanted(entries, "2026-10-05", state), [("16_00_00.ulg", 5), ("15_42_07.ulg", 150)])

    def test_bag_matching_by_armed_time(self):
        bags = {"/f/a": "2026-10-05T15:41:50+00:00", "/f/b": "2026-10-05T16:30:00+00:00", "/f/c": None}
        start = datetime(2026, 10, 5, 15, 42, 7, tzinfo=timezone.utc)
        self.assertEqual(bag_for(start, bags), "/f/a")
        self.assertIsNone(bag_for(datetime(2026, 10, 5, 15, 0, tzinfo=timezone.utc), bags))

    def test_state_and_bag_scan(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "fetched.json")
            save_state(p, {"x": {"size": 1}})
            self.assertEqual(load_state(p), {"x": {"size": 1}})
            self.assertEqual(load_state(os.path.join(d, "missing.json")), {})
            os.makedirs(os.path.join(d, "flight_1")); os.makedirs(os.path.join(d, "other"))
            with open(os.path.join(d, "flight_1", "flight.json"), "w") as f:
                json.dump({"armed_at": "2026-10-05T15:41:50+00:00"}, f)
            self.assertEqual(bags_armed_at(d), {os.path.join(d, "flight_1"): "2026-10-05T15:41:50+00:00"})


if __name__ == "__main__":
    unittest.main()
