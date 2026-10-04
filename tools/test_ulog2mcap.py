import json
import os
import tempfile
import unittest

from ulog2mcap import (cross_check, interpolate, json_schema, plain, pps_series, residual_stats, timesync_series,
                       to_utc_us)

OFFSET = -1_791_132_374_966_676      # px4 - companion, as the live FC reports it (negative: UTC epoch is huge)


class PureTests(unittest.TestCase):
    def test_timesync_series_filters_to_dds_and_dedups(self):
        stamps = [100, 50, 50, 200]
        offsets = [OFFSET + 5, OFFSET, OFFSET + 1, OFFSET + 10]
        protocols = [2, 2, 2, 1]
        self.assertEqual(timesync_series(stamps, offsets, protocols), [(50, OFFSET + 1), (100, OFFSET + 5)])

    def test_interpolation_holds_edges(self):
        s = [(100, 10), (200, 20)]
        self.assertEqual(interpolate(s, 50), 10)
        self.assertEqual(interpolate(s, 150), 15)
        self.assertEqual(interpolate(s, 250), 20)
        self.assertIsNone(interpolate([], 1))

    def test_utc_and_pps_cross_check(self):
        series = [(1_000_000, OFFSET), (61_000_000, OFFSET + 30)]     # 0.5 ppm drift over a minute
        utc = to_utc_us(series, 31_000_000)
        self.assertEqual(utc, 31_000_000 - (OFFSET + 15))
        # PPS edges: hrt at exact UTC seconds given the same offset, plus a 1.2 ms error on one
        rtc = [1_791_132_406_000_000, 1_791_132_407_000_000]
        hrt = [r + OFFSET + 10 for r in rtc]
        hrt[1] += 1200
        stats = cross_check(series, hrt, rtc)
        self.assertEqual(stats["count"], 2)
        self.assertLess(stats["median_us"], 1300)
        self.assertAlmostEqual(stats["max_us"], 1200, delta=20)        # the injected error, within interpolation
        # a pps series built from the edges reproduces the offset
        self.assertEqual(pps_series(hrt, rtc)[0][1], OFFSET + 10)
        self.assertEqual(cross_check(series, [5], [0])["count"], 0)      # rtc 0 = no reference

    def test_residual_stats_and_schema(self):
        self.assertEqual(residual_stats([])["count"], 0)
        s = residual_stats([-3, 1, 2, 10])
        self.assertEqual((s["count"], s["median_us"], s["max_us"]), (4, 2.0, 10.0))
        schema = json_schema(["timestamp", "x", "ok", "q[0]"], ["uint64_t", "float", "bool", "float"])
        self.assertEqual(schema["properties"]["x"], {"type": "number"})
        self.assertEqual(schema["properties"]["ok"], {"type": "boolean"})
        self.assertEqual(schema["properties"]["timestamp"], {"type": "integer"})
        self.assertIsNone(plain(float("nan")))


class EndToEndTests(unittest.TestCase):
    def test_sample_log_converts(self):
        try:
            import pyulog  # noqa: F401
            import mcap  # noqa: F401
        except ImportError:
            self.skipTest("pyulog/mcap not installed")
        sample = os.environ.get("ULOG_SAMPLE")
        if not sample or not os.path.isfile(sample):
            self.skipTest("ULOG_SAMPLE not set")
        from ulog2mcap import convert
        from mcap.reader import make_reader
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, "out.mcap")
            summary = convert(sample, out, "auto", ["vehicle_attitude", "vehicle_status"], log=lambda *_: None)
            self.assertEqual(summary["datasets"], 2)
            with open(out, "rb") as f:
                reader = make_reader(f)
                topics = {c.topic for c in reader.get_summary().channels.values()}
                self.assertIn("/ulog/vehicle_attitude", topics)
                self.assertIn("/ulog/parameters", topics)
                meta = [m for m in reader.iter_metadata() if m.name == "ulog2mcap"]
                self.assertEqual(len(meta), 1)
                n = sum(1 for _ in reader.iter_messages(topics=["/ulog/vehicle_attitude"]))
                self.assertEqual(n, summary["channels"]["/ulog/vehicle_attitude"])
                first = next(reader.iter_messages(topics=["/ulog/vehicle_attitude"]))
                row = json.loads(first[2].data)
                self.assertIn("utc_us", row)


if __name__ == "__main__":
    unittest.main()
