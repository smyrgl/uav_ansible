"""FAST-LIO health: divergence evidence and verdicts, and the consumers' view."""
import unittest

import lio_health as h


def feed(monitor, start, end, every=0.1, line="No Effective Points!"):
    t = start
    while t <= end + 1e-9:
        monitor.log_line(t, line)
        t += every
    return t


class Monitor(unittest.TestCase):
    def test_a_featureless_moment_is_degraded_not_diverged(self):
        m = h.DivergenceMonitor()
        feed(m, 10.0, 10.4)                                 # four failed scans
        self.assertEqual(m.state, h.DEGRADED)
        self.assertEqual(m.update(11.5), h.OK)              # the run ended
        self.assertEqual(m.epoch, 0)

    def test_a_sustained_run_is_a_divergence_dated_to_its_start(self):
        m = h.DivergenceMonitor(sustain_s=2.0, onset_margin_s=1.0)
        feed(m, 100.0, 101.9)
        self.assertEqual(m.state, h.DEGRADED)
        feed(m, 102.0, 102.0)
        v = m.verdict()
        self.assertEqual(v["state"], h.DIVERGED)
        self.assertAlmostEqual(v["onset_mono"], 99.0)
        self.assertAlmostEqual(v["since_mono"], 102.0)
        self.assertIn("no effective points", v["reason"])
        self.assertEqual(m.update(500.0), h.DIVERGED)        # holds: FAST-LIO does not come back by itself

    def test_short_gaps_do_not_break_a_run_long_ones_do(self):
        m = h.DivergenceMonitor(sustain_s=2.0, gap_s=0.5)
        feed(m, 0.0, 1.0)
        feed(m, 1.4, 2.1)                                   # 0.4 s gap: the same run
        self.assertEqual(m.state, h.DIVERGED)
        m2 = h.DivergenceMonitor(sustain_s=2.0, gap_s=0.5)
        feed(m2, 0.0, 1.0)
        feed(m2, 1.7, 2.6)                                  # 0.7 s gap: a new run of 0.9 s
        self.assertEqual(m2.state, h.DEGRADED)

    def test_fastlio_initialising_starts_a_new_epoch(self):
        m = h.DivergenceMonitor()
        feed(m, 0.0, 3.0)
        self.assertEqual(m.state, h.DIVERGED)
        m.log_line(50.0, "IMU Initial Done")
        v = m.verdict()
        self.assertEqual((v["state"], v["epoch"], v["onset_mono"]), (h.OK, 1, None))
        m.log_line(50.2, "[WARN] [laser_mapping]: No point, skip this scan!")   # the usual first scan
        self.assertEqual(m.state, h.DEGRADED)
        self.assertEqual(m.update(51.0), h.OK)

    def test_odometry_leaps_and_restarts(self):
        m = h.DivergenceMonitor(sustain_s=2.0, max_speed_mps=30.0, max_leap_m=5.0)
        for i in range(30):                                 # 1 m/s: fine
            m.pose(10 + i * 0.1, 1000 + i * 0.1, (i * 0.1, 0.0, 0.0))
        self.assertEqual((m.state, m.evidence_total), (h.OK, 0))
        for i in range(30):                                 # 100 m/s: a runaway
            m.pose(13 + i * 0.1, 1003 + i * 0.1, (3 + i * 10.0, 0.0, 0.0))
        self.assertEqual(m.state, h.DIVERGED)
        self.assertIn("implausible motion", m.reason)
        m.pose(20.0, 900.0, (0.0, 0.0, 0.0))                # stamps went back: FAST-LIO started over
        self.assertEqual((m.state, m.epoch), (h.OK, 1))

    def test_the_reason_stays_one_phrase_with_the_peak_speed(self):
        """Live on 2026-10-03 every speed became its own 'kind': the reason grew without bound."""
        m = h.DivergenceMonitor(sustain_s=2.0)
        for i in range(40):                                 # an accelerating runaway, 31 m/s and up
            m.pose(i * 0.1, 1000 + i * 0.1, ((i * i) * 0.5 + i * 3.1, 0.0, 0.0))
        self.assertEqual(m.state, h.DIVERGED)
        self.assertEqual(m.reason.count("implausible motion"), 1)
        self.assertRegex(m.reason, r"^implausible motion \(up to \d+ m/s\) for \d\.\d s$")

    def test_other_log_lines_are_ignored(self):
        m = h.DivergenceMonitor()
        for i in range(50):
            m.log_line(i * 0.1, "[INFO] [laser_mapping]: Initialize the map kdtree")
        self.assertEqual((m.state, m.evidence_total), (h.OK, 0))


class Lost(unittest.TestCase):
    """The patched FAST-LIO exits when lost; its last line is a divergence at once."""

    LINE = "FAST-LIO lost: no effective points for (s) 3.1, exiting for a restart"

    def test_lost_line_diverges_at_once_dated_to_the_run(self):
        m = h.DivergenceMonitor(sustain_s=5.0, onset_margin_s=1.0)   # the run alone would not be enough yet
        feed(m, 10.0, 11.0)
        self.assertEqual(m.state, h.DEGRADED)
        m.log_line(11.05, self.LINE)
        v = m.verdict()
        self.assertEqual(v["state"], h.DIVERGED)
        self.assertAlmostEqual(v["onset_mono"], 9.0)
        self.assertTrue(v["reason"].startswith("FAST-LIO lost: no effective points"))
        m.log_line(14.0, "IMU Initial Done")                          # systemd restarted it
        self.assertEqual((m.state, m.epoch), (h.OK, 1))

    def test_lost_without_a_run_dates_to_now(self):
        m = h.DivergenceMonitor(onset_margin_s=1.0)
        m.log_line(50.0, "FAST-LIO lost: speed (m/s) 41.2, exiting for a restart")
        self.assertEqual(m.state, h.DIVERGED)
        self.assertAlmostEqual(m.verdict()["onset_mono"], 49.0)


class Follower(unittest.TestCase):
    def v(self, state, epoch, session=1):
        return {"state": state, "epoch": epoch, "session": session, "reason": ""}

    def test_events(self):
        f = h.HealthFollower()
        self.assertIsNone(f.update(self.v(h.OK, 0)))
        self.assertEqual(f.update(self.v(h.DIVERGED, 0)), "diverged")
        self.assertTrue(f.diverged)
        self.assertIsNone(f.update(self.v(h.DIVERGED, 0)))   # still diverged: no new event
        self.assertEqual(f.update(self.v(h.OK, 1)), "restarted")
        self.assertFalse(f.diverged)

    def test_a_restarted_watchdog_is_not_fastlio_restarting(self):
        f = h.HealthFollower()
        f.update(self.v(h.OK, 4, session=1))
        self.assertIsNone(f.update(self.v(h.OK, 0, session=2)))
        self.assertEqual(f.update(self.v(h.DIVERGED, 0, session=3)), "diverged")

    def test_first_verdict_diverged(self):
        self.assertEqual(h.HealthFollower().update(self.v(h.DIVERGED, 0)), "diverged")


class Wire(unittest.TestCase):
    def test_round_trip_and_garbage(self):
        m = h.DivergenceMonitor()
        text = h.encode(m.verdict(), session=7, restarts=0)
        self.assertEqual(h.decode(text)["session"], 7)
        for bad in ("", "{", '{"state": "fine", "epoch": 0}', '{"state": "ok"}', "[1, 2]"):
            self.assertIsNone(h.decode(bad))


if __name__ == "__main__":
    unittest.main()
