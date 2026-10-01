import unittest

from uav_flight_recorder.policy import FlightPolicy, IDLE, RECORDING, POST_ROLL


class PolicyTests(unittest.TestCase):
    def test_arm_record_disarm_post_roll(self):
        p = FlightPolicy(post_roll_sec=30, safed_grace_sec=5)
        self.assertEqual(p.update(0, armed=False, safety_off=True).action, None)
        d = p.update(1, armed=True)
        self.assertEqual((d.action, d.state, d.reason), ("start", RECORDING, "armed"))
        self.assertEqual(p.update(100, armed=False).state, POST_ROLL)
        self.assertEqual(p.tick(120).action, None)            # 20 s into post-roll, safety still off
        d = p.tick(131)
        self.assertEqual((d.action, d.state, d.reason), ("stop", IDLE, "post-roll elapsed"))

    def test_safing_stops_early_and_rearm_continues(self):
        p = FlightPolicy(post_roll_sec=30, safed_grace_sec=5)
        p.update(0, armed=True, safety_off=True)
        p.update(50, armed=False)                             # post-roll from t=50
        d = p.update(60, armed=True)
        self.assertEqual((d.action, d.state), (None, RECORDING))   # same bag continues
        p.update(90, armed=False)
        p.update(92, safety_off=False)                        # safed at 92
        self.assertEqual(p.tick(96).action, None)
        d = p.tick(97)
        self.assertEqual((d.action, d.reason), ("stop", "vehicle safed"))

    def test_disarm_while_already_safed_uses_grace_only(self):
        p = FlightPolicy(post_roll_sec=30, safed_grace_sec=5)
        p.update(0, armed=True, safety_off=False)
        p.update(10, armed=False)                             # safety already engaged at disarm
        self.assertEqual(p.tick(14).action, None)
        self.assertEqual(p.tick(15).action, "stop")

    def test_manual_recording_stops_when_withdrawn(self):
        p = FlightPolicy()
        d = p.update(0, manual=True)
        self.assertEqual((d.action, d.reason), ("start", "manual request"))
        d = p.update(5, manual=False)
        self.assertEqual((d.action, d.state, d.reason), ("stop", IDLE, "manual request withdrawn"))
        # a manual request that overlaps a real flight behaves like a flight
        p.update(10, manual=True); p.update(11, armed=True); p.update(12, manual=False)
        self.assertEqual(p.update(20, armed=False).state, POST_ROLL)
        self.assertEqual(p.tick(51).action, "stop")


if __name__ == "__main__":
    unittest.main()
