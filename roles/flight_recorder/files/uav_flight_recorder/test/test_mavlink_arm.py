"""Run with the recorder's venv python (pymavlink); parser tests skip without it."""
import unittest

from uav_flight_recorder.mavlink_arm import MAV_MODE_FLAG_SAFETY_ARMED, RcSwitch, armed_from_base_mode

try:
    from pymavlink.dialects.v20 import common as mavlink
    from uav_flight_recorder.mavlink_arm import AutopilotLink, AutopilotParser
    HAVE_PYMAVLINK = True
except ImportError:
    HAVE_PYMAVLINK = False


def heartbeat(system, component, base_mode, mav_type=2):
    sender = mavlink.MAVLink(None, srcSystem=system, srcComponent=component)
    msg = mavlink.MAVLink_heartbeat_message(mav_type, 12, base_mode, 0, 4, 3)
    return msg.pack(sender)


REST = [1499] * 4 + [999, 1499, 1499, 1499, 1499, 1049, 1049, 1049, 1049, 1499, 1499, 1999, 998, 998]


def rc(system=1, component=1, **channels):
    values = list(REST)
    for key, raw in channels.items():
        values[int(key[2:]) - 1] = raw
    sender = mavlink.MAVLink(None, srcSystem=system, srcComponent=component)
    return mavlink.MAVLink_rc_channels_message(0, 18, *values, 255).pack(sender)


def sys_status(rc_present=True):
    bit = 65536 if rc_present else 0
    sender = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    return mavlink.MAVLink_sys_status_message(bit, bit, bit, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0).pack(sender)


class ArmedFlag(unittest.TestCase):
    def test_safety_armed_bit(self):
        self.assertTrue(armed_from_base_mode(MAV_MODE_FLAG_SAFETY_ARMED | 0x1D))
        self.assertFalse(armed_from_base_mode(0x1D))


class Switch(unittest.TestCase):
    def frames(self, switch, values, t0=0.0):
        for i, v in enumerate(values):
            switch.frame(v, t0 + 0.2 * i)

    def on(self):
        values = list(REST)
        values[10] = 1949
        return values

    def test_level_needs_the_receiver_then_debounces(self):
        s = RcSwitch(11)
        self.frames(s, [self.on(), self.on()])
        self.assertIsNone(s.level)                       # no SYS_STATUS yet
        s.set_present(True)
        self.frames(s, [REST, REST, self.on()], t0=0.4)
        self.assertFalse(s.level)                        # one frame on: not yet
        s.frame(self.on(), 1.0)
        self.assertTrue(s.level)
        self.assertEqual(s.presses, 1)

    def test_baseline_after_loss_or_gap_is_not_a_press(self):
        s = RcSwitch(11)
        s.set_present(True)
        self.frames(s, [REST, REST])
        s.set_present(False)
        self.assertIsNone(s.level)
        s.set_present(True)
        self.frames(s, [self.on(), self.on()], t0=0.5)
        self.assertTrue(s.level)
        self.assertEqual(s.presses, 0)                   # it was already on: a baseline
        self.frames(s, [REST, REST], t0=5.0)             # after a gap: also a baseline
        self.assertFalse(s.level)
        self.assertEqual(s.presses, 0)


@unittest.skipUnless(HAVE_PYMAVLINK, "pymavlink not installed")
class Parser(unittest.TestCase):
    def test_only_the_autopilot_counts(self):
        p = AutopilotParser()
        self.assertEqual(p.feed(heartbeat(1, 100, 128, mav_type=30)), [])   # our camera component
        self.assertEqual(p.feed(heartbeat(255, 190, 128, mav_type=6)), [])  # a ground station
        self.assertEqual(p.feed(rc(255, 190, ch11=1949)), [])
        self.assertEqual(p.feed(heartbeat(1, 1, 128 | 0x1D) + heartbeat(1, 1, 0x1D)),
                         [("armed", True), ("armed", False)])

    def test_rc_and_receiver_presence(self):
        p = AutopilotParser()
        events = p.feed(sys_status() + rc(ch11=1949))
        self.assertEqual(events[0], ("rc_present", True))
        self.assertEqual((events[1][0], events[1][1][10], len(events[1][1])), ("rc", 1949, 18))
        self.assertEqual(p.feed(sys_status(False)), [("rc_present", False)])

    def test_split_frames_and_noise_parse(self):
        p = AutopilotParser()
        frame = heartbeat(1, 1, 128 | 0x1D)
        self.assertEqual(p.feed(b"\x00garbage" + frame[:5]), [])
        self.assertEqual(p.feed(frame[5:]), [("armed", True)])

    def test_link_folds_events_into_the_snapshot(self):
        link = AutopilotLink("tcp:127.0.0.1:9", rc_channel=11)
        parser = AutopilotParser()
        link.apply(parser.feed(heartbeat(1, 1, 0x1D) + sys_status() + rc() + rc(ch11=1949) + rc(ch11=1949)), 10.0)
        snap = link.snapshot(10.5)
        self.assertEqual((snap["armed"], snap["rc_present"], snap["rc_level"], snap["rc_presses"]),
                         (False, True, True, 1))


@unittest.skipUnless(HAVE_PYMAVLINK, "pymavlink not installed")
class Reader(unittest.TestCase):
    def test_reads_a_router_and_keeps_state_when_it_goes(self):
        import socket
        import time
        server = socket.create_server(("127.0.0.1", 0))
        reader = AutopilotLink("tcp:127.0.0.1:%d" % server.getsockname()[1])
        reader.start()
        try:
            conn, _ = server.accept()
            conn.sendall(heartbeat(1, 1, 0x1D) + heartbeat(1, 100, 128, mav_type=30) + heartbeat(1, 1, 128 | 0x1D))
            deadline = time.monotonic() + 3
            while reader.snapshot()["heartbeats"] < 2 and time.monotonic() < deadline:
                time.sleep(0.02)
            snap = reader.snapshot()
            self.assertEqual((snap["armed"], snap["heartbeats"], snap["connected"]), (True, 2, True))
            conn.close()
            deadline = time.monotonic() + 3
            while reader.snapshot()["connected"] and time.monotonic() < deadline:
                time.sleep(0.02)
            snap = reader.snapshot()
            self.assertFalse(snap["connected"])
            self.assertTrue(snap["armed"])          # a lost link keeps the last state
        finally:
            reader.close()
            server.close()


if __name__ == "__main__":
    unittest.main()


def test_mission_current_is_folded_into_the_snapshot():
    from uav_flight_recorder.mavlink_arm import AutopilotLink
    link = AutopilotLink("tcp:127.0.0.1:1", 0)
    link.apply([("mission", 7), ("armed", True)], 10.0)
    assert link.snapshot(10.0)["mission_seq"] == 7
