"""Check the ROS generated type boundary without starting a node or DDS."""

import unittest

try:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from rclpy.serialization import deserialize_message, serialize_message
    from uav_sensor_health.node import SensorHealth
except ImportError:
    ROS_AVAILABLE = False
else:
    ROS_AVAILABLE = True

from uav_sensor_health.core import Assessment, OK, WARN, ERROR, STALE


@unittest.skipUnless(ROS_AVAILABLE, "requires the target ROS environment")
class TestDiagnosticSerialization(unittest.TestCase):
    def test_every_level_survives_ros_serialization(self):
        statuses = []
        for level, expected in (
            (OK, DiagnosticStatus.OK), (WARN, DiagnosticStatus.WARN),
            (ERROR, DiagnosticStatus.ERROR), (STALE, DiagnosticStatus.STALE),
        ):
            with self.subTest(level=level):
                status = SensorHealth._status(
                    "D555 Camera", "261622302751",
                    Assessment(level, "test message", {"rate_hz": 30.0, "verified": False}))
                restored = deserialize_message(serialize_message(status), DiagnosticStatus)
                self.assertEqual(restored.level, expected)
                self.assertEqual(restored.name, "D555 Camera")
                self.assertEqual(restored.hardware_id, "")
                self.assertEqual(restored.message, "test message")
                self.assertEqual({v.key: v.value for v in restored.values},
                                 {"rate_hz": "30.0", "verified": "False", "Identity/source_id": "261622302751"})
                statuses.append(status)
        batch = DiagnosticArray(status=statuses)
        restored = deserialize_message(serialize_message(batch), DiagnosticArray)
        self.assertEqual([s.level for s in restored.status],
                         [DiagnosticStatus.OK, DiagnosticStatus.WARN,
                          DiagnosticStatus.ERROR, DiagnosticStatus.STALE])


if __name__ == "__main__":
    unittest.main()
