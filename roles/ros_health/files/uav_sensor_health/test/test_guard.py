"""The guard's hash port is checked against the numbers PX4's own generator put
in the FC build (uav/v1.17.0-pps at 596ee0b8f3, ORB_DEFINE's fourth argument);
the definitions are the FC tree's .msg files verbatim."""
import unittest

from uav_sensor_health.core import ERROR, OK, WARN
from uav_sensor_health.guard import (c_string, c_string_field, fields_text, guard_health, mavlink_write,
                                     message_hash, parse_fields, type_name)

DEFINITIONS = {
    "MessageFormatRequest": """uint64 timestamp # time since system start (microseconds)

# Request to PX4 to get the hash of a message, to check for message compatibility

uint16 LATEST_PROTOCOL_VERSION = 1 # Current version of this protocol. Increase this whenever the MessageFormatRequest or MessageFormatResponse changes.

uint16 protocol_version           # Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp

char[50] topic_name  # E.g. /fmu/in/vehicle_command
""",
    "MessageFormatResponse": """uint64 timestamp # time since system start (microseconds)

# Response from PX4 with the format of a message

uint16 protocol_version           # Must be set to LATEST_PROTOCOL_VERSION. Do not change this field, it must be the first field after the timestamp

char[50] topic_name  # E.g. /fmu/in/vehicle_command

bool success
uint32 message_hash # hash over all message fields
""",
    "EscReport": """uint64 timestamp					# time since system start (microseconds)
uint32 esc_errorcount					# Number of reported errors by ESC - if supported
int32 esc_rpm						# Motor RPM, negative for reverse rotation [RPM] - if supported
float32 esc_voltage					# Voltage measured from current ESC [V] - if supported
float32 esc_current					# Current measured from current ESC [A] - if supported
float32 esc_temperature					# Temperature measured from current ESC [degC] - if supported
uint8 esc_address					# Address of current ESC (in most cases 1-8 / must be set by driver)
uint8 esc_cmdcount					# Counter of number of commands

uint8 esc_state					# State of ESC - depend on Vendor

uint8 actuator_function				# actuator output function (one of Motor1...MotorN)

uint16 failures					# Bitmask to indicate the internal ESC faults
int8 esc_power					# Applied power 0-100 in % (negative values reserved)

uint8 FAILURE_OVER_CURRENT = 0 			# (1 << 0)
uint8 FAILURE_OVER_VOLTAGE = 1 			# (1 << 1)
uint8 FAILURE_MOTOR_OVER_TEMPERATURE = 2 	# (1 << 2)
uint8 FAILURE_OVER_RPM = 3			# (1 << 3)
uint8 FAILURE_INCONSISTENT_CMD = 4 		# (1 << 4)  Set if ESC received an inconsistent command (i.e out of boundaries)
uint8 FAILURE_MOTOR_STUCK = 5			# (1 << 5)
uint8 FAILURE_GENERIC = 6			# (1 << 6)
uint8 FAILURE_MOTOR_WARN_TEMPERATURE = 7	# (1 << 7)
uint8 FAILURE_WARN_ESC_TEMPERATURE = 8		# (1 << 8)
uint8 FAILURE_OVER_ESC_TEMPERATURE = 9		# (1 << 9)
uint8 ESC_FAILURE_COUNT = 10 			# Counter - keep it as last element!
""",
    "EscStatus": """uint64 timestamp					# time since system start (microseconds)
uint8 CONNECTED_ESC_MAX = 8				# The number of ESCs supported. Current (Q2/2013) we support 8 ESCs

uint8 ESC_CONNECTION_TYPE_PPM = 0			# Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1			# Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOT = 2			# One Shot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3			# I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4			# CAN-Bus
uint8 ESC_CONNECTION_TYPE_DSHOT = 5			# DShot

uint16 counter  					# incremented by the writing thread everytime new data is stored

uint8 esc_count						# number of connected ESCs
uint8 esc_connectiontype				# how ESCs connected to the system

uint8 esc_online_flags					# Bitmask indicating which ESC is online/offline
# esc_online_flags bit 0 : Set to 1 if ESC0 is online

uint8 esc_armed_flags					# Bitmask indicating which ESC is armed. For ESC's where the arming state is not known (returned by the ESC), the arming bits should always be set.

EscReport[8] esc
""",
    "TimesyncStatus": """uint64 timestamp			# time since system start (microseconds)

uint8 SOURCE_PROTOCOL_UNKNOWN = 0
uint8 SOURCE_PROTOCOL_MAVLINK = 1
uint8 SOURCE_PROTOCOL_DDS     = 2
uint8 source_protocol			# timesync source

uint64 remote_timestamp			# remote system timestamp (microseconds)
int64 observed_offset			# raw time offset directly observed from this timesync packet (microseconds)
int64 estimated_offset			# smoothed time offset between companion system and PX4 (microseconds)
uint32 round_trip_time			# round trip time of this timesync packet (microseconds)
""",
    "PpsCapture": """uint64 timestamp			  # time since system start (microseconds) at PPS capture event
uint64 rtc_timestamp		# Corrected GPS UTC timestamp at PPS capture event
uint8  pps_rate_exceeded_counter # Increments when PPS dt < 50ms
""",
}
# ORB_DEFINE(<topic>, struct, size, <hash>u, ...) in build/px4_fmu-v6x_default, 2026-10-04.
FC_HASHES = {"message_format_request": 618794167, "message_format_response": 1621134381,
             "esc_report": 1089051630, "esc_status": 2881296114, "timesync_status": 3493470941,
             "pps_capture": 3333608066}


def resolve(name):
    return DEFINITIONS[name]


class HashPortTests(unittest.TestCase):
    def test_hashes_match_the_fc_build_including_a_nested_array(self):
        for uorb, expected in FC_HASHES.items():
            self.assertEqual(message_hash(resolve(type_name(uorb)), resolve), expected, uorb)

    def test_field_text_is_px4s_format(self):
        self.assertEqual(fields_text(resolve("PpsCapture"), resolve),
                         "uint64 timestamp\nuint64 rtc_timestamp\nuint8 pps_rate_exceeded_counter\n")
        self.assertTrue(fields_text(resolve("EscStatus"), resolve).endswith(
            "EscReport[8] esc\nuint64 timestamp\nuint32 esc_errorcount\nint32 esc_rpm\nfloat32 esc_voltage\n"
            "float32 esc_current\nfloat32 esc_temperature\nuint8 esc_address\nuint8 esc_cmdcount\nuint8 esc_state\n"
            "uint8 actuator_function\nuint16 failures\nint8 esc_power\n"))

    def test_parse_skips_constants_comments_and_handles_package_prefixes(self):
        self.assertEqual(parse_fields("uint8 X = 3\n# c\npx4/EscReport[8] esc  # n\nstring  s\n"),
                         [("px4/EscReport[8]", "esc"), ("string", "s")])
        self.assertEqual(fields_text("px4/EscReport[2] e\n", resolve).splitlines()[0], "EscReport[2] e")

    def test_names_and_char_fields(self):
        self.assertEqual(type_name("vehicle_local_position_setpoint"), "VehicleLocalPositionSetpoint")
        self.assertEqual(type_name("sensor_gps"), "SensorGps")
        field = c_string_field("/fmu/out/vehicle_odometry")
        self.assertEqual(len(field), 50)
        self.assertEqual(c_string(field), "/fmu/out/vehicle_odometry")
        self.assertEqual(c_string(b"/fmu/in/x\0\0"), "/fmu/in/x")
        self.assertEqual(c_string("/fmu/in/y\0"), "/fmu/in/y")


class MavlinkWriteTests(unittest.TestCase):
    def test_classification(self):
        self.assertFalse(mavlink_write("HEARTBEAT", 254, 191, None))
        self.assertFalse(mavlink_write("COMMAND_LONG", 255, 190, 176))          # the GCS may command
        self.assertFalse(mavlink_write("COMMAND_LONG", 1, 1, 176))              # the FC itself
        self.assertFalse(mavlink_write("COMMAND_LONG", 254, 191, 512))          # REQUEST_MESSAGE is read-only
        self.assertTrue(mavlink_write("COMMAND_LONG", 254, 191, 176))           # DO_SET_MODE from the companion
        self.assertTrue(mavlink_write("PARAM_SET", 1, 100, None))               # a companion component writing a param
        self.assertTrue(mavlink_write("SET_POSITION_TARGET_LOCAL_NED", 42, 1, None))


class GuardHealthTests(unittest.TestCase):
    EXPECTED = ("/fmu/in/message_format_request",)

    def hashes(self, state="done", mismatch=False, unanswered=False):
        fc = 5 if mismatch else 7
        return {"vehicle_odometry": {"local": 7, "fc": None if unanswered else fc, "answered": not unanswered},
                "trajectory_setpoint": {"local": 9, "fc": 9, "answered": True}}

    def test_clean_graph_is_ok(self):
        r = guard_health({}, ["/fmu/in/message_format_request"], self.EXPECTED, self.hashes(), "done",
                         {"count": 0, "last": None, "last_mono": None}, 100.0)
        self.assertEqual(r.level, OK)
        self.assertEqual(r.message, "No write path to the FC")
        self.assertEqual(r.values["Message hashes/topic/vehicle_odometry"], "match")

    def test_a_publisher_on_fmu_in_is_error(self):
        r = guard_health({"/fmu/in/trajectory_setpoint": ["/rogue"]}, ["/fmu/in/message_format_request"], self.EXPECTED,
                         self.hashes(), "done", {"count": 0, "last": None, "last_mono": None}, 100.0)
        self.assertEqual(r.level, ERROR)
        self.assertIn("publisher on /fmu/in", r.message)
        self.assertIn("/fmu/in/trajectory_setpoint <- /rogue", r.values["Writers/message"])

    def test_old_firmware_and_hash_mismatch_are_warnings(self):
        readers = ["/fmu/in/message_format_request", "/fmu/in/trajectory_setpoint", "/fmu/in/vehicle_command"]
        r = guard_health({}, readers, self.EXPECTED, self.hashes(mismatch=True), "done",
                         {"count": 0, "last": None, "last_mono": None}, 100.0)
        self.assertEqual(r.level, WARN)
        self.assertIn("firmware still accepts 2 input topic(s)", r.message)
        self.assertIn("old firmware?", r.values["Firmware inputs/message"])
        self.assertEqual(r.values["Message hashes/topic/vehicle_odometry"], "MISMATCH local 7 fc 5")
        self.assertIn("1 differ", r.values["Message hashes/message"])

    def test_unknown_hash_state_is_named_and_a_companion_write_is_error(self):
        r = guard_health({}, ["/fmu/in/message_format_request"], self.EXPECTED, self.hashes(unanswered=True), "unknown",
                         {"count": 0, "last": None, "last_mono": None}, 100.0)
        self.assertEqual(r.level, WARN)
        self.assertIn("unknown: waiting for the FC", r.values["Message hashes/message"])
        self.assertEqual(r.values["Message hashes/topic/vehicle_odometry"], "unanswered")
        r = guard_health({}, ["/fmu/in/message_format_request"], self.EXPECTED, self.hashes(), "done",
                         {"count": 2, "last": {"type": "COMMAND_LONG", "command": 176, "system": 254, "component": 191},
                          "last_mono": 90.0}, 100.0)
        self.assertEqual(r.level, ERROR)
        self.assertIn("companion MAVLink writes", r.message)
        self.assertIn("COMMAND_LONG cmd 176 from 254/191 10 s ago", r.values["MAVLink writes/message"])


if __name__ == "__main__":
    unittest.main()
