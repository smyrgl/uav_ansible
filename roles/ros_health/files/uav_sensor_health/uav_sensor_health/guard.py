"""The PX4 input guard: is there any way for this computer to steer the FC?

Four properties, each a section of the "PX4 / Guard" row:

- Writers: no ROS publisher on any ``/fmu/in`` topic (the XRCE client republishes
  any setpoint it receives to uORB with no mode gate). Only this node's own
  message-format request is allowed, and only while a check runs.
- Firmware inputs: the FC advertises no ``/fmu/in`` subscription except
  ``message_format_request`` (the uav/v1.17.0-pps firmware deletes the rest).
- Message hashes: the px4_msgs definitions this host was built with match the
  FC's, by PX4's own 32-bit FNV-1a hash over the field list (the same number
  the FC returns in MessageFormatResponse). "unknown" until the FC answers.
- MAVLink writes: no command, mode, parameter or setpoint message from a source
  other than the ground station reached the router (it is the second write
  channel; the sniffer only sees it all when the router forwards everything
  to this node's system id).

Pure functions; the node feeds graph snapshots and responses.
"""
import os
import re

from .health import check, grouped

BUILTIN_TYPES = frozenset((
    "bool", "byte", "char", "int8", "uint8", "int16", "uint16", "int32", "uint32",
    "int64", "uint64", "float32", "float64", "string", "wstring"))
FIELD_LINE = re.compile(r"^([A-Za-z0-9_/]+(?:\[[^\]]*\])?)\s+([A-Za-z_][A-Za-z0-9_]*)$")

# MAVLink messages that steer, configure or arm, and the COMMAND_* ids that
# only ask for data (allowed from anyone).
WRITE_MESSAGES = frozenset((
    "COMMAND_LONG", "COMMAND_INT", "SET_MODE", "PARAM_SET", "PARAM_EXT_SET",
    "MANUAL_CONTROL", "RC_CHANNELS_OVERRIDE", "SET_POSITION_TARGET_LOCAL_NED",
    "SET_POSITION_TARGET_GLOBAL_INT", "SET_ATTITUDE_TARGET", "SET_ACTUATOR_CONTROL_TARGET",
    "MISSION_ITEM", "MISSION_ITEM_INT", "MISSION_COUNT", "MISSION_CLEAR_ALL",
    "MISSION_SET_CURRENT", "SAFETY_SET_ALLOWED_AREA", "SET_GPS_GLOBAL_ORIGIN",
    "SET_HOME_POSITION", "PLAY_TUNE", "PLAY_TUNE_V2"))
READ_ONLY_COMMANDS = frozenset((511, 512, 519, 520, 521, 522, 2504, 2505))  # SET_MESSAGE_INTERVAL, REQUEST_MESSAGE, protocol/capabilities/camera info


def parse_fields(text):
    """(type, name) of every field of a .msg definition, in declaration order.

    Comments and constants (lines with '=') are skipped, as PX4's generator
    skips them when it hashes.
    """
    fields = []
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or "=" in line:
            continue
        match = FIELD_LINE.match(re.sub(r"\s+", " ", line))
        if match:
            fields.append((match.group(1), match.group(2)))
    return fields


def fields_text(text, resolve):
    """PX4's get_message_fields_str_for_message_hash: "type name" lines, with
    every nested type's own fields inserted right after the field that uses it.
    `resolve(TypeName)` returns that type's .msg text."""
    out = ""
    for type_name, name in parse_fields(text):
        slash = type_name.find("/")
        if slash >= 0:
            type_name = type_name[slash + 1:]
        base = type_name.split("[", 1)[0]
        if base == "Header":
            continue                        # genmsg's is_header
        out += type_name + " " + name + "\n"
        if base not in BUILTIN_TYPES:
            out += fields_text(resolve(base), resolve)
    return out


def fnv1a_32(text):
    value = 0x811C9DC5
    for char in text:
        value = ((value ^ ord(char)) * 0x01000193) & 0xFFFFFFFF
    return value


def message_hash(text, resolve):
    """The number PX4 stores per uORB topic and returns in MessageFormatResponse."""
    return fnv1a_32(fields_text(text, resolve))


def type_name(uorb_name):
    """vehicle_odometry -> VehicleOdometry (px4_msgs file and type name)."""
    return "".join(part.capitalize() for part in uorb_name.split("_"))


def directory_resolver(directory):
    def resolve(name):
        with open(os.path.join(directory, name + ".msg"), encoding="utf-8") as handle:
            return handle.read()
    return resolve


def local_hashes(uorb_names, directory):
    """{uorb_name: hash} from the px4_msgs .msg files in `directory`; a missing
    definition maps to None."""
    resolve = directory_resolver(directory)
    result = {}
    for name in uorb_names:
        try:
            result[name] = message_hash(resolve(type_name(name)), resolve)
        except OSError:
            result[name] = None
    return result


def c_string(chars):
    """A char[N] field as rclpy delivers it (ints, bytes or str) to text."""
    if isinstance(chars, str):
        return chars.split("\0", 1)[0]
    if isinstance(chars, (bytes, bytearray)):
        data = bytes(chars)
    else:
        items = list(chars)
        if items and isinstance(items[0], str):
            return "".join(items).split("\0", 1)[0]
        data = bytes(bytearray(int(c) & 0xFF for c in items))
    return data.split(b"\0", 1)[0].decode("ascii", "replace")


def c_string_field(text, width=50):
    """Text to the int list a char[width] field accepts (NUL padded, truncated)."""
    data = text.encode("ascii", "replace")[:width - 1]
    return list(data.ljust(width, b"\0"))


def mavlink_write(msg_type, src_system, src_component, command, gcs_systems=(255,), autopilot=(1, 1)):
    """True when a MAVLink message is a write to the FC from a companion-side source."""
    if msg_type not in WRITE_MESSAGES:
        return False
    if (src_system, src_component) == tuple(autopilot) or src_system in gcs_systems:
        return False
    if msg_type in ("COMMAND_LONG", "COMMAND_INT") and command in READ_ONLY_COMMANDS:
        return False
    return True


def guard_health(writers, readers, expected_readers, hashes, hash_state, mavlink, now, firmware="uav/v1.17.0-pps"):
    """writers: {topic: [node names]} for /fmu/in topics with a disallowed publisher.
    readers: /fmu/in topics the FC (or anyone) subscribes to.
    expected_readers: the firmware's agreed input set.
    hashes: {uorb_name: {"local": int|None, "fc": int|None, "answered": bool}}.
    hash_state: "unknown" (FC not answering yet), "checking", "done", "no local definitions".
    mavlink: {"count": int, "last": dict|None, "last_mono": float|None}."""
    sections = {}
    writer_count = sum(len(nodes) for nodes in writers.values())
    sections["Writers"] = check(
        writer_count == 0,
        "No publisher on any /fmu/in topic" if writer_count == 0 else
        "%d publisher(s) on /fmu/in: %s" % (writer_count, "; ".join(
            "%s <- %s" % (topic, ", ".join(nodes)) for topic, nodes in sorted(writers.items()))),
        publisher_count=writer_count)
    unexpected = sorted(set(readers) - set(expected_readers))
    sections["Firmware inputs"] = check(
        not unexpected,
        "FC input topics: %s" % (", ".join(sorted(readers)) or "none") if not unexpected else
        "%d /fmu/in topic(s) beyond the agreed set (old firmware?): %s" % (
            len(unexpected), ", ".join(unexpected[:6]) + (" ..." if len(unexpected) > 6 else "")),
        reader_count=len(readers), expected=", ".join(sorted(expected_readers)), unexpected_count=len(unexpected))
    total = len(hashes)
    matched = sum(1 for h in hashes.values() if h.get("answered") and h.get("fc") is not None and h.get("fc") == h.get("local"))
    answered = sum(1 for h in hashes.values() if h.get("answered"))
    if hash_state == "done":
        ok = matched == total
        text = ("%d/%d px4_msgs definitions match the FC" % (matched, total) if ok else
                "%d/%d px4_msgs definitions match the FC; %d differ or are unknown to it" % (matched, total, total - matched))
    elif hash_state == "no local definitions":
        ok, text = False, "unknown: px4_msgs definitions not found on this host"
    elif hash_state == "checking":
        ok, text = False, "checking: %d/%d answered" % (answered, total)
    else:
        ok, text = False, "unknown: waiting for the FC's XRCE client"
    values = {"state": hash_state, "matched": matched, "answered": answered, "total": total}
    for name, h in sorted(hashes.items()):
        if not h.get("answered"):
            verdict = "unanswered"
        elif h.get("fc") is None:
            verdict = "FC does not know this topic"
        elif h.get("local") is None:
            verdict = "no local definition"
        elif h["fc"] == h["local"]:
            verdict = "match"
        else:
            verdict = "MISMATCH local %u fc %u" % (h["local"], h["fc"])
        values["topic/" + name] = verdict
    sections["Message hashes"] = check(ok, text, **values)
    count = int(mavlink.get("count", 0))
    last = mavlink.get("last") or {}
    sections["MAVLink writes"] = check(
        count == 0,
        "No write-class MAVLink from a companion source" if count == 0 else
        "%d write-class MAVLink message(s) from companion sources; last %s%s from %s/%s %.0f s ago" % (
            count, last.get("type"), " cmd %s" % last.get("command") if last.get("command") is not None else "",
            last.get("system"), last.get("component"), now - mavlink.get("last_mono", now)),
        count=count, scope="only complete when the router forwards all traffic to this node (SnifferSysid)")
    violated = writer_count > 0 or count > 0
    message = ("Write path to the FC: " + ("; ".join(
        s for s, bad in (("publisher on /fmu/in", writer_count > 0), ("companion MAVLink writes", count > 0)) if bad))
               if violated else
               "No write path to the FC" + ("" if not unexpected else "; firmware still accepts %d input topic(s)" % len(unexpected))
               + ("" if hash_state == "done" and matched == total else "; message hashes " + (text.split(":", 1)[0] if ":" in text else "differ")))
    result = grouped(not violated, message, sections)
    result.values["Interface/firmware"] = firmware
    result.values["Interface/scope"] = ("ERROR = a way to steer the FC exists on this host (the row's connection is the "
                                        "absence of any write path); other faults are WARN")
    return result
