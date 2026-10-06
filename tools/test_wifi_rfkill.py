"""Tests for roles/wifi_rfkill/files/uav-wifi-rfkill.py: the HEARTBEAT parser against
frames built by pymavlink (v1 and v2, signed, truncated), and the block/unblock policy.

Run: python3 -m pytest tools/test_wifi_rfkill.py
"""
import importlib.util
import pathlib

import pytest

mavlink = pytest.importorskip("pymavlink.dialects.v20.common")
mavlink1 = pytest.importorskip("pymavlink.dialects.v10.common")

SRC = pathlib.Path(__file__).resolve().parents[1] / "roles" / "wifi_rfkill" / "files" / "uav-wifi-rfkill.py"
spec = importlib.util.spec_from_file_location("uav_wifi_rfkill", SRC)
agent = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agent)


def heartbeat(armed, sysid=1, compid=1, v1=False, signed=False):
    mod = mavlink1 if v1 else mavlink
    mav = mod.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    if signed:
        mav.signing.secret_key = bytes(32)
        mav.signing.link_id = 1
        mav.signing.timestamp = 1
        mav.signing.sign_outgoing = True
    base_mode = agent.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
    msg = mod.MAVLink_heartbeat_message(mod.MAV_TYPE_QUADROTOR, mod.MAV_AUTOPILOT_PX4, base_mode, 0, mod.MAV_STATE_ACTIVE, 3)
    return msg.pack(mav)


def other_message(sysid=1, compid=1):
    mav = mavlink.MAVLink(None, srcSystem=sysid, srcComponent=compid)
    msg = mavlink.MAVLink_sys_status_message(*([0] * 13))
    return msg.pack(mav)


def test_parses_v2_and_v1_frames():
    p = agent.HeartbeatParser()
    data = heartbeat(True) + heartbeat(False, v1=True) + heartbeat(True, sysid=1, compid=100)
    assert p.feed(data) == [(1, 1, True), (1, 1, False), (1, 100, True)]
    assert p.bad == 0


def test_truncated_v2_payload_still_reads_base_mode():
    # custom_mode 0 and system_status 0 let MAVLink 2 trim trailing zeros of the payload
    mav = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    msg = mavlink.MAVLink_heartbeat_message(mavlink.MAV_TYPE_QUADROTOR, 0, agent.MAV_MODE_FLAG_SAFETY_ARMED, 0, 0, 0)
    frame = msg.pack(mav)
    assert frame[1] < agent.HEARTBEAT_LEN
    assert agent.HeartbeatParser().feed(frame) == [(1, 1, True)]


def test_split_stream_and_garbage():
    p = agent.HeartbeatParser()
    # a stray frame marker with a plausible length in the middle must not swallow the next HEARTBEAT
    stream = b"\x00\x11garbage" + other_message() + heartbeat(True) + b"\xfd\x05" + heartbeat(False) + other_message() + heartbeat(True)
    got = []
    for i in range(0, len(stream), 5):          # arbitrary chunking
        got += p.feed(stream[i:i + 5])
    assert got == [(1, 1, True), (1, 1, False), (1, 1, True)]
    assert p.bad >= 1


def test_bad_checksum_is_dropped():
    p = agent.HeartbeatParser()
    frame = bytearray(heartbeat(True))
    frame[-1] ^= 0xFF
    assert p.feed(bytes(frame)) == []
    assert p.bad == 1
    assert p.feed(heartbeat(False)) == [(1, 1, False)]


def test_signed_v2_frame():
    assert agent.HeartbeatParser().feed(heartbeat(True, signed=True)) == [(1, 1, True)]


def test_policy_block_unblock_and_stale():
    pol = agent.Policy(unblock_delay_s=5, stale_s=10)
    assert pol.step(0.0) is None                      # untouched before any heartbeat
    pol.heartbeat(False, 1.0)
    assert pol.step(1.0) == "unblock"                 # first word from the autopilot: disarmed
    pol.heartbeat(True, 2.0)
    assert pol.step(2.0) == "block"
    pol.heartbeat(True, 3.0)
    assert pol.step(3.0) is None
    pol.heartbeat(False, 4.0)
    assert pol.step(4.0) is None                      # not yet: 5 s of disarmed first
    pol.heartbeat(True, 6.0)                          # quick re-arm keeps it blocked
    assert pol.step(6.0) is None
    pol.heartbeat(False, 7.0)
    for t in (8.0, 9.0, 10.0, 11.0):
        pol.heartbeat(False, t)
        assert pol.step(t) is None
    pol.heartbeat(False, 12.0)
    assert pol.step(12.0) == "unblock"
    pol.heartbeat(True, 13.0)
    assert pol.step(13.0) == "block"
    assert pol.step(20.0) is None                     # 7 s of silence: still within stale
    assert pol.step(23.5) == "unblock"                # >10 s without a heartbeat: open the radio


def test_policy_restart_while_armed_blocks_on_first_heartbeat():
    pol = agent.Policy(5, 10)
    assert pol.step(100.0) is None
    pol.heartbeat(True, 100.5)
    assert pol.step(100.5) == "block"
