"""Protocol boundary tests use real generated MAVLink 2 wire messages."""
import math
import threading
import time

import pytest
from pymavlink.dialects.v20 import common as mav

from uav_camera.protocol import CameraProtocol


class Backend:
    def __init__(self):
        self.photos = 0
        self.record_starts = 0
        self.record_stops = 0
        self.streaming = True
        self.recording = False
        self.ready = True
        self.block = None
        self.photo_error = False
        self.record_error = False
        self.last_error = ""

    def status(self):
        return dict(ready=self.ready, streaming=self.streaming, recording=self.recording,
                    recording_started_mono=time.monotonic() - 3 if self.recording else None,
                    width=896, height=504, fps=30, bitrate=4000000,
                    image_count=self.photos, total_mib=20000, available_mib=18000,
                    last_error=self.last_error)

    def capture_photo(self):
        if self.block:
            assert self.block.wait(5)
        if self.photo_error:
            raise OSError("Disk full")
        index = self.photos
        self.photos += 1
        return dict(path=f"/media/{index}.jpg", index=index, success=True,
                    time_boot_ms=1234, time_utc_us=1700000000123456)

    def start_recording(self):
        if self.record_error:
            raise OSError("Encoder unavailable")
        self.recording = True
        self.record_starts += 1
        return "/media/video.mkv"

    def stop_recording(self):
        self.recording = False
        self.record_stops += 1
        return "/media/video.mkv"

    def set_streaming(self, enabled):
        self.streaming = enabled


def command(cmd, *params, confirmation=0, seq=10, target=100, system=1, source=(255, 190)):
    wire = mav.MAVLink(None, srcSystem=source[0], srcComponent=source[1])
    wire.seq = seq
    msg = mav.MAVLink_command_long_message(
        system, target, cmd, confirmation, *(list(params) + [0.0] * (7 - len(params))))
    return mav.MAVLink(None).parse_char(msg.pack(wire))


@pytest.fixture
def camera():
    backend = Backend()
    protocol = CameraProtocol(backend, {})
    sent = []
    protocol._send = sent.append
    yield protocol, backend, sent
    if backend.block:
        backend.block.set()
    protocol.close()


def settle(protocol):
    deadline = time.monotonic() + 2
    while protocol._pending and time.monotonic() < deadline:
        protocol.tick()
        time.sleep(0.005)
    assert protocol._pending is None


def messages(sent, kind):
    return [msg for msg in sent if msg.get_type() == kind]


def acks(sent):
    return messages(sent, "COMMAND_ACK")


def test_filters_targets_and_never_handles_flight_commands(camera):
    p, b, sent = camera
    p.handle_message(command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, target=0))
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, target=101))
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, system=2))
    assert not sent and b.photos == 0
    p.handle_message(command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1))
    assert acks(sent)[-1].result == mav.MAV_RESULT_UNSUPPORTED
    assert b.photos == 0


def test_broadcast_camera_request_and_camera_heartbeat(camera):
    p, _, sent = camera
    p.handle_message(command(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, target=0))
    p.tick()
    assert acks(sent)[-1].result == mav.MAV_RESULT_ACCEPTED
    assert acks(sent)[-1].target_system == 255
    assert acks(sent)[-1].target_component == 190
    info = messages(sent, "CAMERA_INFORMATION")[-1]
    assert info.resolution_h == 896 and info.resolution_v == 504
    assert info.flags & mav.CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE
    assert not info.flags & mav.CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM
    assert messages(sent, "HEARTBEAT")[-1].type == mav.MAV_TYPE_CAMERA


@pytest.mark.parametrize("msg_id", [mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, mav.MAVLINK_MSG_ID_CAMERA_SETTINGS,
                                    mav.MAVLINK_MSG_ID_STORAGE_INFORMATION, mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS,
                                    mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, mav.MAVLINK_MSG_ID_VIDEO_STREAM_STATUS])
def test_request_message_retries_regenerate_wire_encodable_response(camera, msg_id):
    p, _, sent = camera
    for confirmation in (0, 1):
        p.handle_message(command(mav.MAV_CMD_REQUEST_MESSAGE, msg_id, confirmation=confirmation))
    assert [m.result for m in acks(sent)] == [mav.MAV_RESULT_ACCEPTED] * 2
    assert len([m for m in sent if m.get_msgId() == msg_id]) == 2
    for m in sent:
        decoded = mav.MAVLink(None).parse_char(m.pack(p._mav))
        assert decoded.get_type() == m.get_type()
    if msg_id == mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION:
        info = sent[-1]
        assert info.stream_id == 1 and info.count == 1
        assert info.encoding == mav.VIDEO_STREAM_ENCODING_H265
        assert info.uri == "rtsp://192.168.144.1:8554/rgb"


@pytest.mark.parametrize("cmd,response", [(mav.MAV_CMD_REQUEST_CAMERA_INFORMATION, "CAMERA_INFORMATION"),
                                            (mav.MAV_CMD_REQUEST_CAMERA_SETTINGS, "CAMERA_SETTINGS"),
                                            (mav.MAV_CMD_REQUEST_STORAGE_INFORMATION, "STORAGE_INFORMATION"),
                                            (mav.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, "CAMERA_CAPTURE_STATUS"),
                                            (mav.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION, "VIDEO_STREAM_INFORMATION"),
                                            (mav.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS, "VIDEO_STREAM_STATUS")])
def test_legacy_requests(camera, cmd, response):
    p, _, sent = camera
    p.handle_message(command(cmd, 1))
    assert acks(sent)[-1].result == mav.MAV_RESULT_ACCEPTED
    assert messages(sent, response)


def test_bad_stream_id_denied(camera):
    p, _, sent = camera
    p.handle_message(command(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, 2))
    assert acks(sent)[-1].result == mav.MAV_RESULT_DENIED
    assert not messages(sent, "VIDEO_STREAM_INFORMATION")


def test_photo_completion_ack_deduplication_and_no_fabricated_pose(camera):
    p, b, sent = camera
    b.block = threading.Event()
    cmd = command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 17)
    p.handle_message(cmd)
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 17, confirmation=1, seq=11))
    p.tick()
    assert all(m.result == mav.MAV_RESULT_IN_PROGRESS for m in acks(sent))
    assert not messages(sent, "CAMERA_IMAGE_CAPTURED")
    assert messages(sent, "HEARTBEAT")  # Worker cannot block heartbeat processing.
    p.handle_message(command(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_CAMERA_INFORMATION, seq=12))
    assert messages(sent, "CAMERA_INFORMATION")  # Nor unrelated command responses.
    b.block.set()
    settle(p)
    assert b.photos == 1
    assert acks(sent)[-1].result == mav.MAV_RESULT_ACCEPTED
    event = messages(sent, "CAMERA_IMAGE_CAPTURED")[-1]
    assert event.lat == event.lon == event.alt == event.relative_alt == 0x7FFFFFFF
    assert all(math.isnan(value) for value in event.q)
    assert event.time_utc == 1700000000123456 and event.camera_id == 0 and event.image_index == 0
    # Confirmation retries and the protocol's explicit single-photo sequence ID
    # both avoid duplicate files, even if the sender resets confirmation to 0.
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 17, confirmation=2, seq=13))
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 17, seq=14))
    settle(p)
    assert b.photos == 1
    # A new click with a distinct ID is a new photograph.
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 18, seq=15))
    settle(p)
    assert b.photos == 2


def test_photo_event_replay(camera):
    p, _, sent = camera
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 1))
    settle(p)
    sent.clear()
    p.handle_message(command(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED, 0))
    assert acks(sent)[-1].result == mav.MAV_RESULT_ACCEPTED
    assert messages(sent, "CAMERA_IMAGE_CAPTURED")[-1].image_index == 0
    p.handle_message(command(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED, 999))
    assert acks(sent)[-1].result == mav.MAV_RESULT_DENIED


def test_photo_error_and_stale_frame_are_not_success(camera):
    p, b, sent = camera
    b.ready = False
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 1))
    assert acks(sent)[-1].result == mav.MAV_RESULT_TEMPORARILY_REJECTED
    assert b.photos == 0
    b.ready = True
    b.photo_error = True
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 2, seq=11))
    settle(p)
    assert acks(sent)[-1].result == mav.MAV_RESULT_FAILED
    assert not messages(sent, "CAMERA_IMAGE_CAPTURED")


def test_finite_interval_capture_and_indefinite_stop(camera):
    p, b, sent = camera
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 1, 3, 0))
    settle(p)
    assert b.photos == 1 and p._sequence["remaining"] == 2
    for expected in (2, 3):
        p.tick(p._sequence["next"])
        settle(p)
        assert b.photos == expected
    assert p._sequence is None
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 1, 0, 0, seq=11))
    settle(p)
    assert p._sequence is not None
    p.handle_message(command(mav.MAV_CMD_IMAGE_STOP_CAPTURE, 0, seq=12))
    assert p._sequence is None
    p.tick(time.monotonic() + 100)
    assert b.photos == 4
    assert acks(sent)[-1].result == mav.MAV_RESULT_ACCEPTED


def test_record_commands_idempotent_and_streaming_independent(camera):
    p, b, sent = camera
    p.handle_message(command(mav.MAV_CMD_VIDEO_START_CAPTURE, 1, 1))
    settle(p)
    assert b.recording and b.record_starts == 1
    p.handle_message(command(mav.MAV_CMD_VIDEO_START_CAPTURE, 1, 1, seq=11))
    settle(p)
    assert b.record_starts == 1
    p.handle_message(command(mav.MAV_CMD_VIDEO_STOP_STREAMING, 1, seq=12))
    settle(p)
    assert not b.streaming and b.recording
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 1, seq=13))
    settle(p)
    assert b.photos == 1 and b.recording
    p.handle_message(command(mav.MAV_CMD_VIDEO_STOP_CAPTURE, 1, seq=14))
    settle(p)
    assert not b.recording and b.record_stops == 1
    p.handle_message(command(mav.MAV_CMD_VIDEO_STOP_CAPTURE, 1, seq=15))
    assert b.record_stops == 1 and acks(sent)[-1].result == mav.MAV_RESULT_ACCEPTED


def test_record_error(camera):
    p, b, sent = camera
    b.record_error = True
    p.handle_message(command(mav.MAV_CMD_VIDEO_START_CAPTURE, 0, 1))
    settle(p)
    assert acks(sent)[-1].result == mav.MAV_RESULT_FAILED
    assert not b.recording


@pytest.mark.parametrize("params", [(0, 0, 0, 0), (0, -1, 1, 0), (0, 1, 1.5, 0),
                                     (0, math.nan, 1, 0), (0, 1, 2, 5), (0, 0, 1, math.inf)])
def test_invalid_capture_parameters(camera, params):
    p, b, sent = camera
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, *params))
    assert acks(sent)[-1].result == mav.MAV_RESULT_DENIED
    assert b.photos == 0


def test_configuration_and_unavailable_stream(camera):
    p, b, _ = camera
    b.ready = False
    assert not p._stream_information().flags & mav.VIDEO_STREAM_STATUS_FLAGS_RUNNING
    other = CameraProtocol(b, {"mavlink_endpoint": "tcp:localhost:7777", "codec": "h264",
                              "system_id": 3, "component_id": 101})
    assert other.host == "localhost" and other.port == 7777
    assert other.encoding == mav.VIDEO_STREAM_ENCODING_H264
    other.close()
    with pytest.raises(ValueError):
        CameraProtocol(b, {"mavlink_endpoint": "udp:localhost:7777"})


def test_wire_send_failure_keeps_saved_photo_success(camera):
    p, b, sent = camera
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 99))
    p._pending[0].result(timeout=1)
    # Simulate the peer disappearing between saving the file and sending its ACK.
    class BrokenSocket:
        def sendall(self, _):
            raise ConnectionError("peer closed")
        def close(self):
            pass
    p._socket = BrokenSocket()
    p._send = CameraProtocol._send.__get__(p)
    p.tick()
    assert b.photos == 1 and p._socket is None
    p._send = sent.append
    p.handle_message(command(mav.MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 99, confirmation=1, seq=11))
    assert acks(sent)[-1].result == mav.MAV_RESULT_ACCEPTED
    assert len(p._images) == 1


def test_async_media_error_is_reported_to_ground_station_once_per_change(camera):
    p, b, sent = camera
    # An encoder/disk error happens after a recording command already succeeded.
    b.last_error = "recording failed: No space left on device with a long filesystem path"
    p.tick()
    errors = messages(sent, "STATUSTEXT")
    assert len(errors) == 1
    assert errors[0].severity == mav.MAV_SEVERITY_WARNING
    assert errors[0].text.startswith("D555: recording failed: No space")
    assert len(errors[0].text.encode()) <= 50
    p.tick()
    assert len(messages(sent, "STATUSTEXT")) == 1
    b.last_error = "media pipeline failed: NVENC unavailable"
    p.tick()
    assert len(messages(sent, "STATUSTEXT")) == 2
    b.last_error = ""
    p.tick()
    assert len(messages(sent, "STATUSTEXT")) == 2
    b.last_error = "media pipeline failed: NVENC unavailable"
    p.tick()
    assert len(messages(sent, "STATUSTEXT")) == 3


def autopilot(msg, source=(1, 1)):
    wire = mav.MAVLink(None, srcSystem=source[0], srcComponent=source[1])
    return mav.MAVLink(None).parse_char(msg.pack(wire))


def rc_frame(source=(1, 1), **channels):
    values = [1499] * 4 + [999] + [1499] * 4 + [1049] * 4 + [1499, 1499, 1999, 998, 998]
    for key, raw in channels.items():
        values[int(key[2:]) - 1] = raw
    return autopilot(mav.MAVLink_rc_channels_message(0, 18, *values, 255), source)


def sys_status(rc_present=True):
    bit = mav.MAV_SYS_STATUS_SENSOR_RC_RECEIVER if rc_present else 0
    return autopilot(mav.MAVLink_sys_status_message(bit, bit, bit, 0, 0, -1, -1, 0, 0, 0, 0, 0, 0))


@pytest.fixture
def rc_camera():
    backend = Backend()
    protocol = CameraProtocol(backend, {"rc_photo_channel": 12, "rc_video_channel": 13})
    sent = []
    protocol._send = sent.append
    yield protocol, backend, sent
    protocol._socket = None
    protocol.close()


def press(protocol, **channel):
    for msg in (rc_frame(), rc_frame(), rc_frame(**channel), rc_frame(**channel), rc_frame(), rc_frame()):
        protocol.handle_message(msg)
    settle(protocol)


def test_rc_buttons_take_a_photo_and_toggle_recording_without_acks(rc_camera):
    protocol, backend, sent = rc_camera
    protocol.handle_message(sys_status())
    press(protocol, ch12=1949)
    assert backend.photos == 1
    assert any(m.get_type() == "CAMERA_IMAGE_CAPTURED" for m in sent)
    press(protocol, ch13=1949)
    assert backend.recording and backend.record_starts == 1
    press(protocol, ch13=1949)
    assert not backend.recording and backend.record_stops == 1
    assert not any(m.get_type() == "COMMAND_ACK" for m in sent)   # nobody asked
    assert protocol.rc_status()["presses"] == {"photo": 1, "video": 2}


def test_rc_needs_the_autopilot_and_a_present_receiver(rc_camera):
    protocol, backend, _ = rc_camera
    press(protocol, ch12=1949)                       # no SYS_STATUS yet
    protocol.handle_message(sys_status())
    for msg in (rc_frame((255, 190)), rc_frame((255, 190), ch12=1949), rc_frame((255, 190), ch12=1949)):
        protocol.handle_message(msg)                 # a GCS's RC_CHANNELS is not the transmitter
    settle(protocol)
    protocol.handle_message(sys_status(False))
    press(protocol, ch12=1949)
    assert backend.photos == 0


def test_rc_rate_is_requested_when_short_and_not_repeated_within_10s(rc_camera):
    protocol, _, sent = rc_camera
    protocol._socket = object()
    protocol.tick(100.0)
    protocol.tick(105.0)
    requests = [m for m in sent if m.get_type() == "COMMAND_LONG" and m.command == mav.MAV_CMD_SET_MESSAGE_INTERVAL]
    assert len(requests) == 1
    request = requests[0]
    assert (request.target_system, request.target_component) == (1, 1)
    assert (request.param1, request.param2) == (mav.MAVLINK_MSG_ID_RC_CHANNELS, 50000)


def test_rc_disabled_by_default(camera):
    protocol, backend, _ = camera
    protocol.handle_message(sys_status())
    press(protocol, ch12=1949)
    assert protocol.rc_status() is None and backend.photos == 0
