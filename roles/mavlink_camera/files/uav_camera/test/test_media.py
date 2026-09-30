import threading
import time

import pytest

from uav_camera.media import MediaManager, _Frame


def manager(tmp_path, **kwargs):
    return MediaManager(dict(storage_dir=str(tmp_path), photo_timeout_s=0.15, **kwargs))


def frame(media, value=1, stamp=1234):
    media.submit_frame(bytes([value] * 24), 4, 2, stamp, "rgb_optical", {"k": [1, 2, 3]})


def test_latest_frame_queue_is_bounded_and_preserves_source_stamp(tmp_path):
    media = manager(tmp_path)
    for number in range(100):
        frame(media, number, 1000 + number)
    assert media._pending is media._latest
    assert media._pending.sequence == 100
    assert media._pending.stamp_ns == 1099
    assert media._pending.rgb == bytes([99] * 24)
    assert not media.status()["ready"]  # Receiving RGB alone is not encoder readiness.


def test_photo_rejects_pre_request_frame(tmp_path):
    media = manager(tmp_path)
    frame(media)
    result = media.capture_photo()
    assert not result["success"]
    assert media.status()["image_count"] == 0
    assert "fresh" in media.status()["last_error"]


def test_photo_waits_for_next_frame_and_counts_only_saved_capture(tmp_path, monkeypatch):
    media = manager(tmp_path)
    frame(media, stamp=11)
    saved = []
    monkeypatch.setattr(media, "_write_photo", lambda path, value: saved.append(value))
    result = []
    thread = threading.Thread(target=lambda: result.append(media.capture_photo()))
    thread.start()
    time.sleep(0.02)
    frame(media, stamp=22)
    thread.join(1)
    assert result[0]["success"]
    assert saved[0].stamp_ns == 22
    assert result[0]["time_utc_us"] > 1_600_000_000_000_000
    assert media.status()["image_count"] == 1


def test_disk_failure_is_not_reported_as_success(tmp_path, monkeypatch):
    media = manager(tmp_path)
    def fail(*_args):
        raise OSError("No space left on device")
    monkeypatch.setattr(media, "_write_photo", fail)
    result = []
    thread = threading.Thread(target=lambda: result.append(media.capture_photo()))
    thread.start()
    time.sleep(0.02)
    frame(media)
    thread.join(1)
    assert not result[0]["success"]
    assert media.status()["image_count"] == 0
    assert "No space" in media.status()["last_error"]


def test_camera_info_is_snapshot_not_mutable_alias(tmp_path):
    media = manager(tmp_path)
    info = {"k": [1.0]}
    media.submit_frame(bytes(24), 4, 2, 5, "camera", info)
    info["k"][0] = 2.0
    assert media._latest.camera_info == {"k": [1.0]}


def test_invalid_frame_does_not_replace_good_frame(tmp_path):
    media = manager(tmp_path)
    frame(media)
    media.submit_frame(bytes(1), 4, 2, 0, "camera", None)
    assert media._latest.stamp_ns == 1234
    assert "byte count" in media.status()["last_error"]


def test_freshness_and_streaming_require_actual_encoded_output(tmp_path):
    media = manager(tmp_path, frame_timeout_s=0.01)
    frame(media)
    assert not media.status()["ready"]
    assert not media.status()["streaming"]
    media._last_encoded_mono = time.monotonic()
    assert media.status()["ready"]
    assert media.status()["streaming"]
    time.sleep(0.02)
    assert not media.status()["ready"]


def test_stop_unblocks_waiting_photo(tmp_path):
    media = manager(tmp_path)
    result = []
    thread = threading.Thread(target=lambda: result.append(media.capture_photo()))
    thread.start()
    time.sleep(0.02)
    media.close()
    thread.join(1)
    assert result and not result[0]["success"]


@pytest.mark.parametrize("codec", ["h264", "h265"])
def test_pipeline_uses_bounded_hardware_encode_without_b_frames(tmp_path, codec):
    media = manager(tmp_path, codec=codec)
    description = media._pipeline_description(896, 504)
    assert f"nvv4l2{codec}enc" in description
    assert "num-B-Frames=0" in description
    assert "max-buffers=2" in description
    assert "leaky-type=downstream" in description
    assert "insert-sps-pps=true" in description


def test_unsupported_codec_rejected(tmp_path):
    with pytest.raises(ValueError, match="codec"):
        manager(tmp_path, codec="vp9")


def test_stream_restart_preserves_existing_rtp_timestamp_epoch(tmp_path):
    media = manager(tmp_path)
    stream = {"base_pts": 4_000_000_000, "wait_keyframe": False}
    media._rtsp_sources[1] = stream
    media.set_streaming(False)
    media.set_streaming(True)
    assert stream["base_pts"] == 4_000_000_000
    assert stream["wait_keyframe"]


def test_encoder_watchdog_allows_startup_then_detects_no_output(tmp_path):
    media = manager(tmp_path)
    frame(media)
    now = media._latest.received_mono
    media._pipeline = object()
    media._encoder_started_mono = now - 2.999
    assert not media._encoder_stalled(now)
    media._encoder_started_mono = now - 3.0
    assert media._encoder_stalled(now)


def test_encoder_watchdog_requires_fresh_input_and_existing_pipeline(tmp_path):
    media = manager(tmp_path)
    frame(media)
    now = media._latest.received_mono
    media._encoder_started_mono = now - 10
    assert not media._encoder_stalled(now)
    media._pipeline = object()
    assert media._encoder_stalled(now)
    assert not media._encoder_stalled(now + media.frame_timeout)
    media._latest = None
    assert not media._encoder_stalled(now)


def test_encoder_watchdog_tracks_output_and_runs_while_rtsp_paused(tmp_path):
    media = manager(tmp_path)
    frame(media)
    now = media._latest.received_mono
    media._pipeline = object()
    media._encoder_started_mono = now - 10
    media.set_streaming(False)
    media._last_encoded_mono = now - 2.999
    assert not media._encoder_stalled(now)
    media._last_encoded_mono = now - 3.0
    assert media._encoder_stalled(now)
    media._last_encoded_mono = now
    assert not media._encoder_stalled(now)


def test_encoder_watchdog_grace_restarts_after_rebuild(tmp_path):
    media = manager(tmp_path)
    frame(media)
    now = media._latest.received_mono
    media._pipeline = object()
    media._last_encoded_mono = now - 10
    media._encoder_started_mono = now - 0.1
    assert not media._encoder_stalled(now)


def test_encoded_hook_gets_source_metadata_and_pts_index_is_bounded(tmp_path):
    calls = []
    media = MediaManager(dict(storage_dir=str(tmp_path)),
                         on_encoded=lambda data, keyframe, meta: calls.append((data, keyframe, meta)))
    for pts in range(300):
        media._remember_frame(pts, _Frame(b"", 4, 2, 1000 + pts, "rgb_optical", None, pts, 0.0, 5000 + pts))
    assert len(media._pts_index) == 128
    assert media._recall_frame(0) is None  # evicted, never grows with dropped frames
    media._dispatch_encoded(b"\x00\x00\x00\x01", True, 299)
    assert calls == [(b"\x00\x00\x00\x01", True, (1299, "rgb_optical", 5299))]
    assert media._recall_frame(299) is None  # consumed
    assert media.encoded_hook_calls == 1
    media._dispatch_encoded(b"x", False, 42)  # unknown PTS still delivers, with no metadata
    assert calls[-1] == (b"x", False, None)


def test_encoded_hook_failure_is_isolated_from_the_encoder(tmp_path):
    def boom(*_):
        raise RuntimeError("publisher gone")
    media = MediaManager(dict(storage_dir=str(tmp_path)), on_encoded=boom)
    media._dispatch_encoded(b"x", False, 1)  # must not raise into the GStreamer thread
    assert "publisher gone" in media.status()["last_error"]
    assert media.encoded_hook_calls == 0
