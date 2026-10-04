from types import SimpleNamespace
import pytest
from uav_camera.node import rgb_bytes, raw_frame


def test_rgb_rows_discard_padding():
    msg = SimpleNamespace(encoding='rgb8', width=1, height=2, step=4,
                          data=bytes([255, 0, 0, 99, 0, 0, 255, 88]))
    assert rgb_bytes(msg) == bytes([255, 0, 0, 0, 0, 255])


def test_yuy2_black_and_white():
    msg = SimpleNamespace(encoding='yuv422_yuy2', width=2, height=1, step=4,
                          data=bytes([16, 128, 235, 128]))
    assert rgb_bytes(msg) == bytes([0, 0, 0, 255, 255, 255])


@pytest.mark.parametrize('encoding,width,height,step,data', [
    ('rgb8', 2, 1, 5, bytes(5)),
    ('rgb8', 2, 1, 6, bytes(4)),
    ('32FC1', 2, 1, 8, bytes(8)),
    ('rgb8', 0, 1, 0, b''),
])
def test_bad_image_rejected(encoding,width,height,step,data):
    with pytest.raises(ValueError):
        rgb_bytes(SimpleNamespace(encoding=encoding,width=width,height=height,step=step,data=data))


def test_rotation_180_preserves_rgb_channels():
    msg = SimpleNamespace(encoding='rgb8', width=2, height=2, step=6,
                          data=bytes([255,0,0, 0,255,0, 0,0,255, 255,255,255]))
    assert rgb_bytes(msg, 180) == bytes([255,255,255, 0,0,255, 0,255,0, 255,0,0])


def test_raw_frame_passes_yuy2_through_and_trims_stride():
    from types import SimpleNamespace
    data = bytes(range(16)) + b"\xff\xff" + bytes(range(16, 32)) + b"\xee\xee"    # 8 px wide, 2 rows, 2 B pad
    msg = SimpleNamespace(encoding="yuv422_yuy2", width=8, height=2, step=18, data=data)
    pixels, fmt = raw_frame(msg)
    assert fmt == "YUY2" and pixels == bytes(range(16)) + bytes(range(16, 32))
    tight = SimpleNamespace(encoding="rgb8", width=2, height=1, step=6, data=bytes(6))
    assert raw_frame(tight) == (bytes(6), "RGB")
