from types import SimpleNamespace
import pytest
from uav_camera.node import rgb_bytes


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
