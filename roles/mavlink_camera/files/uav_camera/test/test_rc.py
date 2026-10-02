"""Transmitter button logic: debounce, RC presence, failsafe-jump guard, re-baselining."""
import pytest

from uav_camera.rc import BUTTON, TOGGLE, RcButtons

# As read from this transmitter at rest (channels 1-18).
REST = [1499] * 4 + [999, 1499, 1499, 1499, 1499, 1049, 1049, 1049, 1049, 1499, 1499, 1999, 998, 998]


def frame(**channels):
    values = list(REST)
    for key, raw in channels.items():
        values[int(key[2:]) - 1] = raw
    return values


def feed(rc, frames, t0=0.0, dt=0.05):
    pressed = []
    for i, values in enumerate(frames):
        pressed += rc.update(values, t0 + i * dt)
    return pressed


def make(mode=BUTTON):
    rc = RcButtons({"photo": (12, mode), "video": (13, mode)})
    rc.set_present(True)
    return rc


def test_momentary_press_counts_once_after_debounce():
    rc = make()
    up = frame(ch12=1949)
    assert feed(rc, [REST, REST, up]) == []                # one frame high: not yet
    assert rc.update(up, 0.15) == ["photo"]                # held a second frame: press
    assert feed(rc, [up, REST, REST], t0=0.2) == []        # holding, then release: nothing
    assert rc.presses == {"photo": 1, "video": 0}


def test_single_frame_glitch_is_ignored():
    assert feed(make(), [REST, REST, frame(ch12=1949), REST, REST]) == []


def test_toggle_mode_acts_on_every_flip():
    on = frame(ch13=1949)
    assert feed(make(TOGGLE), [REST, REST, on, on, on, REST, REST]) == ["video", "video"]


def test_nothing_counts_without_the_rc_receiver_and_its_return_is_a_baseline():
    rc = make()
    rc.set_present(False)
    held = frame(ch12=1949)
    assert feed(rc, [REST, held, held]) == []
    rc.set_present(True)
    assert feed(rc, [held, held, held], t0=0.5) == []      # already down when the RC came back


def test_gap_rebaselines():
    rc = make()
    feed(rc, [REST, REST])
    held = frame(ch12=1949)
    assert feed(rc, [held, held], t0=5.0) == []


def test_failsafe_jump_is_not_a_press_and_neither_is_the_jump_back():
    rc = make()
    failsafe = frame(ch1=1049, ch2=1049, ch3=1049, ch12=1949)
    assert feed(rc, [REST, REST, failsafe, failsafe, failsafe]) == []
    assert rc.suppressed == 1
    assert feed(rc, [REST, REST], t0=0.25) == []
    assert rc.suppressed == 2
    up = frame(ch12=1949)
    assert feed(rc, [up, up], t0=0.35) == ["photo"]        # a real press afterwards still works


def test_stick_motion_does_not_block_a_press():
    a, b = frame(ch1=1600, ch2=1400, ch12=1949), frame(ch1=1700, ch2=1300, ch12=1949)
    assert feed(make(), [REST, frame(ch1=1550, ch2=1450), a, b]) == ["photo"]


def test_absent_channels_are_ignored():
    short = REST[:11] + [65535] * 7                        # an 11-channel receiver
    assert feed(make(), [short, short, short]) == []


def test_bad_configuration():
    with pytest.raises(ValueError):
        RcButtons({"photo": (19, BUTTON)})
    with pytest.raises(ValueError):
        RcButtons({"photo": (12, "latch")})
