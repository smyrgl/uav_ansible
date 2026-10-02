"""Transmitter buttons for the camera, read from the autopilot's RC_CHANNELS.

PX4 forwards every RC channel in RC_CHANNELS (raw microseconds) but marks
neither loss nor an SBUS failsafe there: during a failsafe the IO chip passes
the receiver's failsafe positions straight through, and SBUS RSSI stays at
maximum. A press therefore counts only

- while the autopilot's SYS_STATUS lists the RC receiver as present (PX4
  drops it from the present mask whenever manual control is invalid);
- once the new position has held for ``debounce_frames`` consecutive frames;
- if that frame did not also move ``jump_channels`` or more other channels by
  over ``jump_us``: a jump to failsafe positions, or back, looks like that.

After a gap, a loss or such a jump, the current positions become the new
baseline without producing a press.
"""
import math

BUTTON, TOGGLE = "button", "toggle"


def _valid(raw):
    # PX4 sends UINT16_MAX for channels beyond the receiver's channel count.
    return raw is not None and 0 < raw < 65535


class RcButtons:
    """``buttons`` maps a name to (channel, mode), channels numbered from 1.
    BUTTON acts when the channel goes high (a momentary button's press);
    TOGGLE acts on every change (a latching button, one action per flip)."""

    def __init__(self, buttons, threshold_us=1500, debounce_frames=2, stale_s=1.0, jump_us=150, jump_channels=3):
        self.buttons = {}
        for name, (channel, mode) in buttons.items():
            if mode not in (BUTTON, TOGGLE):
                raise ValueError(f"RC {name}: mode must be {BUTTON!r} or {TOGGLE!r}")
            if not 1 <= int(channel) <= 18:
                raise ValueError(f"RC {name}: channel must be 1-18")
            self.buttons[name] = (int(channel), mode)
        self.threshold = float(threshold_us)
        self.debounce = max(1, int(debounce_frames))
        self.stale_s = float(stale_s)
        self.jump_us = float(jump_us)
        self.jump_channels = int(jump_channels)
        self.present = False
        self.presses = {name: 0 for name in self.buttons}
        self.suppressed = 0
        self._last_frame = -math.inf
        self._rebaseline()

    def _rebaseline(self):
        self._previous = None
        self._state = {}        # name -> accepted level
        self._candidate = {}    # name -> (level, consecutive frames)

    def set_present(self, present):
        present = bool(present)
        if not present:
            self._rebaseline()
        self.present = present

    def update(self, values, now):
        """``values``: raw microseconds, index 0 = channel 1. Returns the names pressed."""
        gap = now - self._last_frame > self.stale_s
        self._last_frame = now
        if gap or not self.present:
            self._rebaseline()
            if not self.present:
                return []
        previous, self._previous = self._previous, list(values)
        levels = {}
        for name, (channel, _) in self.buttons.items():
            raw = values[channel - 1] if channel <= len(values) else None
            if _valid(raw):
                levels[name] = raw > self.threshold
            else:
                self._state.pop(name, None)
                self._candidate.pop(name, None)
        if previous is not None:
            own = {channel for channel, _ in self.buttons.values()}
            jumped = sum(1 for index, (a, b) in enumerate(zip(previous, values))
                         if index + 1 not in own and _valid(a) and _valid(b) and abs(a - b) > self.jump_us)
            if jumped >= self.jump_channels:
                self.suppressed += 1
                self._state, self._candidate = dict(levels), {}
                return []
        pressed = []
        for name, level in levels.items():
            if name not in self._state:
                self._state[name] = level
                continue
            if level == self._state[name]:
                self._candidate.pop(name, None)
                continue
            seen, count = self._candidate.get(name, (level, 0))
            count = count + 1 if seen == level else 1
            if count < self.debounce:
                self._candidate[name] = (level, count)
                continue
            self._candidate.pop(name, None)
            self._state[name] = level
            if level or self.buttons[name][1] == TOGGLE:
                self.presses[name] += 1
                pressed.append(name)
        return pressed
