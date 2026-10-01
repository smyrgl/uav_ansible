"""When to start and stop a flight bag. Pure logic, no ROS."""
from dataclasses import dataclass

IDLE, RECORDING, POST_ROLL = "idle", "recording", "post_roll"


@dataclass(frozen=True)
class Decision:
    action: str | None      # "start", "stop" or None
    state: str
    reason: str = ""


class FlightPolicy:
    """Start when the vehicle arms (or on a manual request). After disarm keep
    recording for post_roll_sec so the landing's aftermath is in the bag; stop
    sooner once the vehicle is safed (safety engaged) after disarm, after
    safed_grace_sec. A re-arm during post-roll continues the same bag. A manual
    request stops as soon as it is withdrawn, unless the vehicle is armed."""

    def __init__(self, post_roll_sec=30.0, safed_grace_sec=5.0):
        self.post_roll_sec = float(post_roll_sec)
        self.safed_grace_sec = float(safed_grace_sec)
        self.state = IDLE
        self.armed = False
        self.safety_off = None       # None until the FC has reported it
        self.manual = False
        self.disarmed_at = None
        self.safed_at = None
        self.armed_during = False    # did the vehicle arm at any point in this recording?

    def update(self, now, armed=None, safety_off=None, manual=None):
        if armed is not None:
            self.armed = bool(armed)
        if safety_off is not None:
            self.safety_off = bool(safety_off)
        if manual is not None:
            self.manual = bool(manual)
        wanted = self.armed or self.manual
        if self.state == IDLE:
            if wanted:
                self.state = RECORDING
                self.disarmed_at = self.safed_at = None
                self.armed_during = self.armed
                return Decision("start", self.state, "armed" if self.armed else "manual request")
            return Decision(None, self.state)
        if self.state == RECORDING:
            self.armed_during = self.armed_during or self.armed
            if wanted:
                return Decision(None, self.state)
            if not self.armed_during:
                self.state = IDLE
                return Decision("stop", self.state, "manual request withdrawn")
            self.state = POST_ROLL
            self.disarmed_at = now
            self.safed_at = now if self.safety_off is False else None
            return Decision(None, self.state, "disarmed; post-roll")
        # POST_ROLL
        if wanted:
            self.state = RECORDING
            self.disarmed_at = self.safed_at = None
            return Decision(None, self.state, "re-armed; continuing the same bag")
        if self.safety_off is False and self.safed_at is None:
            self.safed_at = now
        if self.safed_at is not None and now - self.safed_at >= self.safed_grace_sec:
            self.state = IDLE
            return Decision("stop", self.state, "vehicle safed")
        if now - self.disarmed_at >= self.post_roll_sec:
            self.state = IDLE
            return Decision("stop", self.state, "post-roll elapsed")
        return Decision(None, self.state)

    def tick(self, now):
        return self.update(now)
