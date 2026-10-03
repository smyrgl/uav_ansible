#!/usr/bin/env python3
"""FAST-LIO's health: the evidence that it has lost track, the verdict the lio nodes share
(/lio/health, from the watchdog) and when to restart it.

FAST-LIO has no output that says it has diverged; two things give it away:
- its log. "No Effective Points!" is printed for every iteration of a scan whose IEKF
  update found nothing to match, and "No point, skip this scan!" for a scan without
  usable points. Either way the state is propagated on the IMU alone, and that runs away
  quadratically: on 2026-10-02, with the Avia facing an obstruction inside its blind
  zone, the lines started 0.3 s after initialisation and the pose reached -155 km.
- its odometry: a speed or a leap between consecutive poses that no X950 makes.

A short run of either is normal (a featureless moment, the first scan after start-up);
a run sustained for `sustain_s` is a divergence, and FAST-LIO does not come back from it
by itself. The verdict therefore holds DIVERGED until FAST-LIO starts over (a new epoch:
"IMU Initial Done" in its log, or its odometry restarting), and carries the onset, the
start of the run less a margin, on the host's monotonic clock, which every process on
the Jetson shares: the map rolls back what arrived after it.
"""
import json
import math
from collections import deque

OK, DEGRADED, DIVERGED = "ok", "degraded", "diverged"
LOG_EVIDENCE = {"No Effective Points": "no effective points", "No point, skip this scan": "no usable points"}
LOG_RESTART = "IMU Initial Done"


class DivergenceMonitor:
    """Evidence in, verdict out; times are monotonic seconds (stamps only for odometry)."""

    def __init__(self, sustain_s=2.0, gap_s=0.5, max_speed_mps=30.0, max_leap_m=5.0, onset_margin_s=1.0,
                 restart_gap_s=5.0):
        self.sustain_s, self.gap_s = sustain_s, gap_s
        self.max_speed, self.max_leap = max_speed_mps, max_leap_m
        self.onset_margin_s, self.restart_gap_s = onset_margin_s, restart_gap_s
        self.epoch = 0
        self.state = OK
        self.reason = ""
        self.onset = self.since = None
        self.run_start = self.run_last = None      # the current run of evidence
        self.run_kinds = set()
        self.run_peak_speed = 0.0
        self.last_pose = None                      # (stamp s, position)
        self.evidence_total = 0

    def log_line(self, now, text):
        """One line of FAST-LIO's log, read at monotonic `now`."""
        if LOG_RESTART in text:
            self.restarted(now, "FAST-LIO initialised")
            return
        for marker, kind in LOG_EVIDENCE.items():
            if marker in text:
                self._evidence(now, kind)
                return

    def pose(self, now, stamp_s, position):
        """FAST-LIO's IMU position at its stamp, received at monotonic `now`."""
        previous, self.last_pose = self.last_pose, (stamp_s, tuple(position))
        if previous is None:
            return
        dt = stamp_s - previous[0]
        if dt < -1.0 or dt > self.restart_gap_s:
            self.restarted(now, "FAST-LIO odometry started over")
            return
        if dt <= 0:
            return
        leap = math.dist(position, previous[1])
        if leap > self.max_leap or leap / dt > self.max_speed:
            self._evidence(now, "implausible motion", speed=leap / dt)

    def _evidence(self, now, kind, speed=None):
        self.evidence_total += 1
        if self.run_last is None or now - self.run_last > self.gap_s:
            self.run_start, self.run_kinds, self.run_peak_speed = now, set(), 0.0
        self.run_last = now
        self.run_kinds.add(kind)
        if speed is not None:
            self.run_peak_speed = max(self.run_peak_speed, speed)
        self.update(now)

    def _describe(self):
        kinds = sorted(self.run_kinds)
        return " and ".join(k + (" (up to %.0f m/s)" % self.run_peak_speed if k == "implausible motion" else "") for k in kinds)

    def update(self, now):
        """Re-evaluate at `now` (also called on a timer, so a run that stopped clears)."""
        if self.state == DIVERGED:
            return self.state                      # until FAST-LIO starts over
        running = self.run_last is not None and now - self.run_last <= self.gap_s
        if running and self.run_last - self.run_start >= self.sustain_s:
            self.state, self.since = DIVERGED, now
            self.onset = self.run_start - self.onset_margin_s
            self.reason = "%s for %.1f s" % (self._describe(), self.run_last - self.run_start)
        elif running:
            self.state, self.reason = DEGRADED, self._describe()
        else:
            self.state, self.reason = OK, ""
        return self.state

    def restarted(self, now, reason):
        self.epoch += 1
        self.state, self.reason = OK, reason
        self.onset = self.since = None
        self.run_start = self.run_last = None
        self.run_kinds = set()
        self.run_peak_speed = 0.0
        self.last_pose = None

    def verdict(self):
        return {"state": self.state, "epoch": self.epoch, "reason": self.reason,
                "onset_mono": self.onset, "since_mono": self.since}


class RestartPolicy:
    """When to restart FAST-LIO: diverged for `after_s`, at most once per `min_interval_s`
    and `max_per_hour` an hour. Past that it gives up (an aircraft facing a wall diverges
    again at once; restarting it all day helps nobody) until a restart from elsewhere."""

    def __init__(self, enabled=True, after_s=10.0, min_interval_s=120.0, max_per_hour=5):
        self.enabled, self.after_s = enabled, after_s
        self.min_interval_s, self.max_per_hour = min_interval_s, max_per_hour
        self.history = deque()

    def _recent(self, now):
        while self.history and now - self.history[0] > 3600.0:
            self.history.popleft()
        return len(self.history)

    def exhausted(self, now):
        return self._recent(now) >= self.max_per_hour

    def due(self, now, verdict):
        if not self.enabled or verdict["state"] != DIVERGED or verdict["since_mono"] is None:
            return False
        if now - verdict["since_mono"] < self.after_s or self.exhausted(now):
            return False
        return not self.history or now - self.history[-1] >= self.min_interval_s

    def record(self, now):
        self.history.append(now)

    def next_in(self, now, verdict):
        """Seconds until the next restart would be due, or None (not diverged, disabled, exhausted)."""
        if not self.enabled or verdict["state"] != DIVERGED or self.exhausted(now):
            return None
        wait = verdict["since_mono"] + self.after_s - now
        if self.history:
            wait = max(wait, self.history[-1] + self.min_interval_s - now)
        return max(0.0, wait)


class HealthFollower:
    """A consumer's view of /lio/health: the latest verdict and what just changed. An epoch
    counts as FAST-LIO starting over only within one watchdog session (a restarted watchdog
    starts again at epoch 0 and must not wipe anybody's map)."""

    def __init__(self):
        self.verdict = None

    @property
    def diverged(self):
        return self.verdict is not None and self.verdict["state"] == DIVERGED

    def update(self, verdict):
        """'restarted' (FAST-LIO started over: a new origin), 'diverged' (just declared), or None."""
        previous, self.verdict = self.verdict, verdict
        if previous is None:
            return "diverged" if verdict["state"] == DIVERGED else None
        same_session = previous.get("session") == verdict.get("session")
        if same_session and verdict["epoch"] != previous["epoch"]:
            return "restarted"
        if verdict["state"] == DIVERGED and (previous["state"] != DIVERGED or not same_session):
            return "diverged"
        return None


def encode(verdict, **extra):
    return json.dumps(dict(verdict, **extra), separators=(",", ":"))


def decode(text):
    """The verdict from /lio/health, or None for anything malformed."""
    try:
        verdict = json.loads(text)
        return verdict if verdict.get("state") in (OK, DEGRADED, DIVERGED) and isinstance(verdict.get("epoch"), int) else None
    except (ValueError, AttributeError):
        return None
