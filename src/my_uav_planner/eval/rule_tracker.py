"""Offline rule state machine.

Feed it a time series of (t, own_pos, own_vel, opp_pos, opp_vel, ...) and it
tells you which terminal condition *would* fire. This is the harness you use to
debug strategy logic before the real simulator is available, and to replay a
logged match after it is.

It is intentionally a plain Python class -- no ROS, no simulator -- so the same
code backs both the offline evaluator and (later) the online planner's safety
checks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from my_uav_planner import rules as R
from my_uav_planner.rules import Vec3


class Outcome(Enum):
    ONGOING = "ongoing"
    SCAN_WIN = "scan_win"          # we reported the correct code first
    SCAN_LOSS = "scan_loss"        # opponent reported first
    COLLISION_LOSS = "collision_loss"
    COLLISION_WIN = "collision_win"
    FLIGHT_LOSS = "flight_loss"    # fled the envelope (floor / low altitude)
    TIMEOUT_WIN = "timeout_win"
    TIMEOUT_LOSS = "timeout_loss"
    TAKEOFF_FAIL = "takeoff_fail"  # never climbed within the grace period


@dataclass
class TrackerConfig:
    """Knobs kept separate from rules.py because they are tuning, not rulebook."""

    #: How often to sample the state for the low-altitude timer, seconds.
    sample_dt: float = 0.02
    #: Velocity look-back used for the collision projection window, seconds.
    collision_window: float = 0.5


@dataclass
class RuleTracker:
    """Tracks our side only; the opponent is scored symmetrically by a second
    tracker seeded with ``Side.other`` if you want both verdicts.
    """

    side: R.Side
    config: TrackerConfig = field(default_factory=TrackerConfig)

    # Internal state, all in simulated seconds unless noted.
    _t: float = 0.0
    _ever_climbed: bool = False
    _low_since: float | None = None
    _swept_angle: float = 0.0
    _prev_pos: Vec3 | None = None
    _collisions: int = 0
    _finished: Outcome | None = None

    # -- inputs ------------------------------------------------------------ #

    def step(
        self,
        t: float,
        own_pos: Vec3,
        own_vel: Vec3,
        opp_pos: Vec3,
        opp_vel: Vec3,
    ) -> Outcome:
        """Advance the tracker to simulated time ``t`` and return the outcome."""
        if self._finished is not None:
            return self._finished

        self._t = t

        # 1. Takeoff grace: must first reach 1.2m within 15s.
        if not self._ever_climbed:
            if R.is_airborne(own_pos.z):
                self._ever_climbed = True
            elif t > R.TAKEOFF_GRACE_PERIOD:
                return self._finish(Outcome.TAKEOFF_FAIL)

        # 2. Floor contact and low-altitude timer (only armed once airborne).
        if self._ever_climbed:
            if R.floor_contact(own_pos):
                return self._finish(Outcome.FLIGHT_LOSS)
            if not R.is_airborne(own_pos.z):
                if self._low_since is None:
                    self._low_since = t
                elif t - self._low_since > R.LOW_ALTITUDE_DURATION:
                    return self._finish(Outcome.FLIGHT_LOSS)
            else:
                self._low_since = None

        # 3. Lap sweep about the pillar axis.
        if self._prev_pos is not None:
            self._swept_angle += self._wrapped_angle_delta(self._prev_pos, own_pos)

        # 4. Collisions against walls / ceiling / pillar.
        if R.wall_contact(own_pos) or R.pillar_box().overlaps(
            R.CollisionBox(own_pos, R.UAV_BODY_HALF_EXTENTS)
        ):
            self._collisions += 1

        # 5. Timeout.
        if t >= R.ROUND_DURATION:
            return self._finish(Outcome.TIMEOUT_WIN)

        self._prev_pos = own_pos
        return Outcome.ONGOING

    # -- derived ----------------------------------------------------------- #

    @property
    def clockwise_laps(self) -> int:
        return R.laps_from_unwrapped_angle(self._swept_angle)

    @property
    def net_laps(self) -> int:
        return R.net_laps(self.clockwise_laps, self._collisions)

    @property
    def elapsed(self) -> float:
        return self._t

    def report_scan(self, correct: bool, success: bool) -> Outcome | None:
        """Record the result of an upload. Returns a terminal outcome or None.

        Only the *first* correct report ends the round, so a wrong report is a
        no-op here (the rules do not penalize wrong content).
        """
        if not success or not correct:
            return None
        return self._finish(Outcome.SCAN_WIN)

    # -- helpers ----------------------------------------------------------- #

    def _wrapped_angle_delta(self, prev: Vec3, curr: Vec3) -> float:
        """Shortest signed change in the polar angle about the pillar axis."""
        a0 = R.angle_about_axis(prev)
        a1 = R.angle_about_axis(curr)
        d = a1 - a0
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def _finish(self, outcome: Outcome) -> Outcome:
        self._finished = outcome
        return outcome
