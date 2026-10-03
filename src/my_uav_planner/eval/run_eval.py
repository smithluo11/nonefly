"""Batch evaluator for planner runs.

Two modes, both stubbed until the simulator's log format is known (see README,
"Open questions"):

  * ``--synthetic``  generate simple scripted trajectories and check that the
    tracker's verdicts match hand-worked expectations. Useful as a smoke test
    of the rule layer right now.
  * ``--replay PATH``  feed a recorded match (ROS bag / CSV once we have one)
    through the tracker and print a verdict.

Run:  uv run python eval/run_eval.py --synthetic
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from eval.rule_tracker import Outcome, RuleTracker  # noqa: E402
from my_uav_planner import rules as R  # noqa: E402
from my_uav_planner.rules import Side, Vec3  # noqa: E402


def synthetic_takeoff_failure() -> Outcome:
    """Never leave the ground; the grace period should expire."""
    trk = RuleTracker(side=Side.POS)
    t = 0.0
    while t <= R.TAKEOFF_GRACE_PERIOD + 1.0:
        out = trk.step(t, Vec3(3, 0, 0), Vec3(0, 0, 0), Vec3(-3, 0, 0), Vec3(0, 0, 0))
        if out is not Outcome.ONGOING:
            return out
        t += 0.1
    return trk._finished or Outcome.ONGOING


def synthetic_hover_to_timeout() -> Outcome:
    """Climb, hover safely, and let the clock run out."""
    trk = RuleTracker(side=Side.POS)
    t = 0.0
    while t <= R.ROUND_DURATION + 1.0:
        out = trk.step(t, Vec3(3, 0, 1.5), Vec3(0, 0, 0), Vec3(-3, 0, 1.5), Vec3(0, 0, 0))
        if out is not Outcome.ONGOING:
            return out
        t += 0.5
    return Outcome.ONGOING


def synthetic_low_altitude_loss() -> Outcome:
    """Climb, then sit below 1.2m past the threshold."""
    trk = RuleTracker(side=Side.POS)
    t = 0.0
    # Establish "ever climbed".
    trk.step(t, Vec3(3, 0, 1.5), Vec3(0, 0, 0), Vec3(-3, 0, 1.5), Vec3(0, 0, 0))
    t += 0.1
    while t <= 60.0:
        out = trk.step(t, Vec3(3, 0, 1.0), Vec3(0, 0, 0), Vec3(-3, 0, 1.5), Vec3(0, 0, 0))
        if out is not Outcome.ONGOING:
            return out
        t += 0.5
    return Outcome.ONGOING


def synthetic_one_clockwise_lap() -> int:
    """Fly a full clockwise circle about the pillar and count the laps."""
    trk = RuleTracker(side=Side.POS)
    radius = 1.0
    t = 0.0
    # Seed position so the first step establishes prev_pos.
    trk.step(t, Vec3(3, 0, 1.5), Vec3(0, 0, 0), Vec3(-3, 0, 1.5), Vec3(0, 0, 0))
    steps = 360
    for i in range(1, steps + 1):
        # Clockwise as seen from above: decreasing polar angle.
        ang = -2.0 * 3.141592653589793 * (i / steps)
        import math

        pos = Vec3(radius * math.cos(ang), radius * math.sin(ang), 1.5)
        t += 0.02
        trk.step(t, pos, Vec3(0, 0, 0), Vec3(-3, 0, 1.5), Vec3(0, 0, 0))
    return trk.clockwise_laps


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--synthetic", action="store_true", help="run built-in smoke checks")
    ap.add_argument("--replay", type=Path, help="replay a recorded match (not yet implemented)")
    args = ap.parse_args()

    if args.replay:
        print("replay mode not implemented yet -- waiting on the simulator's log format")
        return 1

    checks = [
        ("takeoff failure", synthetic_takeoff_failure(), Outcome.TAKEOFF_FAIL),
        ("hover to timeout", synthetic_hover_to_timeout(), Outcome.TIMEOUT_WIN),
        ("low-altitude loss", synthetic_low_altitude_loss(), Outcome.FLIGHT_LOSS),
        ("one clockwise lap", synthetic_one_clockwise_lap(), 1),
    ]

    failed = 0
    for name, got, want in checks:
        ok = got == want
        failed += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {name}: got {got!r}, want {want!r}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
