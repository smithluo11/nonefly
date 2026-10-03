"""Competition rules as code.

Every constant and predicate here is a direct translation of
"20无人机对抗赛（仿真）比赛规则.pdf". Keep this module dependency-free
(stdlib only) so it can be unit-tested without the simulator and reused by
both the planner and the offline evaluator.

Conventions used throughout
---------------------------
* World frame is ROS/REP-103 style: x forward-ish, y left, **z up**.
* The field origin (0, 0, 0) is the *center of the floor*.
* "Center coordinates" means the point is the geometric center of the object,
  matching the PDF ("以下坐标均为中心点坐标").
* Sides are named by the sign of the start x-coordinate:
  ``POS_SIDE`` is the machine that starts at (+3, 0), ``NEG_SIDE`` at (-3, 0).
  Which side *you* get is assigned by the referee at match start; the two sides
  are exact mirrors of each other, so nothing here assumes you are either one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

# --------------------------------------------------------------------------- #
# Field geometry (PDF section 三(二))
# --------------------------------------------------------------------------- #

#: Interior dimensions of the sealed arena: long edge / short edge / height, meters.
FIELD_LENGTH = 9.0   # x, PDF calls this the 长边
FIELD_WIDTH = 6.0    # y, 短边
FIELD_HEIGHT = 3.0   # z, floor to ceiling


@dataclass(frozen=True)
class Vec3:
    """Plain 3-vector. Deliberately not numpy so rules.py stays import-light."""

    x: float
    y: float
    z: float

    def __add__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x + o.x, self.y + o.y, self.z + o.z)

    def __sub__(self, o: "Vec3") -> "Vec3":
        return Vec3(self.x - o.x, self.y - o.y, self.z - o.z)

    def __mul__(self, s: float) -> "Vec3":
        return Vec3(self.x * s, self.y * s, self.z * s)

    __rmul__ = __mul__

    def dot(self, o: "Vec3") -> float:
        return self.x * o.x + self.y * o.y + self.z * o.z

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self) -> "Vec3":
        n = self.norm()
        if n == 0.0:
            raise ValueError("cannot normalize a zero vector")
        return self * (1.0 / n)

    def horizontal(self) -> "Vec3":
        """Projection onto the floor plane (drop z)."""
        return Vec3(self.x, self.y, 0.0)

    def horizontal_distance_to(self, o: "Vec3") -> float:
        """Distance in the xy-plane -- used for the timeout tie-break."""
        return math.hypot(self.x - o.x, self.y - o.y)

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


# --------------------------------------------------------------------------- #
# Sides, takeoff points, obstacle (PDF section 三(二), 三(四))
# --------------------------------------------------------------------------- #

class Side(Enum):
    """Which half of the field a competitor starts from."""

    POS = +1   # starts at (+3, 0), initially faces -x
    NEG = -1   # starts at (-3, 0), initially faces +x

    @property
    def sign(self) -> int:
        return 1 if self is Side.POS else -1

    @property
    def other(self) -> "Side":
        return Side.NEG if self is Side.POS else Side.POS


#: Takeoff point of each side, on the floor (z = 0).
TAKEOFF = {
    Side.POS: Vec3(+3.0, 0.0, 0.0),
    Side.NEG: Vec3(-3.0, 0.0, 0.0),
}

#: Initial heading: each machine points its nose at the opponent.
#: (+3,0) faces -x, (-3,0) faces +x.
INITIAL_HEADING_XY = {
    Side.POS: Vec3(-1.0, 0.0, 0.0),
    Side.NEG: Vec3(+1.0, 0.0, 0.0),
}

#: Central square pillar: 600mm x 600mm footprint, 2400mm tall, at field center.
OBSTACLE_HALF_EXTENT = 0.300     # 600mm / 2
OBSTACLE_HEIGHT = 2.400          # 2400mm


def obstacle_bounds() -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    """Axis-aligned bounds of the pillar as (x_range, y_range, z_range)."""
    h = OBSTACLE_HALF_EXTENT
    return ((-h, h), (-h, h), (0.0, OBSTACLE_HEIGHT))


#: The QR board hangs on the tail: 100mm x 100mm, attached in the UAV's frame.
QR_BOARD_SIZE = 0.100            # meters, square


# --------------------------------------------------------------------------- #
# Flight envelope (PDF section 三(三), 四(二), 五)
# --------------------------------------------------------------------------- #

#: Body reference point (odometry origin) must stay at or above this, meters.
MIN_FLIGHT_ALTITUDE = 1.2

#: After the start signal, altitude must *first* reach MIN_FLIGHT_ALTITUDE
#: within this many seconds of simulated time, or the round is lost.
TAKEOFF_GRACE_PERIOD = 15.0

#: Once airborne, altitude below the minimum for a *continuous* span longer
#: than this is a loss. It is a duration, not an instantaneous check.
LOW_ALTITUDE_DURATION = 10.0

#: A match is at most this long, in simulated seconds.
ROUND_DURATION = 5.0 * 60.0

#: Maximum upload rate for the decoded string; faster messages are dropped.
MAX_UPLOAD_RATE_HZ = 30.0


def is_airborne(z: float, tol: float = 1e-9) -> bool:
    """True when the reference point has cleared the minimum altitude."""
    return z >= MIN_FLIGHT_ALTITUDE - tol


# --------------------------------------------------------------------------- #
# Lap counting / the timing gate (PDF section 三(三))
# --------------------------------------------------------------------------- #
#
# Each side has its own counting gate: the vertical half-plane that contains
# both its takeoff point and the pillar's axis. For the (+3, 0) side that is the
# half-plane {(x, y) : x > 0, y = 0}; for the (-3, 0) side, x < 0.
#
# A lap only counts if the UAV circles the pillar *clockwise as seen from above*
# and completes a full revolution. We accumulate the yaw angle of the vector
# (UAV - pillar_axis) frame-by-frame; a clockwise revolution sweeps -2*pi.

#: Two samples must be at least this far apart in angle before we trust the
#: cross-product sign, to avoid noise-dominated tiny steps near the axis.
_MIN_ANGLE_STEP = 1e-6


def laps_from_unwrapped_angle(unwrapped_delta: float) -> int:
    """Convert accumulated clockwise sweep (radians, negative) into whole laps.

    ``unwrapped_delta`` is the running sum of signed per-frame angle changes
    as the UAV orbits the pillar. Clockwise motion makes it negative, so a
    sweep of -2*pi is one lap. Counter-clockwise sweeps (positive) never count.
    """
    if unwrapped_delta >= 0.0:
        return 0
    return int((-unwrapped_delta) // (2.0 * math.pi))


def is_clockwise_step(prev_xy: Vec3, curr_xy: Vec3, axis_xy: Vec3 = Vec3(0.0, 0.0, 0.0)) -> bool:
    """True if the pair of positions is a clockwise step about ``axis_xy``.

    Clockwise about +z means the cross product (prev-axis) x (curr-axis) has a
    negative z-component.
    """
    a = prev_xy.horizontal() - axis_xy.horizontal()
    b = curr_xy.horizontal() - axis_xy.horizontal()
    cross_z = a.x * b.y - a.y * b.x
    return cross_z < -_MIN_ANGLE_STEP


def angle_about_axis(point_xy: Vec3, axis_xy: Vec3 = Vec3(0.0, 0.0, 0.0)) -> float:
    """Polar angle (radians, [-pi, pi]) of ``point`` around the pillar axis."""
    return math.atan2(point_xy.y - axis_xy.y, point_xy.x - axis_xy.x)


# --------------------------------------------------------------------------- #
# Collision detection and scoring (PDF section 三(四), 五)
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class CollisionBox:
    """Axis-aligned box used to approximate the UAV body / pillar / walls."""

    center: Vec3
    half_extents: Vec3

    def overlaps(self, other: "CollisionBox") -> bool:
        return (
            abs(self.center.x - other.center.x) <= self.half_extents.x + other.half_extents.x
            and abs(self.center.y - other.center.y) <= self.half_extents.y + other.half_extents.y
            and abs(self.center.z - other.center.z) <= self.half_extents.z + other.half_extents.z
        )


#: Approximate UAV collision box. The rules fix the model and forbid changing
#: it, so these numbers are placeholders until the official template publishes
#: the real dimensions -- keeping them here (not inlined) makes the one edit
#: point obvious.
UAV_BODY_HALF_EXTENTS = Vec3(0.150, 0.150, 0.075)   # 300mm x 300mm x 150mm


def pillar_box() -> CollisionBox:
    h = OBSTACLE_HALF_EXTENT
    return CollisionBox(
        center=Vec3(0.0, 0.0, OBSTACLE_HEIGHT / 2.0),
        half_extents=Vec3(h, h, OBSTACLE_HEIGHT / 2.0),
    )


def wall_contact(uav_center: Vec3, uav_half: Vec3 = UAV_BODY_HALF_EXTENTS) -> bool:
    """True when the UAV body touches a wall or the ceiling.

    A contact costs one lap (see ``net_laps``). The floor is *not* included
    here -- touching the floor is its own, harsher penalty.
    """
    x_lim = FIELD_LENGTH / 2.0 - uav_half.x
    y_lim = FIELD_WIDTH / 2.0 - uav_half.y
    z_lim = FIELD_HEIGHT - uav_half.z
    return (
        abs(uav_center.x) >= x_lim
        or abs(uav_center.y) >= y_lim
        or uav_center.z >= z_lim
    )


def floor_contact(uav_center: Vec3, uav_half: Vec3 = UAV_BODY_HALF_EXTENTS) -> bool:
    """True when any part of the UAV is at or below the floor plane."""
    return uav_center.z - uav_half.z <= 0.0


def collision_projection(
    own_vel: Vec3,
    opp_vel: Vec3,
    own_pos: Vec3,
    opp_pos: Vec3,
) -> tuple[float, float]:
    """Mean closing speed along the inter-body axis for a mutual-collision ruling.

    Per the rules: within the final 0.5s window, take each body's velocity
    projected onto the vector from *itself* toward the other body, average the
    two magnitudes, and the side with the *larger* value is the loser. Returns
    ``(projection_own, projection_opp)`` in m/s; call ``collision_loser`` to get
    the verdict.
    """
    toward_opp = (opp_pos - own_pos).horizontal().normalized()
    toward_own = (own_pos - opp_pos).horizontal().normalized()
    proj_own = abs(own_vel.horizontal().dot(toward_opp))
    proj_opp = abs(opp_vel.horizontal().dot(toward_own))
    return proj_own, proj_opp


def collision_loser(own_proj: float, opp_proj: float) -> int:
    """+1 if we lose the mutual collision, -1 if the opponent loses, 0 if equal."""
    if own_proj > opp_proj:
        return +1
    if own_proj < opp_proj:
        return -1
    return 0


# --------------------------------------------------------------------------- #
# Lap bookkeeping and the timeout tie-break (PDF section 三(三), 五)
# --------------------------------------------------------------------------- #

def net_laps(clockwise_laps: int, collisions: int) -> int:
    """Net laps = full clockwise laps minus collision penalties.

    May go negative; the rules explicitly allow that.
    """
    return clockwise_laps - collisions


def timeout_winner(
    own_net_laps: int,
    opp_net_laps: int,
    own_pos: Vec3,
    opp_pos: Vec3,
) -> int:
    """Resolve a timeout. +1 = we win, -1 = we lose, 0 = truly tied.

    Higher net lap count wins. If equal, the UAV whose horizontal distance to
    the pillar axis (the z-axis) is *smaller* wins.
    """
    if own_net_laps != opp_net_laps:
        return +1 if own_net_laps > opp_net_laps else -1
    own_d = own_pos.horizontal_distance_to(Vec3(0.0, 0.0, 0.0))
    opp_d = opp_pos.horizontal_distance_to(Vec3(0.0, 0.0, 0.0))
    if abs(own_d - opp_d) < 1e-9:
        return 0
    return +1 if own_d < opp_d else -1
