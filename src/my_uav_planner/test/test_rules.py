"""Self-tests for rules.py, runnable without the simulator.

    uv run python -m unittest discover -s test -v
"""

import math
import unittest

from my_uav_planner import rules as R
from my_uav_planner.rules import Side, Vec3


class TestGeometry(unittest.TestCase):
    def test_field_matches_pdf(self):
        self.assertEqual((R.FIELD_LENGTH, R.FIELD_WIDTH, R.FIELD_HEIGHT), (9.0, 6.0, 3.0))

    def test_takeoff_points(self):
        self.assertEqual(R.TAKEOFF[Side.POS].as_tuple(), (3.0, 0.0, 0.0))
        self.assertEqual(R.TAKEOFF[Side.NEG].as_tuple(), (-3.0, 0.0, 0.0))

    def test_initial_headings_face_each_other(self):
        self.assertEqual(R.INITIAL_HEADING_XY[Side.POS].as_tuple(), (-1.0, 0.0, 0.0))
        self.assertEqual(R.INITIAL_HEADING_XY[Side.NEG].as_tuple(), (1.0, 0.0, 0.0))

    def test_obstacle_is_600mm_wide_2400mm_tall(self):
        (xr, yr, zr) = R.obstacle_bounds()
        self.assertAlmostEqual(xr[1] - xr[0], 0.600)
        self.assertAlmostEqual(yr[1] - yr[0], 0.600)
        self.assertAlmostEqual(zr[1] - zr[0], 2.400)

    def test_side_other_is_involution(self):
        self.assertIs(Side.POS.other, Side.NEG)
        self.assertIs(Side.NEG.other.other, Side.NEG)


class TestVec3(unittest.TestCase):
    def test_normalize(self):
        v = Vec3(3.0, 4.0, 0.0).normalized()
        self.assertAlmostEqual(v.norm(), 1.0)
        self.assertAlmostEqual(v.x, 0.6)

    def test_normalize_zero_raises(self):
        with self.assertRaises(ValueError):
            Vec3(0.0, 0.0, 0.0).normalized()

    def test_horizontal_distance_ignores_z(self):
        self.assertAlmostEqual(Vec3(0, 0, 10).horizontal_distance_to(Vec3(3, 4, 0)), 5.0)


class TestFlightEnvelope(unittest.TestCase):
    def test_airborne_boundary_is_inclusive(self):
        self.assertTrue(R.is_airborne(1.2))
        self.assertTrue(R.is_airborne(1.2000001))
        self.assertFalse(R.is_airborne(1.1999999))

    def test_grace_and_low_altitude_constants(self):
        self.assertEqual(R.TAKEOFF_GRACE_PERIOD, 15.0)
        self.assertEqual(R.LOW_ALTITUDE_DURATION, 10.0)
        self.assertEqual(R.ROUND_DURATION, 300.0)


class TestQuadrantStar(unittest.TestCase):
    """Each side's counting gate lies on its own side of the pillar."""

    def test_pos_side_point_is_on_positive_x(self):
        p = Vec3(2.0, 1.0, 1.5)
        self.assertGreater(p.x, 0.0)

    def test_negative_control(self):
        p = Vec3(-2.0, 1.0, 1.5)
        self.assertLess(p.x, 0.0)


class TestAngleAndLaps(unittest.TestCase):
    def test_angle_about_axis_cardinals(self):
        self.assertAlmostEqual(R.angle_about_axis(Vec3(1, 0, 0)), 0.0)
        self.assertAlmostEqual(R.angle_about_axis(Vec3(0, 1, 0)), math.pi / 2)
        self.assertAlmostEqual(R.angle_about_axis(Vec3(-1, 0, 0)), math.pi)

    def test_clockwise_step_detection(self):
        # Going (1,0) -> (0,-1) is clockwise about +z.
        self.assertTrue(R.is_clockwise_step(Vec3(1, 0, 0), Vec3(0, -1, 0)))
        # The reverse is counter-clockwise.
        self.assertFalse(R.is_clockwise_step(Vec3(0, -1, 0), Vec3(1, 0, 0)))

    def test_partial_lap_does_not_count(self):
        # Three quarters of a clockwise turn.
        self.assertEqual(R.laps_from_unwrapped_angle(-1.5 * math.pi), 0)

    def test_one_full_clockwise_lap_counts(self):
        self.assertEqual(R.laps_from_unwrapped_angle(-2.0 * math.pi), 1)

    def test_counter_clockwise_never_counts(self):
        self.assertEqual(R.laps_from_unwrapped_angle(+2.0 * math.pi), 0)
        self.assertEqual(R.laps_from_unwrapped_angle(+10.0 * math.pi), 0)

    def test_two_and_a_half_laps(self):
        self.assertEqual(R.laps_from_unwrapped_angle(-2.5 * 2.0 * math.pi), 2)


class TestCollisionScoring(unittest.TestCase):
    def test_overlapping_boxes_detected(self):
        a = R.CollisionBox(Vec3(0, 0, 1), Vec3(0.1, 0.1, 0.1))
        b = R.CollisionBox(Vec3(0.15, 0, 1), Vec3(0.1, 0.1, 0.1))
        self.assertTrue(a.overlaps(b))

    def test_separated_boxes_not_detected(self):
        a = R.CollisionBox(Vec3(0, 0, 1), Vec3(0.1, 0.1, 0.1))
        b = R.CollisionBox(Vec3(5, 0, 1), Vec3(0.1, 0.1, 0.1))
        self.assertFalse(a.overlaps(b))

    def test_pillar_box_bounds(self):
        box = R.pillar_box()
        self.assertAlmostEqual(box.center.z, 1.2)
        self.assertAlmostEqual(box.half_extents.x, 0.3)

    def test_head_on_approach_charges_the_faster_mover(self):
        # Both fly straight at each other on the x-axis.
        own_pos, opp_pos = Vec3(-1, 0, 1.5), Vec3(1, 0, 1.5)
        own_vel, opp_vel = Vec3(2.0, 0, 0), Vec3(-1.0, 0, 0)
        p_own, p_opp = R.collision_projection(own_vel, opp_vel, own_pos, opp_pos)
        self.assertAlmostEqual(p_own, 2.0)
        self.assertAlmostEqual(p_opp, 1.0)
        # We moved toward the opponent faster => we are penalized.
        self.assertEqual(R.collision_loser(p_own, p_opp), +1)

    def test_backing_away_wins_the_collision_ruling(self):
        # We retreat while the opponent charges: opponent takes the loss.
        own_pos, opp_pos = Vec3(-1, 0, 1.5), Vec3(1, 0, 1.5)
        own_vel, opp_vel = Vec3(1.5, 0, 0), Vec3(-2.0, 0, 0)
        p_own, p_opp = R.collision_projection(own_vel, opp_vel, own_pos, opp_pos)
        self.assertEqual(R.collision_loser(p_own, p_opp), -1)


class TestWallAndFloorContact(unittest.TestCase):
    def test_center_is_clear(self):
        self.assertFalse(R.wall_contact(Vec3(0, 0, 1.5)))
        self.assertFalse(R.floor_contact(Vec3(0, 0, 1.5)))

    def test_x_wall_contact(self):
        self.assertTrue(R.wall_contact(Vec3(4.5 - R.UAV_BODY_HALF_EXTENTS.x, 0, 1.5)))

    def test_ceiling_contact(self):
        self.assertTrue(R.wall_contact(Vec3(0, 0, 3.0 - R.UAV_BODY_HALF_EXTENTS.z)))

    def test_floor_contact_at_rest(self):
        self.assertTrue(R.floor_contact(Vec3(3.0, 0.0, 0.0)))


class TestLapBookkeeping(unittest.TestCase):
    def test_net_laps_subtract_collisions(self):
        self.assertEqual(R.net_laps(3, 1), 2)

    def test_net_laps_can_be_negative(self):
        self.assertEqual(R.net_laps(1, 4), -3)


class TestTimeout(unittest.TestCase):
    def test_more_laps_wins(self):
        origin = Vec3(0, 0, 1.5)
        self.assertEqual(R.timeout_winner(2, 1, origin, origin), +1)
        self.assertEqual(R.timeout_winner(1, 2, origin, origin), -1)

    def test_tie_broken_by_distance_to_axis(self):
        near = Vec3(0.5, 0.0, 1.5)
        far = Vec3(3.0, 0.0, 1.5)
        self.assertEqual(R.timeout_winner(2, 2, near, far), +1)
        self.assertEqual(R.timeout_winner(2, 2, far, near), -1)

    def test_exact_tie(self):
        p = Vec3(1.0, 1.0, 1.5)
        self.assertEqual(R.timeout_winner(0, 0, p, p), 0)


if __name__ == "__main__":
    unittest.main()
