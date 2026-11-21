"""Search logic without ROS: angles, the particle filter, visibility and leg planning."""

import math

import numpy as np
import pytest
from mission.search_core import (
    CoverageMap,
    ParticleFilter,
    RejectedZone,
    angle_difference,
    compass_to_math,
    measurement_to_point,
    point_visible,
    travel_cost,
)

HFOV = 85.2


@pytest.mark.parametrize(
    ("a", "b", "expected"),
    [(10, 350, 20), (350, 10, -20), (180, 0, -180), (90, 90, 0)],
)
def test_angle_difference(a, b, expected):
    assert angle_difference(a, b) == pytest.approx(expected)


@pytest.mark.parametrize(
    ("compass", "math_deg"), [(0, 90), (90, 0), (180, 270), (270, 180)]
)
def test_compass_to_math(compass, math_deg):
    assert compass_to_math(compass) == pytest.approx(math_deg)


def test_measurement_to_point():
    x, y = measurement_to_point(10.0, 20.0, 90.0, 5.0)  # 5 m north
    assert (x, y) == pytest.approx((10.0, 25.0))


def test_point_visible_respects_fov_and_range():
    # Boat at the origin facing east (math 0 deg)
    assert point_visible(0, 0, 0, 10, 0, HFOV, 4, 15)
    assert point_visible(0, 0, 0, 10, 8, HFOV, 4, 15)  # ~39 deg off the bow
    assert not point_visible(0, 0, 0, 10, 12, HFOV, 4, 15)  # ~50 deg, outside FOV
    assert not point_visible(0, 0, 0, -10, 0, HFOV, 4, 15)  # behind
    assert not point_visible(0, 0, 0, 2, 0, HFOV, 4, 15)  # too close
    assert not point_visible(0, 0, 0, 20, 0, HFOV, 4, 15)  # too far


def test_travel_cost_penalizes_upwind_only():
    # Wind from the north (upwind = 90 deg), no-go 45 deg
    assert travel_cost(10, 0, 90, 45) == pytest.approx(10)  # beam reach
    assert travel_cost(0, -10, 90, 45) == pytest.approx(10)  # downwind
    assert travel_cost(0, 10, 90, 45) == pytest.approx(10 / math.cos(math.radians(45)))
    assert travel_cost(0, 10, None, 45) == pytest.approx(10)  # wind unknown


def test_particle_filter_converges_on_buoy():
    rng = np.random.default_rng(0)
    buoy = (30.0, 40.0)
    filt = ParticleFilter(500, 32.0, 37.0, 5.0, rng)
    # Measurements from a boat moving past the buoy
    for boat_x in np.linspace(0, 40, 20):
        dx, dy = buoy[0] - boat_x, buoy[1]
        bearing = math.degrees(math.atan2(dy, dx)) + rng.normal(0, 3)
        observed_range = math.hypot(dx, dy) + rng.normal(0, 1)
        assert filt.update(bearing, boat_x, 0.0, observed_range, 8.0, 2.0)
        filt.resample(0.2)
    assert math.dist(filt.estimate(), buoy) < 2.0
    assert filt.spread() < 3.0


def test_particle_filter_gates_out_unrelated_detection():
    filt = ParticleFilter(300, 0.0, 10.0, 1.0, np.random.default_rng(1))
    before = filt.weights.copy()
    # A detection 30 m away in the opposite direction
    assert not filt.update(-90.0, 0.0, 0.0, 30.0, 8.0, 2.0)
    np.testing.assert_array_equal(filt.weights, before)


def test_rejected_zone_contains():
    zone = RejectedZone(0.0, 0.0, 10.0, ignore_until=5.0)
    assert zone.contains(6, 8)
    assert not zone.contains(8, 8)


def test_observe_credits_only_cells_in_view():
    cov = CoverageMap(0, 0, 30, 2.0)
    before = cov.p.copy()
    cov.observe(0, 0, 0, HFOV, 4, 10, 15, 1.0, 1.0)  # facing east from the center
    seen = cov.p < before
    assert seen.any()
    assert np.all(cov.xs[seen] > 0)  # only cells to the east lost probability
    assert cov.coverage() > 0


def sail(cov, x, y, to_x, to_y, step=1.0):
    """Sail a straight line, crediting the camera's view every meter."""
    heading = math.degrees(math.atan2(to_y - y, to_x - x))
    steps = max(int(math.hypot(to_x - x, to_y - y) / step), 1)
    for i in range(1, steps + 1):
        px = x + (to_x - x) * i / steps
        py = y + (to_y - y) * i / steps
        cov.observe(px, py, heading, HFOV, 4, 10, 15, 1.0, step)
    return to_x, to_y


def test_lawnmower_legs_cover_the_area():
    radius, spacing = 50.0, 16.0
    cov = CoverageMap(0, 0, radius, 2.0)
    x, y = -radius - 10, 0.0
    legs = []
    while len(legs) < 50:
        leg = cov.plan_leg(x, y, 0.0, spacing, 10.0, 10.0, 0.35)
        if leg is None:
            break
        start, end, cost = leg
        assert cost > 0
        for point in (start, end):
            assert math.hypot(*point) <= radius + 1e-6  # legs stay inside the circle
        x, y = sail(cov, x, y, *start)
        x, y = sail(cov, x, y, *end)
        legs.append(leg)
    assert legs and len(legs) < 50, "planner should finish a pass"
    assert cov.coverage() > 0.9


def test_planning_skips_water_already_seen():
    cov = CoverageMap(0, 0, 40, 2.0)
    first = cov.plan_leg(-50, 0, 0.0, 16.0, 10.0, 10.0, 0.35)
    # Sail to the first leg and along it before asking again
    (sx, sy), (ex, ey), _ = first
    sail(cov, *sail(cov, -50, 0, sx, sy), ex, ey)
    second = cov.plan_leg(ex, ey, 0.0, 16.0, 10.0, 10.0, 0.35)
    assert second is not None
    # Legs run north-south (direction 0 deg), so lines differ in x
    assert abs(second[0][0] - first[0][0]) > 8.0
