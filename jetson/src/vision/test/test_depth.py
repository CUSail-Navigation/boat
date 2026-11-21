"""Depth and size estimates from a detection box, without a camera."""

import numpy as np
import pytest
from vision.depth import box_depth, box_width

SCALE = 0.001  # RealSense z16 units are millimeters


def depth_image(value_mm):
    return np.full((480, 640), value_mm, dtype=np.uint16)


def test_median_depth_of_box_center():
    image = depth_image(5000)
    assert box_depth(image, (100, 100, 200, 200), SCALE, 0.5, 0.3, 25.0) == 5.0


def test_ignores_background_at_box_edges():
    # Water behind the buoy fills the edges of the box; only the center is sampled.
    image = depth_image(20000)
    image[125:176, 125:176] = 6000
    assert box_depth(image, (100, 100, 200, 200), SCALE, 0.5, 0.3, 25.0) == 6.0


def test_ignores_missing_and_out_of_range_depths():
    image = depth_image(0)
    image[150, 150] = 4000
    image[151, 151] = 40000  # past max depth
    assert box_depth(image, (100, 100, 200, 200), SCALE, 0.5, 0.3, 25.0) == 4.0


def test_no_valid_depth_returns_none():
    assert (
        box_depth(depth_image(0), (100, 100, 200, 200), SCALE, 0.5, 0.3, 25.0) is None
    )


def test_box_width_from_pinhole_model():
    # 60 px wide at 10 m with fx = 600 px is 1 m.
    assert box_width((100, 0, 160, 50), 10.0, 600.0) == pytest.approx(1.0)
