"""Compass-topic headings must retain the policy's CCW-positive yaw rate."""

from types import SimpleNamespace

import numpy as np
import pytest
from sailing.reinforcement_learning.reinforcement_learning_node import (
    ReinforcementLearning,
)


@pytest.mark.parametrize(
    "previous,current,expected_degrees",
    [(359, 1, -2), (1, 359, 2), (0, 90, -90), (90, 0, 90)],
)
def test_compass_yaw_rate(previous, current, expected_degrees):
    node = SimpleNamespace(
        imu=SimpleNamespace(z=previous),
        imu_time=0.0,
        yaw_rate=None,
        _now_seconds=lambda: 1.0,
    )
    message = SimpleNamespace(z=current)
    ReinforcementLearning.imu_callback(node, message)
    assert node.yaw_rate == pytest.approx(np.radians(expected_degrees))
    assert node.imu is message
    assert node.imu_time == 1.0
