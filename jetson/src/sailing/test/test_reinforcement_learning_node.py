"""ROS tests for the RL node, checked against sailbench WaypointEnv conventions."""

import math
import time
import uuid
from dataclasses import dataclass
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Vector3
from interface.srv import Waypoint
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sailing.constants import EARTH_RADIUS_METERS, PHYSICAL
from sailing.reinforcement_learning import reinforcement_learning_node as rl
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, UInt8

LAT0, LON0 = 42.444, -76.483
# Apparent wind arrives as whole degrees, so wind-derived inputs are only this close.
OBS_TOLERANCE = 0.02


@pytest.fixture
def ros():
    rclpy.init(args=["--ros-args", "-r", "__ns:=/test_" + uuid.uuid4().hex])
    yield
    rclpy.shutdown()


@pytest.fixture
def rl_node(ros, monkeypatch):
    node = rl.ReinforcementLearning()
    node.timer.cancel()
    clock = {"t": 0.0}
    monkeypatch.setattr(node, "_now_seconds", lambda: clock["t"])
    monkeypatch.setattr(node.policy, "predict", Mock(wraps=node.policy.predict))
    peer = Node("test_peer")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    yield node, peer, clock, executor
    executor.shutdown()
    peer.destroy_node()
    node.destroy_node()


def spin_until(executor, predicate):
    deadline = time.monotonic() + 10
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate(), "Timed out waiting for ROS"


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    spin_until(executor, lambda: time.monotonic() >= deadline)


def enu_to_fix(east, north):
    """Flat-earth offset in metres from (LAT0, LON0) -> NavSatFix."""
    return NavSatFix(
        latitude=LAT0 + math.degrees(north / EARTH_RADIUS_METERS),
        longitude=LON0
        + math.degrees(east / (EARTH_RADIUS_METERS * math.cos(math.radians(LAT0)))),
    )


def compass_deg(psi):
    """Sim heading (rad, CCW from east) -> compass heading (deg, CW from north)."""
    return (90.0 - math.degrees(psi)) % 360.0


def world_to_boat(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, s], [-s, c]])


@dataclass
class Scenario:
    """Boat state in sailbench frames: world x=east, y=north; boat x=fwd, y=port."""

    psi_deg: float  # heading, CCW from east
    u: float  # surge, m/s
    v: float  # sway, m/s
    r: float  # yaw rate, rad/s CCW
    wind_to_deg: float  # true wind "blows to" direction, CCW from east
    wind_speed: float  # true wind speed, m/s
    waypoint_enu: tuple  # waypoint offset from the boat, metres
    actual_rudder: int = 0
    actual_mainsail: int = 0


SCENARIOS = {
    "north_beam_reach": Scenario(90, 2.0, 0.1, 0.2, 180, 5.0, (-8.0, 15.0), 10, 40),
    "southwest_sideslip": Scenario(
        225, 1.2, -0.4, -0.5, 30, 8.0, (12.0, -4.0), -20, 70
    ),
    "east_far_waypoint": Scenario(0, 0.5, 0.0, 0.05, 270, 3.0, (60.0, 30.0), 0, 0),
    "upwind_slow": Scenario(300, 0.3, 0.2, -0.1, 300 + 180, 6.0, (3.0, 9.0), 35, 85),
}


def feed(node, clock, s, dt=0.5):
    """Deliver the scenario as sensor messages: two GPS/IMU samples dt apart."""
    psi = math.radians(s.psi_deg)
    velocity_enu = world_to_boat(psi).T @ np.array([s.u, s.v])
    clock["t"] = 0.0
    node.gps_callback(enu_to_fix(*(-velocity_enu * dt)))
    node.imu_callback(Vector3(z=compass_deg(psi - s.r * dt)))
    clock["t"] = dt
    node.gps_callback(enu_to_fix(0.0, 0.0))
    node.imu_callback(Vector3(z=compass_deg(psi)))

    # Apparent wind = true wind - boat velocity, sent as "from, CW from bow".
    true_wind = s.wind_speed * np.array(
        [math.cos(math.radians(s.wind_to_deg)), math.sin(math.radians(s.wind_to_deg))]
    )
    apparent = world_to_boat(psi) @ true_wind - np.array([s.u, s.v])
    angle = round(math.degrees(math.atan2(apparent[1], -apparent[0]))) % 360
    node.wind_callback(Int32(data=angle))
    node.wind_speed = float(np.linalg.norm(apparent))  # Stands in for the constant.

    node.current_waypoint_callback(enu_to_fix(*s.waypoint_enu))
    node.actual_rudder_angle_callback(Int32(data=s.actual_rudder))
    node.actual_mainsail_angle_callback(Int32(data=s.actual_mainsail))


def waypoint_env_observation(s, prev_action, cfg):
    """WaypointEnv._get_observation for the scenario, plus the actuator state."""
    psi = math.radians(s.psi_deg)
    radius = cfg["waypoint_max_radius_m"]
    rel_wp = world_to_boat(psi) @ np.array(s.waypoint_enu)
    distance = max(math.hypot(*s.waypoint_enu), 1e-9)
    rel_dir = rel_wp / distance
    wind_rad = math.radians(s.wind_to_deg)
    wind_boat = world_to_boat(psi) @ np.array([math.cos(wind_rad), math.sin(wind_rad)])
    return np.array(
        [
            np.clip(rel_wp[0] / radius, -1.0, 1.0),
            np.clip(rel_wp[1] / radius, -1.0, 1.0),
            np.clip(distance / radius, 0.0, 1.0),
            np.clip(rel_dir[0], -1.0, 1.0),
            np.clip(rel_dir[1], -1.0, 1.0),
            np.clip(s.u / cfg["speed_scale"], -1.0, 1.0),
            np.clip(s.v / cfg["speed_scale"], -1.0, 1.0),
            np.clip(s.r / cfg["yaw_rate_scale"], -1.0, 1.0),
            np.clip(wind_boat[0], -1.0, 1.0),
            np.clip(wind_boat[1], -1.0, 1.0),
            np.clip(s.wind_speed / cfg["wind_speed_scale"], 0.0, 1.0),
            prev_action[0],
            prev_action[1],
            np.clip(s.actual_rudder / cfg["max_rudder_deg"], -1.0, 1.0),
            np.clip(2.0 * s.actual_mainsail / cfg["max_sail_deg"] - 1.0, -1.0, 1.0),
        ],
        dtype=np.float32,
    )


def observe_commands(node, peer, executor):
    received = {}
    topics = {
        "mainsail_angle": Int32,
        "rudder_angle": Int32,
        "jib_angle": Int32,
        "jib_side_flag": UInt8,
    }
    subscriptions = [
        peer.create_subscription(
            message_type,
            topic,
            lambda msg, topic=topic: received.setdefault(topic, []).append(msg.data),
            10,
        )
        for topic, message_type in topics.items()
    ]
    spin_until(
        executor,
        lambda: all(
            peer.count_publishers(subscription.topic_name) == 1
            and node.count_subscribers(subscription.topic_name) == 1
            for subscription in subscriptions
        ),
    )
    return received


def test_packaged_policy_matches_node(rl_node):
    node, *_ = rl_node
    assert node.policy.obs_dim == rl.OBS_DIM
    assert {
        "max_rudder_deg",
        "max_sail_deg",
        "waypoint_max_radius_m",
        "speed_scale",
        "yaw_rate_scale",
        "wind_speed_scale",
    } <= set(node.cfg)


def test_policy_with_wrong_observation_size_is_rejected(ros, tmp_path):
    path = tmp_path / "policy.npz"
    np.savez(
        path,
        n_layers=1,
        activation="tanh",
        w0=np.zeros((2, 13), np.float32),
        b0=np.zeros(2, np.float32),
    )
    rclpy.shutdown()
    rclpy.init(args=["--ros-args", "-p", f"policy_path:={path}"])
    node = rl.ReinforcementLearning.__new__(rl.ReinforcementLearning)
    try:
        with pytest.raises(ValueError, match="expects 13 observations"):
            node.__init__()
    finally:
        Node.destroy_node(node)


@pytest.mark.parametrize("scenario", SCENARIOS.values(), ids=SCENARIOS)
def test_observation_matches_waypoint_env(rl_node, scenario):
    node, _, clock, _ = rl_node
    feed(node, clock, scenario)
    node.run_inference()
    obs = node.policy.predict.call_args.args[0]
    action = rl.NumpyPolicy.predict(node.policy, obs)

    expected = waypoint_env_observation(scenario, [0.0, 0.0], node.cfg)
    np.testing.assert_allclose(obs, expected, atol=OBS_TOLERANCE)
    # The previous action is fed back on the next step.
    node.run_inference()
    np.testing.assert_allclose(node.policy.predict.call_args.args[0][11:13], action)


def test_yaw_rate_wraps_through_north(rl_node):
    node, _, clock, _ = rl_node
    node.imu_callback(Vector3(z=359.0))
    clock["t"] = 0.5
    node.imu_callback(Vector3(z=1.0))
    # Turning 2 degrees clockwise is a negative (CCW-positive) yaw rate.
    assert node.yaw_rate == pytest.approx(-math.radians(2.0) / 0.5)


@pytest.mark.parametrize(
    "missing",
    [
        "gps",
        "imu",
        "wind",
        "current_waypoint",
        "actual_mainsail_angle",
        "actual_rudder_angle",
        "velocity_enu",
        "yaw_rate",
    ],
)
def test_no_command_until_inputs_arrive(rl_node, monkeypatch, missing):
    node, peer, clock, executor = rl_node
    received = observe_commands(node, peer, executor)
    feed(node, clock, SCENARIOS["north_beam_reach"])
    setattr(node, missing, None)
    node.run_inference()
    spin_for(executor, 0.2)
    node.policy.predict.assert_not_called()
    assert not received


def test_single_gps_fix_has_no_velocity(rl_node):
    node, _, clock, _ = rl_node
    node.gps_callback(enu_to_fix(0.0, 0.0))
    assert node.velocity_enu is None


@pytest.mark.parametrize(
    ("action", "wind", "cfg", "expected"),
    [
        ([1.0, 1.0], 90, {}, (85, 35, 85, PHYSICAL.JIB_SIDE_PORT)),
        ([-1.0, -1.0], 270, {}, (0, -35, 0, PHYSICAL.JIB_SIDE_STB)),
        ([0.2, 0.5], 45, {}, (64, 7, 64, PHYSICAL.JIB_SIDE_PORT)),
        # Wider training limits are clamped to the physical ranges.
        (
            [1.0, 1.0],
            200,
            {"max_rudder_deg": 60.0, "max_sail_deg": 120.0},
            (90, 45, 90, PHYSICAL.JIB_SIDE_STB),
        ),
        (
            [-1.0, 1.0],
            200,
            {"max_rudder_deg": 60.0, "max_sail_deg": 120.0},
            (90, -45, 90, PHYSICAL.JIB_SIDE_STB),
        ),
    ],
)
def test_action_is_decoded_and_published(rl_node, action, wind, cfg, expected):
    node, peer, clock, executor = rl_node
    received = observe_commands(node, peer, executor)
    feed(node, clock, SCENARIOS["north_beam_reach"])
    node.cfg = {**node.cfg, **cfg}
    node.wind = wind
    node.policy.predict = Mock(return_value=np.array(action, dtype=np.float32))
    node.run_inference()
    spin_until(executor, lambda: len(received) == 4)
    assert (
        received["mainsail_angle"][-1],
        received["rudder_angle"][-1],
        received["jib_angle"][-1],
        received["jib_side_flag"][-1],
    ) == expected


@pytest.mark.parametrize("scenario", SCENARIOS.values(), ids=SCENARIOS)
def test_real_policy_commands_are_within_limits(rl_node, scenario):
    node, peer, clock, executor = rl_node
    received = observe_commands(node, peer, executor)
    feed(node, clock, scenario)
    for _ in range(5):
        node.run_inference()
    spin_until(
        executor,
        lambda: all(len(v) == 5 for v in received.values()) and len(received) == 4,
    )
    assert all(
        PHYSICAL.MAINSAIL_MIN_ANGLE <= a <= PHYSICAL.MAINSAIL_MAX_ANGLE
        for a in received["mainsail_angle"]
    )
    assert all(
        PHYSICAL.RUDDER_MIN_ANGLE <= a <= PHYSICAL.RUDDER_MAX_ANGLE
        for a in received["rudder_angle"]
    )
    assert all(
        PHYSICAL.JIB_MIN_ANGLE <= a <= PHYSICAL.JIB_MAX_ANGLE
        for a in received["jib_angle"]
    )


def fake_waypoint_queue(peer, executor, node, queue):
    """Serve mutate_waypoint_queue from peer, publishing the next waypoint like the real one."""
    publisher = peer.create_publisher(NavSatFix, "current_waypoint", 10)
    requests = []

    def handle(request, response):
        requests.append(request.command)
        queue.pop(0)
        if queue:
            publisher.publish(enu_to_fix(*queue[0]))
        response.success = True
        return response

    peer.create_service(Waypoint, "mutate_waypoint_queue", handle)
    spin_until(
        executor,
        lambda: node.waypoint_client.service_is_ready()
        and publisher.get_subscription_count() == 1,
    )
    return requests


def test_reached_waypoint_is_popped_without_commanding(rl_node):
    node, peer, clock, executor = rl_node
    received = observe_commands(node, peer, executor)
    queue = [(3.0, 2.0), (20.0, 20.0)]
    requests = fake_waypoint_queue(peer, executor, node, queue)
    feed(node, clock, Scenario(90, 1.0, 0.0, 0.0, 0, 5.0, queue[0]))

    node.run_inference()
    assert node.current_waypoint is None
    node.run_inference()  # No repeat pop while the next waypoint is in flight.
    spin_until(executor, lambda: node.current_waypoint is not None)

    assert requests == ["pop"]
    node.policy.predict.assert_not_called()
    assert not received
    expected = enu_to_fix(20.0, 20.0)
    assert node.current_waypoint.latitude == pytest.approx(expected.latitude)
    assert node.current_waypoint.longitude == pytest.approx(expected.longitude)
    # The boat sails again once the new waypoint is out of range.
    node.run_inference()
    node.policy.predict.assert_called_once()


def test_reached_waypoint_without_service_retries(rl_node, monkeypatch):
    node, _, clock, _ = rl_node
    logger = Mock(wraps=node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    feed(node, clock, Scenario(90, 1.0, 0.0, 0.0, 0, 5.0, (1.0, 1.0)))
    node.run_inference()
    assert node.current_waypoint is not None
    logger.warning.assert_called_once()
    assert "unavailable" in logger.warning.call_args.args[0]
    node.policy.predict.assert_not_called()


def test_topics_drive_timer_end_to_end(ros):
    """Real subscriptions, real clock and timer, real policy."""
    node = rl.ReinforcementLearning()
    peer = Node("test_peer")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    try:
        received = observe_commands(node, peer, executor)
        inputs = {
            "gps": NavSatFix,
            "imu": Vector3,
            "wind": Int32,
            "current_waypoint": NavSatFix,
            "actual_mainsail_angle": Int32,
            "actual_rudder_angle": Int32,
        }
        pubs = {
            topic: peer.create_publisher(t, topic, 10) for topic, t in inputs.items()
        }
        spin_until(
            executor,
            lambda: all(p.get_subscription_count() == 1 for p in pubs.values()),
        )

        pubs["current_waypoint"].publish(enu_to_fix(0.0, 20.0))
        pubs["wind"].publish(Int32(data=90))
        pubs["actual_mainsail_angle"].publish(Int32(data=30))
        pubs["actual_rudder_angle"].publish(Int32(data=0))
        for north, heading in [(0.0, 0.0), (0.5, 1.0)]:
            pubs["gps"].publish(enu_to_fix(0.0, north))
            pubs["imu"].publish(Vector3(z=heading))
            spin_for(executor, 0.3)

        spin_until(executor, lambda: len(received) == 4)
        assert (
            PHYSICAL.RUDDER_MIN_ANGLE
            <= received["rudder_angle"][-1]
            <= PHYSICAL.RUDDER_MAX_ANGLE
        )
        assert node.velocity_enu[1] > 0  # Moving north.
    finally:
        executor.shutdown()
        peer.destroy_node()
        node.destroy_node()
