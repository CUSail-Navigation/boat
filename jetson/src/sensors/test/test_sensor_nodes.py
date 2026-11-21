"""Real ROS topic delivery with hardware mocked only inside tests."""

import time
import uuid
from unittest.mock import Mock

import pytest
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensors.vectornav import vectornav_node
from sensors.vectornav.vectornav_node import VectorNav
from sensors.vectornav.vectornav_real import NavigationSample, SailVectorNav


@pytest.fixture
def sensors(monkeypatch):
    rclpy.init(
        args=[
            "--ros-args",
            "-r",
            "__ns:=/test_" + uuid.uuid4().hex,
        ]
    )
    monkeypatch.setattr(
        vectornav_node,
        "SailVectorNav",
        Mock(
            return_value=Mock(
                read_sample=Mock(return_value=NavigationSample(0.0, 0.0, 0.0, 0.0, 0.0))
            )
        ),
    )
    vectornav = VectorNav()
    peer = Node("sensor_test_peer")
    executor = SingleThreadedExecutor()
    for node in (vectornav, peer):
        executor.add_node(node)
    yield vectornav, peer, executor
    executor.shutdown()
    for node in (vectornav, peer):
        node.destroy_node()
    rclpy.shutdown()


def spin_until(executor, predicate):
    deadline = time.monotonic() + 10
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert predicate(), "Timed out waiting for sensor topics"


def test_sensor_topics_and_message_mapping(sensors):
    vectornav, peer, executor = sensors
    vectornav.sensor = Mock(
        read_sample=Mock(return_value=NavigationSample(90.0, -2.0, 7.0, 42.8, -77.0))
    )
    received = {}
    subscriptions = [
        peer.create_subscription(
            kind, topic, lambda msg, topic=topic: received.update({topic: msg}), 10
        )
        for topic, kind in (
            ("gps", NavSatFix),
            ("imu", Vector3),
        )
    ]
    spin_until(executor, lambda: len(received) == 2)
    assert all(
        sub.topic_name.startswith(peer.get_namespace() + "/") for sub in subscriptions
    )
    assert received["gps"].latitude == 42.8
    assert received["gps"].longitude == -77.0
    assert received["gps"].altitude == 0.0
    assert received["gps"].header.stamp.sec == 0
    assert received["gps"].header.stamp.nanosec == 0
    assert received["gps"].header.frame_id == ""
    assert (received["imu"].x, received["imu"].y, received["imu"].z) == (7, -2, 90)


def test_startup_samples_are_published_at_boat_period(sensors):
    vectornav, _, _ = sensors
    assert vectornav.timer.timer_period_ns == 500_000_000
    vectornav.timer.cancel()
    vectornav.sensor = Mock(
        read_sample=Mock(return_value=NavigationSample(0.0, 0.0, 0.0, 0.0, 0.0))
    )
    vectornav.publisher_gps = Mock()
    vectornav.publisher_imu = Mock()
    vectornav.gps_callback()
    vectornav.gps_callback()
    assert vectornav.publisher_gps.publish.call_count == 2
    assert vectornav.publisher_imu.publish.call_count == 2
    assert vectornav.publisher_gps.publish.call_args.args[0].latitude == 0.0
    assert vectornav.publisher_imu.publish.call_args.args[0].z == 0.0


def test_cleanup_closes_sensor(sensors):
    vectornav, _, _ = sensors
    vectornav.sensor.close = Mock()
    vectornav.destroy_node()
    vectornav.sensor.close.assert_called_once()


@pytest.mark.parametrize(
    "yaw,heading",
    [(0, 0), (90, 90), (-90, 270), (180, 180), (-180, 180), (-0.1, 359.9), (360, 0)],
)
def test_published_compass_heading(sensors, yaw, heading):
    vectornav, _, _ = sensors
    vectornav.timer.cancel()
    # Exercise the actual serial parser through the publisher boundary.
    vectornav.sensor.read_sample.return_value = SailVectorNav.parse(
        f"$VNINS,1,2,3,{yaw},-2,7,42.8,-77.0"
    )
    vectornav.publisher_imu = Mock()
    vectornav.gps_callback()
    message = vectornav.publisher_imu.publish.call_args.args[0]
    assert (message.x, message.y) == (7, -2)
    assert message.z == pytest.approx(heading)
