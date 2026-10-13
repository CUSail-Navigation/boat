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
from std_msgs.msg import Float32, Int32
from sensors.licor import licor_node
from sensors.licor.licor_node import LicorNode
from sensors.licor.licor_real import WindSample
from sensors.vectornav import vectornav_node
from sensors.vectornav.vectornav_node import VectorNav
from sensors.vectornav.vectornav_real import NavigationSample


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
        licor_node,
        "LicorAnemometer",
        Mock(return_value=Mock(read_sample=Mock(return_value=WindSample(0.0, 0)))),
    )
    monkeypatch.setattr(
        vectornav_node,
        "SailVectorNav",
        Mock(
            return_value=Mock(
                read_sample=Mock(
                    return_value=NavigationSample(0.0, 0.0, 0.0, 0.0, 0.0)
                )
            )
        ),
    )
    licor = LicorNode()
    vectornav = VectorNav()
    peer = Node("sensor_test_peer")
    executor = SingleThreadedExecutor()
    for node in (licor, vectornav, peer):
        executor.add_node(node)
    yield licor, vectornav, peer, executor
    executor.shutdown()
    for node in (licor, vectornav, peer):
        node.destroy_node()
    rclpy.shutdown()


def spin_until(executor, predicate):
    deadline = time.monotonic() + 10
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert predicate(), "Timed out waiting for sensor topics"


def test_sensor_topics_and_message_mapping(sensors):
    licor, vectornav, peer, executor = sensors
    licor.alpha = 1.0
    licor.sensor = Mock(read_sample=Mock(return_value=WindSample(2.5, 270)))
    vectornav.sensor = Mock(
        read_sample=Mock(return_value=NavigationSample(90.0, -2.0, 7.0, 42.8, -77.0))
    )
    received = {}
    subscriptions = [
        peer.create_subscription(
            kind, topic, lambda msg, topic=topic: received.update({topic: msg}), 10
        )
        for topic, kind in (
            ("wind_angle", Int32),
            ("wind_speed", Float32),
            ("gps", NavSatFix),
            ("imu", Vector3),
        )
    ]
    spin_until(executor, lambda: len(received) == 4)
    assert all(
        sub.topic_name.startswith(peer.get_namespace() + "/") for sub in subscriptions
    )
    assert received["wind_angle"].data == 270
    assert received["wind_speed"].data == 2.5
    assert received["gps"].latitude == 42.8
    assert received["gps"].longitude == -77.0
    assert received["gps"].altitude == 0.0
    assert received["gps"].header.stamp.sec == 0
    assert received["gps"].header.stamp.nanosec == 0
    assert received["gps"].header.frame_id == ""
    assert (received["imu"].x, received["imu"].y, received["imu"].z) == (7, -2, 90)


def test_circular_filter_and_low_wind_hold(sensors):
    licor, _, _, _ = sensors
    licor.timer.cancel()
    licor.alpha = 0.5
    licor.filtered_u = 0.0
    licor.filtered_v = 1.0
    licor.sensor = Mock(read_sample=Mock(return_value=WindSample(3.0, 359)))
    licor.publish_wind()
    licor.sensor.read_sample.return_value = WindSample(3.0, 1)
    licor.publish_wind()
    assert licor.last_direction in (0, 359)
    previous = licor.last_direction
    licor.sensor.read_sample.return_value = WindSample(0.2, 180)
    licor.publish_wind()
    assert licor.last_direction == previous


def test_startup_samples_are_published_at_boat_period(sensors):
    licor, vectornav, _, _ = sensors
    for node, sample, callback, publishers in (
        (
            licor,
            WindSample(0.0, 0.0),
            licor.publish_wind,
            ("publisher", "speed_publisher"),
        ),
        (
            vectornav,
            NavigationSample(0.0, 0.0, 0.0, 0.0, 0.0),
            vectornav.gps_callback,
            ("publisher_gps", "publisher_imu"),
        ),
    ):
        assert node.timer.timer_period_ns == 500_000_000
        node.timer.cancel()
        node.sensor = Mock(read_sample=Mock(return_value=sample))
        for name in publishers:
            setattr(node, name, Mock())
        callback()
        callback()
        for name in publishers:
            assert getattr(node, name).publish.call_count == 2
    assert licor.publisher.publish.call_args.args[0].data == 0
    assert vectornav.publisher_gps.publish.call_args.args[0].latitude == 0.0
    assert vectornav.publisher_imu.publish.call_args.args[0].z == 0.0


def test_cleanup_closes_sensor(sensors):
    licor, vectornav, _, _ = sensors
    for node in (licor, vectornav):
        node.sensor.close = Mock()
        node.destroy_node()
        node.sensor.close.assert_called_once()
