"""Real ROS tests with only the hardware driver faked."""

import time
import uuid
from dataclasses import replace
from unittest.mock import Mock, call

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sailing.teensy import teensy_node
from std_msgs.msg import Int32, UInt8


@pytest.fixture
def ros():
    rclpy.init(args=["--ros-args", "-r", "__ns:=/test_" + uuid.uuid4().hex])
    yield
    rclpy.shutdown()


@pytest.fixture
def ros_node(ros, monkeypatch):
    hardware = Mock(
        send_command=Mock(return_value=0), read_telemetry=Mock(return_value=1)
    )
    monkeypatch.setattr(
        teensy_node.serial_port, "SerialPort", Mock(return_value=hardware)
    )
    node = teensy_node.Teensy()
    peer = Node("test_peer")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    yield node, peer, hardware, executor
    executor.shutdown()
    peer.destroy_node()
    # Cleanup also works after the explicit close-failure test.
    hardware.close.side_effect = None
    node.destroy_node()


def spin_until(executor, predicate):
    deadline = time.monotonic() + 10
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate(), "Timed out waiting for ROS"


def observe_telemetry(node, peer, executor):
    received = {}
    topics = {
        "actual_mainsail_angle": Int32,
        "actual_rudder_angle": Int32,
        "actual_jib_angle": Int32,
        "actual_jib_side_flag": UInt8,
        "dropped_packets": Int32,
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


def test_command_topics_send_full_latest_command(ros_node):
    node, peer, hardware, executor = ros_node
    node.timer.cancel()
    commands = [
        ("mainsail_angle", Int32, 20),
        ("rudder_angle", Int32, -10),
        ("jib_angle", Int32, 30),
        ("jib_side_flag", UInt8, 1),
    ]
    for count, (topic, message_type, value) in enumerate(commands, start=1):
        publisher = peer.create_publisher(message_type, topic, 10)
        spin_until(executor, lambda: publisher.get_subscription_count() == 1)
        publisher.publish(message_type(data=value))
        spin_until(executor, lambda: hardware.send_command.call_count >= count)
    assert hardware.send_command.call_args_list == [
        call(20, 0, 0, 0),
        call(20, -10, 0, 0),
        call(20, -10, 30, 0),
        call(20, -10, 30, 1),
    ]


def test_timer_publishes_complete_telemetry(ros_node):
    node, peer, hardware, executor = ros_node
    received = observe_telemetry(node, peer, executor)
    sample = dict(
        wind_angle=300,
        mainsail_angle=20,
        rudder_angle=-10,
        jib_angle=30,
        jib_side_flag=1,
        dropped_packets=7,
    )

    def read(data):
        data.update(sample)
        return 0

    hardware.read_telemetry.side_effect = read
    expected = {
        "actual_mainsail_angle": 20,
        "actual_rudder_angle": -10,
        "actual_jib_angle": 30,
        "actual_jib_side_flag": 1,
        "dropped_packets": 7,
    }
    spin_until(executor, lambda: set(received) == set(expected))
    assert {topic: values[-1] for topic, values in received.items()} == expected


@pytest.mark.parametrize("error", [None, OSError("unplugged")])
def test_no_publication_on_missing_or_failed_read(ros_node, monkeypatch, error):
    node, peer, hardware, executor = ros_node
    node.timer.cancel()
    received = observe_telemetry(node, peer, executor)
    logger = Mock(wraps=node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    hardware.read_telemetry.side_effect = error
    node.check_telemetry()
    deadline = time.monotonic() + 0.2
    spin_until(executor, lambda: time.monotonic() >= deadline)
    assert not received
    if error:
        logger.warning.assert_called_once()
        assert logger.warning.call_args.kwargs["throttle_duration_sec"] > 0
    else:
        logger.warning.assert_not_called()


@pytest.mark.parametrize(
    "error", [None, OSError("unplugged"), ValueError("invalid angle")]
)
def test_command_failure_is_reported(ros_node, monkeypatch, error):
    node, _, hardware, _ = ros_node
    logger = Mock(wraps=node.get_logger())
    monkeypatch.setattr(node, "get_logger", lambda: logger)
    hardware.send_command.return_value = 1
    hardware.send_command.side_effect = error
    node.rudder_angle_callback(Int32(data=10))
    logger.warning.assert_called_once()
    assert "Command failed" in logger.warning.call_args.args[0]
    if error:
        assert str(error) in logger.warning.call_args.args[0]


@pytest.mark.parametrize("period", [0, -0.1])
def test_invalid_poll_period_does_not_open_port(ros, monkeypatch, period):
    monkeypatch.setattr(
        teensy_node,
        "TEENSY",
        replace(teensy_node.TEENSY, DEFAULT_TELEMETRY_POLL_PERIOD_SECONDS=period),
    )
    factory = Mock()
    monkeypatch.setattr(teensy_node.serial_port, "SerialPort", factory)
    node = teensy_node.Teensy.__new__(teensy_node.Teensy)
    try:
        with pytest.raises(ValueError):
            node.__init__()
        factory.assert_not_called()
    finally:
        Node.destroy_node(node)


def test_serial_open_failure_propagates(ros, monkeypatch):
    monkeypatch.setattr(
        teensy_node.serial_port, "SerialPort", Mock(side_effect=OSError("missing port"))
    )
    node = teensy_node.Teensy.__new__(teensy_node.Teensy)
    try:
        with pytest.raises(OSError, match="missing port"):
            node.__init__()
    finally:
        Node.destroy_node(node)


@pytest.mark.parametrize("error", [None, OSError("close failed")])
def test_shutdown_releases_ros_even_if_serial_close_fails(ros_node, error):
    node, _, hardware, _ = ros_node
    hardware.close.side_effect = error
    if error:
        with pytest.raises(OSError, match="close failed"):
            node.destroy_node()
    else:
        node.destroy_node()
    hardware.close.assert_called_once()
    assert not list(node.publishers)
    assert not list(node.subscriptions)
    assert not list(node.timers)
