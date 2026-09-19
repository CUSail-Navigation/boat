"""Exercise the waypoint contract through real ROS services and topics."""

import time
import uuid
from unittest.mock import Mock

import pytest
import rclpy
from interface.srv import Waypoint
from mission.waypoint_service import WaypointService
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


def spin_until(executor, predicate):
    deadline = time.monotonic() + 10
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert predicate(), "Timed out waiting for ROS"


@pytest.fixture
def queue(request, monkeypatch):
    namespace = "/test_" + uuid.uuid4().hex
    args = ["--ros-args", "-r", f"__ns:={namespace}"]
    initial = getattr(request, "param", None)
    if initial is not None:
        args.extend(["-p", f"waypoints:={initial!r}"])
    rclpy.init(args=args)
    node = peer = executor = None
    try:
        node = WaypointService()
        peer = Node("test_peer")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(peer)
        client = peer.create_client(Waypoint, "mutate_waypoint_queue")
        assert client.wait_for_service(timeout_sec=10)
        received = []
        subscription = peer.create_subscription(
            NavSatFix, "current_waypoint", received.append, 10
        )
        spin_until(
            executor,
            lambda: node.count_subscribers(subscription.topic_name) == 1
            and peer.count_publishers(subscription.topic_name) == 1,
        )
        # Keep real publication, but observe absence of publication deterministically.
        publisher = Mock(wraps=node.waypoint_publisher)
        monkeypatch.setattr(node, "waypoint_publisher", publisher)

        def call(command, argument=""):
            future = client.call_async(
                Waypoint.Request(command=command, argument=argument)
            )
            spin_until(executor, future.done)
            return future.result()

        yield call, received, executor, publisher
    finally:
        if executor is not None:
            executor.shutdown()
        if peer is not None:
            peer.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_default_queue_is_empty_and_pop_fails(queue):
    call, _, _, publisher = queue
    response = call("get")
    assert response.success
    assert response.message == "[]"
    assert not call("pop").success
    assert call("get").message == "[]"
    publisher.publish.assert_not_called()


@pytest.mark.parametrize("queue", [["42.0,-71.0", "42.1,-71.1"]], indirect=True)
def test_initial_waypoints_are_loaded_in_order(queue):
    call, _, _, _ = queue
    response = call("get")
    assert response.success
    assert response.message == "[(42.0, -71.0), (42.1, -71.1)]"


def test_set_replaces_queue_and_publishes_only_first_waypoint(queue):
    call, received, executor, _ = queue
    assert call("set", "42.0,-71.0;42.1,-71.1").success
    spin_until(executor, lambda: len(received) == 1)
    assert call("get").message == "[(42.0, -71.0), (42.1, -71.1)]"
    assert call("set", "43.0,-72.0").success
    spin_until(executor, lambda: len(received) == 2)
    assert call("get").message == "[(43.0, -72.0)]"
    assert [(msg.latitude, msg.longitude) for msg in received] == [
        (42.0, -71.0),
        (43.0, -72.0),
    ]
    for msg in received:
        assert msg.header.frame_id == "map"
        assert msg.header.stamp.sec > 0
        assert msg.altitude == 0.0
        assert msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN


def test_pop_advances_target_and_final_pop_publishes_nothing(queue):
    call, received, executor, publisher = queue
    assert call("set", "42.0,-71.0;42.1,-71.1").success
    spin_until(executor, lambda: len(received) == 1)
    assert call("pop").success
    spin_until(executor, lambda: len(received) == 2)
    assert (received[-1].latitude, received[-1].longitude) == (42.1, -71.1)
    assert call("get").message == "[(42.1, -71.1)]"
    publisher.reset_mock()
    assert call("pop").success
    assert call("get").message == "[]"
    assert not call("pop").success
    # Preserve the current contract: no empty-state / NaN message is emitted.
    publisher.publish.assert_not_called()


@pytest.mark.parametrize(
    "argument", ["", "invalid", "42.0", "42.0,-71.0,0", "43.0,-72.0;bad", "43.0,-72.0;"]
)
def test_invalid_set_preserves_previous_queue_without_publication(queue, argument):
    call, received, executor, publisher = queue
    assert call("set", "42.0,-71.0").success
    spin_until(executor, lambda: len(received) == 1)
    publisher.reset_mock()
    response = call("set", argument)
    assert not response.success
    assert "Failed to update waypoints" in response.message
    assert call("get").message == "[(42.0, -71.0)]"
    publisher.publish.assert_not_called()


def test_unknown_command_preserves_queue_without_publication(queue):
    call, received, executor, publisher = queue
    assert call("set", "42.0,-71.0").success
    spin_until(executor, lambda: len(received) == 1)
    publisher.reset_mock()
    response = call("clear")
    assert not response.success
    assert "Invalid command" in response.message
    assert call("get").message == "[(42.0, -71.0)]"
    publisher.publish.assert_not_called()
