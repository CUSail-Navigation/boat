"""Exercise the waypoint contract through real ROS services and topics."""

import time
import uuid
from unittest.mock import Mock

import pytest
import rclpy
from interface.msg import Waypoint
from interface.srv import Waypoint as WaypointSrv
from mission.waypoint_service import WaypointService
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


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
        client = peer.create_client(WaypointSrv, "mutate_waypoint_queue")
        assert client.wait_for_service(timeout_sec=10)
        received = []
        subscription = peer.create_subscription(
            Waypoint, "current_waypoint", received.append, 10
        )
        spin_until(
            executor,
            lambda: node.count_subscribers(subscription.topic_name) == 1
            and peer.count_publishers(subscription.topic_name) == 1,
        )
        # Keep real publication, but observe absence of publication deterministically.
        publisher = Mock(wraps=node.waypoint_publisher)
        monkeypatch.setattr(node, "waypoint_publisher", publisher)

        def call(command, expected_version="", waypoints=None):
            future = client.call_async(
                WaypointSrv.Request(
                    command=command,
                    expected_version=expected_version,
                    waypoints=waypoints or [],
                )
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


def read(call):
    response = call("get")
    assert response.success
    return {"version": response.version, "waypoints": response.waypoints}


def point(key, latitude=42.0, longitude=-71.0):
    return Waypoint(id=key, latitude=float(latitude), longitude=float(longitude))


def replace(call, points):
    return call("set", read(call)["version"], points)


def test_default_queue_is_empty_and_pop_fails(queue):
    call, _, _, publisher = queue
    state = read(call)
    assert state["waypoints"] == []
    assert isinstance(state["version"], str) and state["version"]
    assert not call("pop").success
    assert read(call) == state
    publisher.publish.assert_not_called()


@pytest.mark.parametrize("queue", [["42.0,-71.0", "42.1,-71.1"]], indirect=True)
def test_initial_waypoints_are_loaded_in_order(queue):
    call, _, _, _ = queue
    items = read(call)["waypoints"]
    assert [(w.latitude, w.longitude) for w in items] == [
        (42, -71),
        (42.1, -71.1),
    ]
    assert len({w.id for w in items}) == 2
    assert read(call)["waypoints"] == items


def test_set_reorders_deletes_and_clears_queue(queue):
    call, received, executor, publisher = queue
    points = [point("a"), point("b", 43, -72), point("c", 43, -72)]
    for items in (points, list(reversed(points)), [points[1]]):
        previous = read(call)
        response = replace(call, items)
        assert response.success
        state = {"version": response.version, "waypoints": response.waypoints}
        assert state == read(call)
        assert state["waypoints"] == items
        assert state["version"] != previous["version"]
    spin_until(executor, lambda: len(received) == 3)
    assert [(msg.latitude, msg.longitude) for msg in received] == [
        (42, -71),
        (43, -72),
        (43, -72),
    ]
    assert received == [points[0], points[2], points[1]]
    publisher.reset_mock()
    assert replace(call, []).success
    assert read(call)["waypoints"] == []
    publisher.publish.assert_not_called()


def test_pop_advances_target_and_rejects_stale_edits(queue):
    call, received, executor, publisher = queue
    assert replace(call, [point("a"), point("b", 43, -72)]).success
    before = read(call)
    spin_until(executor, lambda: len(received) == 1)
    assert call("pop").success
    spin_until(executor, lambda: len(received) == 2)
    after = read(call)
    assert after["waypoints"] == [point("b", 43, -72)]
    assert before["version"] != after["version"]
    assert received[-1] == point("b", 43, -72)
    publisher.reset_mock()
    response = call("set", before["version"], before["waypoints"])
    assert not response.success
    assert response.error == "Queue changed; refresh before editing"
    assert read(call) == after
    assert call("pop").success
    assert read(call)["waypoints"] == []
    assert not call("pop").success
    publisher.publish.assert_not_called()


def test_rejected_requests_are_atomic(queue):
    call, _, _, publisher = queue
    assert replace(call, [point("original")]).success
    state = read(call)
    invalid_items = [
        [point("a"), point("a")],
        [point("")],
        [point("x" * 101)],
        [point("a", 91)],
        [point("a", float("nan"))],
        [point("a", float("inf"))],
        [point("a", longitude=181)],
        [point("a"), point("b", longitude=-181)],
    ]
    requests = [("set", state["version"], items) for items in invalid_items]
    requests += [("set", "", []), ("set", "stale", []), ("clear", "", [])]
    publisher.reset_mock()
    for command, version, items in requests:
        response = call(command, version, items)
        assert not response.success, (command, version, items)
        assert response.error
        assert {"version": response.version, "waypoints": response.waypoints} == state
        assert read(call) == state
        publisher.publish.assert_not_called()


def test_version_from_another_node_is_rejected(queue):
    call, _, _, _ = queue
    state = read(call)
    another = WaypointService()
    try:
        assert another.version != state["version"]
        with pytest.raises(ValueError, match="Queue changed"):
            another._set(state["version"], state["waypoints"])
    finally:
        another.destroy_node()
