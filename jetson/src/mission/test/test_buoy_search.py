"""The buoy search, mode manager and waypoint service working together over ROS."""

import time
import uuid

import pytest
import rclpy
from geometry_msgs.msg import Vector3
from interface.srv import SetModeWithParams
from interface.srv import Waypoint as WaypointSrv
from mission.buoy_search_node import BuoySearch
from mission.mode_manager import ModeManager
from mission.waypoint_service import WaypointService
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

CENTER = (42.444, -76.483)


@pytest.fixture
def boat():
    namespace = "/test_" + uuid.uuid4().hex
    rclpy.init(
        args=[
            "--ros-args",
            "-r",
            f"__ns:={namespace}",
            # Keep the fake sensors on this test's namespace too
            "-r",
            f"/gps:={namespace}/gps",
            "-r",
            f"/imu:={namespace}/imu",
            "-p",
            f"search_center_lat:={CENTER[0]}",
            "-p",
            f"search_center_lon:={CENTER[1]}",
            "-p",
            "search_radius:=40.0",
        ]
    )
    nodes = []
    executor = SingleThreadedExecutor()
    try:
        nodes = [WaypointService(), ModeManager(), BuoySearch(), Node("test_peer")]
        for node in nodes:
            executor.add_node(node)
        search, peer = nodes[2], nodes[3]
        gps_pub = peer.create_publisher(NavSatFix, "gps", 10)
        imu_pub = peer.create_publisher(Vector3, "imu", 10)
        queue = peer.create_client(WaypointSrv, "mutate_waypoint_queue")
        set_mode = peer.create_client(SetModeWithParams, "set_mode")
        assert queue.wait_for_service(timeout_sec=10)
        assert set_mode.wait_for_service(timeout_sec=10)

        def spin_until(predicate, timeout=10.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                # Boat sitting south-west of the area, pointing north
                gps_pub.publish(
                    NavSatFix(latitude=CENTER[0] - 0.0005, longitude=CENTER[1] - 0.0005)
                )
                imu_pub.publish(Vector3(z=90.0))
                executor.spin_once(timeout_sec=0.05)
            assert predicate(), "Timed out waiting for ROS"

        def call(client, request):
            future = client.call_async(request)
            spin_until(future.done)
            return future.result()

        yield search, queue, set_mode, spin_until, call
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


def queued(call, queue):
    return call(queue, WaypointSrv.Request(command="get")).waypoints


def test_search_mode_sends_a_leg_to_the_waypoint_queue(boat):
    search, queue, set_mode, spin_until, call = boat
    response = call(set_mode, SetModeWithParams.Request(mode="search"))
    assert response.success

    spin_until(lambda: search.state.value == "searching")
    spin_until(lambda: len(queued(call, queue)) == 2)
    waypoints = queued(call, queue)
    assert all(w.id.startswith("search-") for w in waypoints)
    # The leg lies within the 40 m search area (~0.0005 deg)
    for w in waypoints:
        assert abs(w.latitude - CENTER[0]) < 0.0006
        assert abs(w.longitude - CENTER[1]) < 0.0008


def test_set_rejected_by_queue_change_is_resent(boat):
    search, queue, set_mode, spin_until, call = boat
    call(set_mode, SetModeWithParams.Request(mode="search"))
    spin_until(lambda: len(queued(call, queue)) == 2)

    # Someone else changes the queue, so the search's version is stale
    assert call(queue, WaypointSrv.Request(command="pop")).success
    search.set_waypoints([(CENTER[0], CENTER[1])])

    def resent():
        waypoints = queued(call, queue)
        return len(waypoints) == 1 and waypoints[0].latitude == pytest.approx(CENTER[0])

    spin_until(resent)


def test_radius_only_keeps_configured_center(boat):
    search, queue, set_mode, spin_until, call = boat
    request = SetModeWithParams.Request(mode="search", search_radius=25.0)
    assert call(set_mode, request).success
    spin_until(lambda: search.search_radius == 25.0)
    spin_until(lambda: len(queued(call, queue)) == 2)
    for w in queued(call, queue):
        assert abs(w.latitude - CENTER[0]) < 0.0004  # within 25 m of the center


def test_invalid_mode_is_rejected(boat):
    _, _, set_mode, _, call = boat
    assert not call(set_mode, SetModeWithParams.Request(mode="sail_fast")).success


def test_plain_search_drops_previous_custom_area(boat):
    search, _, set_mode, spin_until, call = boat
    call(set_mode, SetModeWithParams.Request(mode="search", search_radius=25.0))
    spin_until(lambda: search.search_radius == 25.0)
    call(set_mode, SetModeWithParams.Request(mode="manual"))
    call(set_mode, SetModeWithParams.Request(mode="search"))
    spin_until(lambda: search.search_radius == 40.0)  # the configured radius
