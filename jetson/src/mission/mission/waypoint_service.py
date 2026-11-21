import math
import threading
import uuid

import rclpy
from rclpy.node import Node
from interface.msg import Waypoint
from interface.srv import Waypoint as WaypointSrv
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class WaypointService(Node):
    def __init__(self):
        super().__init__("waypoint_service")

        self.lock = threading.Lock()

        self.service = self.create_service(
            WaypointSrv, "mutate_waypoint_queue", self.waypoint_service_callback
        )

        self.waypoint_publisher = self.create_publisher(
            Waypoint, "current_waypoint", 10
        )

        self.declare_parameter(
            "waypoints",
            [""],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description="Array of waypoint strings in 'latitude,longitude' format. Example: ['37.7749,-122.4194', '37.7749,-122.4195']",
            ),
        )
        waypoints_param = (
            self.get_parameter("waypoints").get_parameter_value().string_array_value
        )
        self.get_logger().info(f"Waypoints: {waypoints_param}")

        # Use a string-array sentinel for the default empty waypoint queue.
        self.waypoints = (
            self._parse_waypoints(waypoints_param) if waypoints_param != [""] else []
        )

        self.version = str(uuid.uuid4())

        self.publish_current_waypoint()

        self.get_logger().info("Waypoint service started")

    def _parse_waypoints(self, waypoint_strings):
        waypoints = []
        for s in waypoint_strings:
            latitude, longitude = map(float, s.split(","))
            waypoints.append(
                Waypoint(id=str(uuid.uuid4()), latitude=latitude, longitude=longitude)
            )
        return waypoints

    def publish_current_waypoint(self):
        """Publish the first queued waypoint, including its stable ID."""
        if not self.waypoints:
            self.get_logger().debug("No waypoints to publish")
            return

        msg = self.waypoints[0]
        self.waypoint_publisher.publish(msg)
        self.get_logger().info(
            f"Published current waypoint: Lat={msg.latitude}, Lon={msg.longitude}"
        )

    def _set(self, expected_version, items):
        if not expected_version:
            raise ValueError("A queue version is required")
        if expected_version != self.version:
            raise ValueError("Queue changed; refresh before editing")
        ids = set()
        waypoints = []
        for item in items:
            if not item.id or len(item.id) > 100 or item.id in ids:
                raise ValueError("Waypoint IDs must be nonempty and unique")
            for value, limit in ((item.latitude, 90), (item.longitude, 180)):
                if not math.isfinite(value) or abs(value) > limit:
                    raise ValueError("Invalid latitude or longitude")
            ids.add(item.id)
            waypoints.append(
                Waypoint(id=item.id, latitude=item.latitude, longitude=item.longitude)
            )
        # Validate everything before changing the queue or publishing a target.
        self.waypoints = waypoints
        self.version = str(uuid.uuid4())
        self.publish_current_waypoint()

    def waypoint_service_callback(self, request, response):
        with self.lock:
            if request.command == "get":
                response.success = True
            elif request.command == "set":
                try:
                    self._set(request.expected_version, request.waypoints)
                    response.success = True
                except (ValueError, TypeError, OverflowError) as error:
                    response.error = str(error)
                    response.success = False
            elif request.command == "pop":
                if self.waypoints:
                    self.waypoints.pop(0)
                    self.version = str(uuid.uuid4())
                    response.success = True
                    self.publish_current_waypoint()
                else:
                    response.error = "No waypoints to pop. The queue is empty."
                    response.success = False
            else:
                response.error = "Invalid command. Use 'get', 'set', or 'pop'."
                response.success = False
            response.version = self.version
            response.waypoints = list(self.waypoints)
            return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WaypointService()
        rclpy.spin(node)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
