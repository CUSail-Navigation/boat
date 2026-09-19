import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from interface.srv import Waypoint
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class WaypointService(Node):
    def __init__(self):
        super().__init__("waypoint_service")

        self.lock = threading.Lock()

        self.service = self.create_service(
            Waypoint, "mutate_waypoint_queue", self.waypoint_service_callback
        )

        self.waypoint_publisher = self.create_publisher(
            NavSatFix, "current_waypoint", 10
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

        self.publish_current_waypoint()

        self.get_logger().info("Waypoint service started")

    def _parse_waypoints(self, waypoint_strings):
        waypoints = []
        for s in waypoint_strings:
            latitude, longitude = map(float, s.split(","))
            waypoints.append((latitude, longitude))
        return waypoints

    def publish_current_waypoint(self):
        """
        Publish the current waypoint (front of queue) as a NavSatFix message.
        Assumes coordinates are in latitude, longitude format.
        """
        if not self.waypoints:
            self.get_logger().debug("No waypoints to publish")
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.latitude = self.waypoints[0][0]
        msg.longitude = self.waypoints[0][1]

        msg.altitude = 0.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.waypoint_publisher.publish(msg)
        self.get_logger().info(
            f"Published current waypoint: Lat={msg.latitude}, Lon={msg.longitude}"
        )

    def waypoint_service_callback(self, request, response):
        with self.lock:
            self.get_logger().debug(
                f"Received request: command={request.command}, argument={request.argument}"
            )

            if request.command == "get":
                response.message = str(self.waypoints)
                response.success = True

            elif request.command == "set":
                try:
                    self.waypoints = self._parse_waypoints(request.argument.split(";"))
                    response.message = "Waypoint queue successfully updated"
                    response.success = True
                    self.get_logger().info(f"New waypoint queue: {self.waypoints}")

                    self.publish_current_waypoint()
                except Exception as e:
                    response.message = f"Failed to update waypoints: {str(e)}"
                    response.success = False
                    self.get_logger().error(f"Error parsing new waypoints: {str(e)}")

            elif request.command == "pop":
                if self.waypoints:
                    popped_waypoint = self.waypoints.pop(0)
                    response.message = (
                        f"Waypoint {popped_waypoint} removed successfully."
                    )
                    response.success = True
                    self.get_logger().info(f"Popped waypoint: {popped_waypoint}")

                    self.publish_current_waypoint()
                else:
                    response.message = "No waypoints to pop. The queue is empty."
                    response.success = False
                    self.get_logger().warning(
                        "Attempted to pop from an empty waypoint queue."
                    )

            else:
                response.message = "Invalid command. Use 'get', 'set', or 'pop'."
                response.success = False
                self.get_logger().error("Invalid command received")

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
