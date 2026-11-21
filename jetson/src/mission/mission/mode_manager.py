"""
Owns the boat's mission mode. The set_mode service switches modes and publishes the mode
and its parameters on latched topics, so nodes that start or restart later still get them.
"""

import rclpy
from geometry_msgs.msg import Point
from interface.srv import SetModeWithParams
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

ALLOWED_MODES = ("manual", "search")
LATCHED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ModeManager(Node):
    def __init__(self):
        super().__init__("mode_manager")
        self.current_mode = "manual"

        self.mode_pub = self.create_publisher(String, "current_mode", LATCHED_QOS)
        # Search area: x = lon, y = lat, z = radius in m (0 = the search node's default)
        self.search_center_pub = self.create_publisher(
            Point, "search_center_point", LATCHED_QOS
        )
        self.create_service(SetModeWithParams, "set_mode", self.set_mode_callback)

        self.publish_mode()
        self.get_logger().info(f"Mode manager started in '{self.current_mode}' mode")

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.current_mode))

    def set_mode_callback(self, request, response):
        mode = request.mode.lower()
        if mode not in ALLOWED_MODES:
            response.success = False
            response.message = f"Invalid mode '{mode}'. Allowed: {list(ALLOWED_MODES)}"
            self.get_logger().warn(response.message)
            return response

        if mode == "search":
            center = request.search_center_point
            radius = max(request.search_radius, 0.0)
            # Always published, so a search without an area drops the previous one's.
            # Zeros mean the search node's configured center and radius. Published
            # before the mode, so the search plans around it from the start.
            self.search_center_pub.publish(Point(x=center.x, y=center.y, z=radius))
            given = center.x != 0.0 or center.y != 0.0
            where = f"({center.y}, {center.x})" if given else "default center"
            self.get_logger().info(
                f"Search mode: {where}, radius {radius or 'default'}"
            )

        self.current_mode = mode
        self.publish_mode()
        response.success = True
        response.message = f"Switched to '{mode}' mode."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ModeManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
