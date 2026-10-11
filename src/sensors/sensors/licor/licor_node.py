"""Publish LI-COR apparent wind direction and speed over ROS 2."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

from .licor_real import LicorAnemometer


class LicorNode(Node):
    def __init__(self):
        super().__init__("licor")
        self.sensor = None
        try:
            for name, default in (
                ("timer_period", 0.5),
                ("licor_port", "/dev/ttyUSB0"),
                ("baud_rate", 115200),
                ("min_speed", 1.0),
                ("alpha", 0.3),
            ):
                self.declare_parameter(name, default)
            period = self.get_parameter("timer_period").value
            self.min_speed = self.get_parameter("min_speed").value
            self.alpha = self.get_parameter("alpha").value
            self.publisher = self.create_publisher(Int32, "wind_angle", 10)
            self.speed_publisher = self.create_publisher(Float32, "wind_speed", 10)
            self.last_direction = 0
            self.filtered_u = 0.0
            self.filtered_v = 1.0
            self.sensor = LicorAnemometer(
                self.get_parameter("licor_port").value,
                self.get_parameter("baud_rate").value,
            )
            self.timer = self.create_timer(period, self.publish_wind)
        except Exception:
            self.destroy_node()
            raise

    def publish_wind(self):
        sample = self.sensor.read_sample()
        if sample.speed >= self.min_speed:
            raw = math.radians(sample.direction)
            self.filtered_u += self.alpha * (math.sin(raw) - self.filtered_u)
            self.filtered_v += self.alpha * (math.cos(raw) - self.filtered_v)
            self.last_direction = int(
                math.degrees(math.atan2(self.filtered_u, self.filtered_v)) % 360
            )
        self.publisher.publish(Int32(data=self.last_direction))
        self.speed_publisher.publish(Float32(data=sample.speed))
        self.get_logger().info(
            f"Wind direction: {self.last_direction} (speed: {sample.speed:.2f} m/s)"
        )

    def destroy_node(self):
        try:
            if self.sensor is not None:
                self.sensor.close()
        finally:
            result = super().destroy_node()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LicorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
