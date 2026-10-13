"""Publish GPS and roll/pitch/heading in degrees (east=0, counter-clockwise positive)."""

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from .vectornav_real import SailVectorNav


class VectorNav(Node):
    def __init__(self):
        super().__init__("vectornav")
        self.sensor = None
        try:
            for name, default in (
                ("timer_period", 0.5),
                (
                    "vectornav_port",
                    "/dev/serial/by-id/usb-FTDI_USB-RS232-WE_AV0LF3XT-if00-port0",
                ),
            ):
                self.declare_parameter(name, default)
            period = self.get_parameter("timer_period").value
            self.publisher_gps = self.create_publisher(NavSatFix, "gps", 10)
            self.publisher_imu = self.create_publisher(Vector3, "imu", 10)
            self.sensor = SailVectorNav(self.get_parameter("vectornav_port").value)
            self.timer = self.create_timer(period, self.gps_callback)
        except Exception:
            self.destroy_node()
            raise

    def gps_callback(self):
        sample = self.sensor.read_sample()
        gps = NavSatFix()
        gps.latitude = sample.latitude
        gps.longitude = sample.longitude
        gps.altitude = 0.0  # Preserve sailbot's default; altitude is not decoded.
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.publisher_gps.publish(gps)
        self.publisher_imu.publish(Vector3(x=sample.roll, y=sample.pitch, z=sample.yaw))
        self.get_logger().info(
            f"Publishing GPS Data: Lat: {sample.latitude} Long: {sample.longitude}"
        )
        self.get_logger().info(
            f"Publishing IMU Data (Yaw, Pitch, Roll) = ({sample.yaw}, {sample.pitch}, {sample.roll})"
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
        node = VectorNav()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
