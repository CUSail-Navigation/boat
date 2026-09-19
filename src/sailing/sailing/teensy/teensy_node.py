import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32, UInt8

from sailing.constants import PHYSICAL, TEENSY

from . import serial_port


class Teensy(Node):
    """
    ROS 2 Node that interfaces with a Teensy microcontroller to control and
    monitor the boat's hardware (mainsail, rudder, and jib). It communicates
    with the Teensy, reads telemetry (wind, angles, dropped packets), and sends
    control commands from ROS subscriptions.

    Command topics (namespace-relative): ``mainsail_angle`` (Int32), ``rudder_angle``
    (Int32), ``jib_angle`` (Int32), ``jib_side_flag`` (UInt8: 0 = port side,
    1 = starboard side).
    """

    def __init__(self):
        super().__init__("teensy")

        # Declare parameters.
        self.declare_parameter("teensy_port", TEENSY.DEFAULT_PORT)
        self.declare_parameter(
            "rx_period", TEENSY.DEFAULT_TELEMETRY_POLL_PERIOD_SECONDS
        )

        # Read parameters.
        self.telemetry_poll_period = float(self.get_parameter("rx_period").value)
        self.teensy_port = self.get_parameter("teensy_port").value

        if self.telemetry_poll_period <= 0:
            raise ValueError("Telemetry polling period must be positive.")

        # Initialize the hardware driver.
        try:
            self.serial_port = serial_port.SerialPort(self.teensy_port)
        except Exception as exc:
            self.get_logger().error(
                f"Serial connection failed: port={self.teensy_port} reason={exc}"
            )
            raise
        self.get_logger().info(
            f"Serial connection opened: port={self.teensy_port} "
            f"poll_period={self.telemetry_poll_period}s"
        )

        # Initialize desired actuator positions.
        self.desired_mainsail_angle = TEENSY.INITIAL_MAINSAIL_ANGLE
        self.desired_rudder_angle = TEENSY.INITIAL_RUDDER_ANGLE
        self.desired_jib_angle = TEENSY.INITIAL_JIB_ANGLE
        self.desired_jib_side_flag = PHYSICAL.JIB_SIDE_PORT

        # Create telemetry publishers.
        self.actual_mainsail_angle_pub = self.create_publisher(
            Int32, "actual_mainsail_angle", 10
        )
        self.actual_rudder_angle_pub = self.create_publisher(
            Int32, "actual_rudder_angle", 10
        )
        self.actual_jib_angle_pub = self.create_publisher(Int32, "actual_jib_angle", 10)
        self.actual_jib_side_flag_pub = self.create_publisher(
            UInt8, "actual_jib_side_flag", 10
        )
        self.dropped_packets_pub = self.create_publisher(Int32, "dropped_packets", 10)

        # Poll telemetry periodically.
        self.timer = self.create_timer(self.telemetry_poll_period, self.check_telemetry)

        # Subscribe to actuator commands.
        self.create_subscription(
            Int32, "mainsail_angle", self.mainsail_angle_callback, 10
        )
        self.create_subscription(Int32, "rudder_angle", self.rudder_angle_callback, 10)
        self.create_subscription(Int32, "jib_angle", self.jib_angle_callback, 10)
        self.create_subscription(
            UInt8, "jib_side_flag", self.jib_side_flag_callback, 10
        )

    def mainsail_angle_callback(self, msg):
        """Update the mainsail goal and send the full command."""
        self.desired_mainsail_angle = msg.data
        self._send_command_to_teensy()

    def rudder_angle_callback(self, msg):
        """Update the rudder goal and send the full command."""
        self.desired_rudder_angle = msg.data
        self._send_command_to_teensy()

    def jib_angle_callback(self, msg):
        """Update the jib goal and send the full command."""
        self.desired_jib_angle = msg.data
        self._send_command_to_teensy()

    def jib_side_flag_callback(self, msg):
        """Update the jib side and send the full command."""
        self.desired_jib_side_flag = int(msg.data)
        self._send_command_to_teensy()

    def check_telemetry(self):
        """Read and publish telemetry from the Teensy."""
        data = {}
        try:
            received = self.serial_port.read_telemetry(data) == 0
        except OSError as exc:
            self.get_logger().warning(
                f"Telemetry read failed: reason={exc}",
                throttle_duration_sec=TEENSY.WARNING_THROTTLE_SECONDS,
            )
            return

        if received:
            mainsail_angle_msg = Int32()
            mainsail_angle_msg.data = data["mainsail_angle"]
            self.actual_mainsail_angle_pub.publish(mainsail_angle_msg)

            rudder_angle_msg = Int32()
            rudder_angle_msg.data = data["rudder_angle"]
            self.actual_rudder_angle_pub.publish(rudder_angle_msg)

            jib_angle_msg = Int32()
            jib_angle_msg.data = data["jib_angle"]
            self.actual_jib_angle_pub.publish(jib_angle_msg)

            jib_side_flag_msg = UInt8()
            jib_side_flag_msg.data = data["jib_side_flag"]
            self.actual_jib_side_flag_pub.publish(jib_side_flag_msg)

            dropped_packets_msg = Int32()
            dropped_packets_msg.data = data["dropped_packets"]
            self.dropped_packets_pub.publish(dropped_packets_msg)

            side = (
                "port"
                if data["jib_side_flag"] == PHYSICAL.JIB_SIDE_PORT
                else "starboard"
            )
            self.get_logger().debug(
                f"RX mainsail={data['mainsail_angle']} rudder={data['rudder_angle']} "
                f"jib={data['jib_angle']} jib_side={side} "
                f"dropped_packets={data['dropped_packets']}"
            )
        else:
            self.get_logger().debug("No telemetry received.")

    def _send_command_to_teensy(self):
        """Send the latest mainsail, rudder, and jib goals in one packet."""
        side = (
            "port"
            if self.desired_jib_side_flag == PHYSICAL.JIB_SIDE_PORT
            else "starboard"
        )
        command = (
            f"mainsail={self.desired_mainsail_angle} rudder={self.desired_rudder_angle} "
            f"jib={self.desired_jib_angle} jib_side={side}"
        )
        try:
            result = self.serial_port.send_command(
                self.desired_mainsail_angle,
                self.desired_rudder_angle,
                self.desired_jib_angle,
                self.desired_jib_side_flag,
            )
        except (OSError, ValueError) as exc:
            reason = str(exc)
        else:
            if result == 0:
                self.get_logger().debug(f"TX {command}")
                return
            reason = f"driver returned status {result}"
        self.get_logger().warning(
            f"Command failed: {command} reason={reason}",
            throttle_duration_sec=TEENSY.WARNING_THROTTLE_SECONDS,
        )

    def destroy_node(self):
        """Close the serial connection and release ROS resources."""
        try:
            self.serial_port.close()
        finally:
            result = super().destroy_node()
        return result


def main(args=None):
    """Run the Teensy node and release resources on exit."""
    rclpy.init(args=args)
    teensy_node = Teensy()

    try:
        rclpy.spin(teensy_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        teensy_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
