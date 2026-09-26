"""ROS interfaces for the reinforcement learning sailing algorithm."""

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, Int32, UInt8

from sailing.constants import RL_INFERENCE_PERIOD_SECONDS


class ReinforcementLearning(Node):
    """Cache observations and periodically invoke the future policy integration."""

    def __init__(self):
        super().__init__("reinforcement_learning")

        # None distinguishes missing observations from legitimate zero values.
        self.gps = None
        self.imu = None
        self.wind = None
        self.wind_speed = None
        self.current_waypoint = None
        self.actual_mainsail_angle = None
        self.actual_rudder_angle = None
        self.actual_jib_angle = None
        self.actual_jib_side_flag = None

        self.mainsail_angle_pub = self.create_publisher(Int32, "mainsail_angle", 10)
        self.rudder_angle_pub = self.create_publisher(Int32, "rudder_angle", 10)
        self.jib_angle_pub = self.create_publisher(Int32, "jib_angle", 10)
        self.jib_side_flag_pub = self.create_publisher(UInt8, "jib_side_flag", 10)

        self.create_subscription(NavSatFix, "gps", self.gps_callback, 10)
        self.create_subscription(Vector3, "imu", self.imu_callback, 10)
        self.create_subscription(Int32, "wind", self.wind_callback, 10)
        self.create_subscription(Float32, "wind_speed", self.wind_speed_callback, 10)
        self.create_subscription(
            NavSatFix, "current_waypoint", self.current_waypoint_callback, 10
        )
        self.create_subscription(
            Int32, "actual_mainsail_angle", self.actual_mainsail_angle_callback, 10
        )
        self.create_subscription(
            Int32, "actual_rudder_angle", self.actual_rudder_angle_callback, 10
        )
        self.create_subscription(
            Int32, "actual_jib_angle", self.actual_jib_angle_callback, 10
        )
        self.create_subscription(
            UInt8, "actual_jib_side_flag", self.actual_jib_side_flag_callback, 10
        )

        # TODO: Load the trained policy and initialize any recurrent model
        # state. Also set RL_INFERENCE_PERIOD_SECONDS in constants.py.
        self.timer = self.create_timer(RL_INFERENCE_PERIOD_SECONDS, self.run_inference)
        self.get_logger().info(
            "Reinforcement learning scaffold started; inference is not implemented."
        )

    def gps_callback(self, msg):
        """Cache the latest GPS message."""
        self.gps = msg

    def imu_callback(self, msg):
        """Cache roll, pitch, and yaw in degrees."""
        self.imu = msg

    def wind_callback(self, msg):
        """Cache relative wind direction in degrees."""
        self.wind = msg.data

    def wind_speed_callback(self, msg):
        """Cache relative (apparent) wind speed in metres per second."""
        self.wind_speed = msg.data

    def current_waypoint_callback(self, msg):
        """Cache the current destination."""
        self.current_waypoint = msg

    def actual_mainsail_angle_callback(self, msg):
        self.actual_mainsail_angle = msg.data

    def actual_rudder_angle_callback(self, msg):
        self.actual_rudder_angle = msg.data

    def actual_jib_angle_callback(self, msg):
        self.actual_jib_angle = msg.data

    def actual_jib_side_flag_callback(self, msg):
        self.actual_jib_side_flag = msg.data

    def run_inference(self):
        """Wait for inputs, run the policy, and publish its decoded commands.

        TODO: Assemble the expected policy inputs:
        - Relative wind direction (degrees): self.wind.
        - Relative wind speed (metres per second): self.wind_speed.
        - Current location: self.gps.latitude and self.gps.longitude.
        - Waypoint location: self.current_waypoint.latitude and self.current_waypoint.longitude.
        - Heading (degrees): self.imu.z.
        - Time: not implemented
        - Boat speed: not implemented
        - Previous action: not implemented

        TODO: Preprocess these inputs and run the trained policy.

        TODO: Replace NotImplementedError with inference that produces
        mainsail_angle, rudder_angle, jib_angle, and jib_side_flag, then pass
        these outputs to publish_command() with:
        self.publish_command(mainsail_angle, rudder_angle, jib_angle, jib_side_flag).
        Validate integer angles in degrees against the inclusive bounds in
        sailing.constants.PHYSICAL before publishing:
        - mainsail_angle: 0 to 90 degrees (MAINSAIL_MIN_ANGLE/MAX_ANGLE).
        - rudder_angle: -45 to 45 degrees (RUDDER_MIN_ANGLE/MAX_ANGLE).
        - jib_angle: 0 to 90 degrees (JIB_MIN_ANGLE/MAX_ANGLE), independently
          controllable from the mainsail.
        - jib_side_flag: 0=port or 1=starboard (JIB_SIDE_PORT/JIB_SIDE_STB).
        Return early without calling publish_command() to skip a command.
        """
        # Do nothing if any inputs have not yet arrived.
        if any(
            value is None
            for value in (
                self.gps,
                self.imu,
                self.wind,
                self.wind_speed,
                self.current_waypoint,
            )
        ):
            return

        raise NotImplementedError


    def publish_command(self, mainsail_angle, rudder_angle, jib_angle, jib_side_flag):
        """Publish validated integer angles in degrees and the jib-side flag."""
        self.mainsail_angle_pub.publish(Int32(data=mainsail_angle))
        self.rudder_angle_pub.publish(Int32(data=rudder_angle))
        self.jib_angle_pub.publish(Int32(data=jib_angle))
        self.jib_side_flag_pub.publish(UInt8(data=jib_side_flag))


def main(args=None):
    """Run the controller and release ROS resources on exit."""
    rclpy.init(args=args)
    node = None
    try:
        node = ReinforcementLearning()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            # TODO: Add RL cleanup to node shutdown if needed.
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
