"""ROS interfaces for the reinforcement learning sailing algorithm."""
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from interface.srv import Waypoint
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32, UInt8

from sailing.constants import (
    EARTH_RADIUS_METERS,
    PHYSICAL,
    RL_INFERENCE_PERIOD_SECONDS,
    RL_POLICY_PATH,
    RL_WIND_SPEED_METERS_PER_SECOND,
    WAYPOINT_REACHED_RADIUS_METERS,
)
from sailing.numpy_policy import NumpyPolicy

# Length of the observation run_inference() builds: WaypointEnv's 13 values plus
# the actual rudder and mainsail positions (include_actuator_state).
OBS_DIM = 15
# Exponential smoothing weights for the newest sample of the derived inputs.
VELOCITY_FILTER_ALPHA = 0.5
YAW_RATE_FILTER_ALPHA = 0.5


class ReinforcementLearning(Node):
    """Cache observations and periodically run the trained policy."""

    def __init__(self):
        super().__init__("reinforcement_learning")

        # None distinguishes missing observations from legitimate zero values.
        self.gps = None
        self.imu = None
        self.wind = None
        # No wind speed sensor yet: assume a constant apparent wind speed (m/s).
        self.wind_speed = RL_WIND_SPEED_METERS_PER_SECOND
        self.current_waypoint = None
        self.reported_mainsail_angle = None
        self.reported_rudder_angle = None
        self.reported_jib_angle = None
        self.reported_jib_side_flag = None

        self.mainsail_angle_pub = self.create_publisher(Int32, "mainsail_angle", 10)
        self.rudder_angle_pub = self.create_publisher(Int32, "rudder_angle", 10)
        self.jib_angle_pub = self.create_publisher(Int32, "jib_angle", 10)
        self.jib_side_flag_pub = self.create_publisher(UInt8, "jib_side_flag", 10)

        self.create_subscription(NavSatFix, "gps", self.gps_callback, 10)
        self.create_subscription(Vector3, "imu", self.imu_callback, 10)
        self.create_subscription(Int32, "wind", self.wind_callback, 10)
        self.create_subscription(
            NavSatFix, "current_waypoint", self.current_waypoint_callback, 10
        )
        self.create_subscription(
            Int32, "reported_mainsail_angle", self.reported_mainsail_angle_callback, 10
        )
        self.create_subscription(
            Int32, "reported_rudder_angle", self.reported_rudder_angle_callback, 10
        )
        self.create_subscription(
            Int32, "reported_jib_angle", self.reported_jib_angle_callback, 10
        )
        self.create_subscription(
            UInt8, "reported_jib_side_flag", self.reported_jib_side_flag_callback, 10
        )

        # Pops reached waypoints; the service then publishes the next current_waypoint.
        self.waypoint_client = self.create_client(Waypoint, "mutate_waypoint_queue")

        # Derived inputs the sensors don't provide directly (filled by gps/imu callbacks).
        self.gps_time = None
        self.imu_time = None
        self.velocity_enu = None  # (east, north) m/s
        self.yaw_rate = None  # rad/s, CCW-positive

        # Load the exported numpy policy (see sailbench scripts/export_policy_numpy.py).
        # The default looks in the installed package share dir; override with
        #   ros2 run sailing reinforcement_learning_node --ros-args -p policy_path:=/path/to/policy.npz
        default_policy_path = os.path.join(
            get_package_share_directory("sailing"), RL_POLICY_PATH
        )
        policy_path = self.declare_parameter("policy_path", default_policy_path).value
        self.policy = NumpyPolicy(policy_path)
        if self.policy.obs_dim != OBS_DIM:
            raise ValueError(
                f"{policy_path} expects {self.policy.obs_dim} observations, but "
                f"run_inference builds {OBS_DIM}."
            )
        # Training scales (dt, max_rudder_deg, speed_scale, ...) embedded in the .npz.
        self.cfg = self.policy.env_config
        # Feedforward MLP: no recurrent state, only the previous action is fed back.
        self.prev_action = np.zeros(2, dtype=np.float32)

        self.timer = self.create_timer(RL_INFERENCE_PERIOD_SECONDS, self.run_inference)
        self.get_logger().info(f"Reinforcement learning started with policy {policy_path}.")

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def gps_callback(self, msg):
        """Cache the latest GPS message and update the filtered ENU velocity."""
        now = self._now_seconds()
        if self.gps is not None and self.gps_time is not None and now > self.gps_time:
            dt = now - self.gps_time
            lat0 = np.radians(self.gps.latitude)
            displacement_enu = EARTH_RADIUS_METERS * np.array(
                [
                    np.radians(msg.longitude - self.gps.longitude) * np.cos(lat0),
                    np.radians(msg.latitude - self.gps.latitude),
                ]
            )
            velocity = displacement_enu / dt
            if self.velocity_enu is None:
                self.velocity_enu = velocity
            else:
                self.velocity_enu = (
                    VELOCITY_FILTER_ALPHA * velocity
                    + (1.0 - VELOCITY_FILTER_ALPHA) * self.velocity_enu
                )
        self.gps = msg
        self.gps_time = now

    def imu_callback(self, msg):
        """Cache roll, pitch, and yaw in degrees and update the filtered yaw rate."""
        now = self._now_seconds()
        if self.imu is not None and self.imu_time is not None and now > self.imu_time:
            # Compass heading is CW-positive; the sim's yaw rate is CCW-positive.
            heading_change_deg = (msg.z - self.imu.z + 180.0) % 360.0 - 180.0
            yaw_rate = -np.radians(heading_change_deg) / (now - self.imu_time)
            if self.yaw_rate is None:
                self.yaw_rate = yaw_rate
            else:
                self.yaw_rate = (
                    YAW_RATE_FILTER_ALPHA * yaw_rate
                    + (1.0 - YAW_RATE_FILTER_ALPHA) * self.yaw_rate
                )
        self.imu = msg
        self.imu_time = now

    def wind_callback(self, msg):
        """Cache relative wind direction in degrees."""
        self.wind = msg.data

    def current_waypoint_callback(self, msg):
        """Cache the current destination."""
        self.current_waypoint = msg

    def reported_mainsail_angle_callback(self, msg):
        self.reported_mainsail_angle = msg.data

    def reported_rudder_angle_callback(self, msg):
        self.reported_rudder_angle = msg.data

    def reported_jib_angle_callback(self, msg):
        self.reported_jib_angle = msg.data

    def reported_jib_side_flag_callback(self, msg):
        self.reported_jib_side_flag = msg.data

    def run_inference(self):
        """Wait for inputs, run the policy, and publish its decoded commands.

        Policy inputs, in WaypointEnv._get_observation order and scaling:
        - Waypoint offset, distance and direction in the boat frame (GPS + waypoint + heading).
        - Boat velocity (u, v) from differentiated GPS, and yaw rate from differentiated heading.
        - True wind direction and speed, reconstructed from apparent wind and boat velocity.
        - Previous action.
        - Actual rudder and mainsail positions reported by the Teensy.

        Angles are validated against the inclusive bounds in sailing.constants.PHYSICAL
        before publishing. Returns early without publishing while inputs are missing.
        """
        # Do nothing if any inputs have not yet arrived.
        if any(
            value is None
            for value in (
                self.gps,
                self.imu,
                self.wind,
                self.current_waypoint,
                self.reported_mainsail_angle,
                self.reported_rudder_angle,
                self.velocity_enu,
                self.yaw_rate,
            )
        ):
            return

        # Assemble the policy inputs in the sim's frames: world x=east, y=north;
        # boat frame x=forward, y=port; heading psi CCW from east.
        # Compass heading (CW from north) -> sim heading (CCW from east).
        psi = np.radians(90.0 - self.imu.z)
        # Rotates a world (east, north) vector into the boat frame.
        world_to_boat = np.array(
            [[np.cos(psi), np.sin(psi)], [-np.sin(psi), np.cos(psi)]]
        )

        # Current location and waypoint location -> waypoint offset in metres.
        lat0 = np.radians(self.gps.latitude)
        waypoint_enu = EARTH_RADIUS_METERS * np.array(
            [
                np.radians(self.current_waypoint.longitude - self.gps.longitude) * np.cos(lat0),
                np.radians(self.current_waypoint.latitude - self.gps.latitude),
            ]
        )
        waypoint_boat = world_to_boat @ waypoint_enu
        distance = max(np.linalg.norm(waypoint_boat), 1e-9)

        # Waypoint reached: advance the queue and wait for the next one.
        if distance < WAYPOINT_REACHED_RADIUS_METERS:
            self.pop_waypoint(distance)
            return

        # Boat speed: filtered GPS velocity (from gps_callback) in the boat frame.
        u, v = world_to_boat @ np.asarray(self.velocity_enu)

        # Relative wind is apparent and measured as "from, CW from bow"; the policy
        # was trained on true wind as a "blows to" direction. True = apparent + boat velocity.
        theta = np.radians(self.wind)
        apparent_wind = self.wind_speed * np.array([-np.cos(theta), np.sin(theta)])
        true_wind = apparent_wind + np.array([u, v])
        true_wind_speed = np.linalg.norm(true_wind)
        wind_dir_boat = true_wind / true_wind_speed if true_wind_speed > 1e-9 else np.zeros(2)

        # Actual surface positions, on the scales the actions use.
        # Verify the rudder sign on hardware matches the sim's positive angle.
        actual_rudder = self.reported_rudder_angle / self.cfg["max_rudder_deg"]
        actual_sheet = 2.0 * abs(self.reported_mainsail_angle) / self.cfg["max_sail_deg"] - 1.0

        # Same order and scaling as WaypointEnv._get_observation.
        radius = self.cfg["waypoint_max_radius_m"]
        obs = np.concatenate(
            [
                np.clip(waypoint_boat / radius, -1.0, 1.0),
                [np.clip(distance / radius, 0.0, 1.0)],
                waypoint_boat / distance,
                np.clip(np.array([u, v]) / self.cfg["speed_scale"], -1.0, 1.0),
                [np.clip(self.yaw_rate / self.cfg["yaw_rate_scale"], -1.0, 1.0)],
                wind_dir_boat,
                [np.clip(true_wind_speed / self.cfg["wind_speed_scale"], 0.0, 1.0)],
                self.prev_action,  # Previous action: fed back so the policy can smooth control.
                np.clip([actual_rudder, actual_sheet], -1.0, 1.0),
            ]
        ).astype(np.float32)

        action = self.policy.predict(obs)  # [rudder, sail], each in [-1, 1]
        self.prev_action = action

        # Decode the action exactly as WaypointEnv.step does.
        # Verify the rudder sign on hardware: positive here is the sim's positive angle.
        rudder_angle = int(np.rint(action[0] * self.cfg["max_rudder_deg"]))
        mainsail_angle = int(np.rint((action[1] + 1.0) / 2.0 * self.cfg["max_sail_deg"]))

        # Validate against the inclusive bounds in PHYSICAL.
        rudder_angle = int(
            np.clip(rudder_angle, PHYSICAL.RUDDER_MIN_ANGLE, PHYSICAL.RUDDER_MAX_ANGLE)
        )
        mainsail_angle = int(
            np.clip(mainsail_angle, PHYSICAL.MAINSAIL_MIN_ANGLE, PHYSICAL.MAINSAIL_MAX_ANGLE)
        )

        # The policy was trained without a jib: trim it like the main, on the leeward side.
        jib_angle = int(np.clip(mainsail_angle, PHYSICAL.JIB_MIN_ANGLE, PHYSICAL.JIB_MAX_ANGLE))
        wind_from_starboard = np.sin(theta) > 0.0
        jib_side_flag = PHYSICAL.JIB_SIDE_PORT if wind_from_starboard else PHYSICAL.JIB_SIDE_STB

        self.publish_command(mainsail_angle, rudder_angle, jib_angle, jib_side_flag)

    def pop_waypoint(self, distance):
        """Ask the waypoint service to remove the reached waypoint from the queue."""
        if not self.waypoint_client.service_is_ready():
            self.get_logger().warning(
                "Waypoint reached but mutate_waypoint_queue is unavailable; retrying.",
                throttle_duration_sec=5.0,
            )
            return
        self.get_logger().info(
            f"Reached waypoint ({self.current_waypoint.latitude}, "
            f"{self.current_waypoint.longitude}) at {distance:.1f} m; popping it."
        )
        # Cleared here rather than in the response callback: the service publishes the
        # next waypoint before it responds, so that message can arrive first. Clearing
        # also stops inference (and repeat pops) until the next waypoint arrives.
        self.current_waypoint = None
        future = self.waypoint_client.call_async(Waypoint.Request(command="pop"))
        future.add_done_callback(self.pop_waypoint_done)

    def pop_waypoint_done(self, future):
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            self.get_logger().warning(f"Waypoint pop failed: {message}")

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
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
