"""
Buoy search: the waypoint queue, mode manager, buoy search and camera.

    ros2 launch bringup search.launch.py
    ros2 launch bringup search.launch.py config:=search_land_test.yaml
    ros2 launch bringup search.launch.py camera:=false

Sensor (/gps, /imu) and Teensy (wind_angle) nodes are not part of this launch yet.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("bringup"), "config", LaunchConfiguration("config")]
    )

    def node(package, executable, **kwargs):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            parameters=[config],
            output="screen",
            **kwargs,
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value="search.yaml",
                description="Parameter file in bringup/config",
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="true",
                description="Start the RealSense buoy detector",
            ),
            node("mission", "waypoint_service"),
            node("mission", "mode_manager"),
            node("mission", "buoy_search"),
            node(
                "vision",
                "buoy_detection",
                condition=IfCondition(LaunchConfiguration("camera")),
            ),
        ]
    )
