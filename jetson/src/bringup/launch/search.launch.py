"""
Buoy search: the waypoint queue, mode manager, buoy search and camera.

    ros2 launch bringup search.launch.py
    ros2 launch bringup search.launch.py config:=search_land_test.yaml
    ros2 launch bringup search.launch.py camera:=false

With lake_test:=true it also starts lake_test.launch.py (VectorNav, Teensy, waypoint
service, RL waypoint follower and rosbridge) and uses that waypoint service:

    ros2 launch bringup search.launch.py lake_test:=true config:=search_land_test.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("bringup"), "config", LaunchConfiguration("config")]
    )
    lake_test = LaunchConfiguration("lake_test")

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
            DeclareLaunchArgument(
                "lake_test",
                default_value="false",
                description="Also start lake_test.launch.py (sensors, Teensy, RL)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("bringup"), "launch", "lake_test.launch.py"]
                    )
                ),
                condition=IfCondition(lake_test),
            ),
            # lake_test.launch.py starts its own waypoint service
            node("mission", "waypoint_service", condition=UnlessCondition(lake_test)),
            node("mission", "mode_manager"),
            node("mission", "buoy_search"),
            node(
                "vision",
                "buoy_detection",
                condition=IfCondition(LaunchConfiguration("camera")),
            ),
        ]
    )
