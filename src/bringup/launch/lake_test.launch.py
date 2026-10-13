"""Start the lake-test nodes using their existing hardware defaults."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="sensors", executable="vectornav_node", output="screen"),
            Node(package="sensors", executable="licor_node", output="screen"),
            Node(package="sailing", executable="teensy_node", output="screen"),
            Node(package="mission", executable="waypoint_service", output="screen"),
            Node(
                package="sailing",
                executable="reinforcement_learning_node",
                output="screen",
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                prefix="/usr/bin/python3",
                parameters=[{"port": 9090}],
                output="screen",
            ),
        ]
    )
