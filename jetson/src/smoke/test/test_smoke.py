"""Check local ROS topic communication, failing after 10 seconds."""

import time
import uuid

import rclpy
from std_msgs.msg import String


def test_ros_message_exchange():
    rclpy.init()
    node = rclpy.create_node("smoke")
    topic = f"smoke/check_{uuid.uuid4().hex}"
    received = False

    def on_message(message):
        nonlocal received
        received = message.data == "hello"

    publisher = node.create_publisher(String, topic, 10)
    node.create_subscription(String, topic, on_message, 10)

    try:
        deadline = time.monotonic() + 10
        while rclpy.ok() and not received and time.monotonic() < deadline:
            publisher.publish(String(data="hello"))
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    assert received, "No matching ROS message received within 10 seconds."
