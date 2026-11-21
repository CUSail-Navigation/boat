"""
Detect buoys with YOLO on the RealSense color stream and publish each detection with
its position from the aligned depth stream. Frames are only processed in search mode.

Needs pyrealsense2, torch and ultralytics, which are installed on the Jetson only.
"""

import math
import os
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from geometry_msgs.msg import Point
from interface.msg import BuoyDetection
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, String

from vision.buoy_detection import YOLOBuoyDetector
from vision.depth import box_depth, box_width

MODEL_FILE = "buoy2.pt"


def default_model_path():
    """The model installed with the package, else the copy next to this file."""
    try:
        path = os.path.join(get_package_share_directory("vision"), MODEL_FILE)
        if os.path.exists(path):
            return path
    except PackageNotFoundError:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), MODEL_FILE)


class BuoyDetectorNode(Node):
    def __init__(self):
        super().__init__("buoy_detection")

        params = {
            "model_path": "",  # empty = default_model_path()
            "detection_threshold": 0.85,
            "timer_period": 0.1,  # s between processed frames
            "show_frames": False,
            "frame_timeout_ms": 1000,
            "min_depth": 0.3,  # m, RealSense minimum depth
            "max_depth": 25.0,  # m, stereo depth is too noisy past this
            "depth_roi_fraction": 0.5,  # central fraction of the box sampled for depth
            "min_buoy_width": 0.2,  # m
            "max_buoy_width": 2.0,  # m
            "save_frames": False,
            "save_dir": os.path.expanduser("~/boat_images"),
            "save_every_n_frames": 10,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in params}
        self.frame_count = 0
        if self.p["save_frames"]:
            os.makedirs(self.p["save_dir"], exist_ok=True)

        self.create_subscription(
            Float32, "update_detection_threshold", self.detection_threshold_callback, 10
        )
        self.detection_publisher = self.create_publisher(
            BuoyDetection, "/buoy_detection", 10
        )
        # Unstamped position and distance, kept for the webserver.
        self.position_publisher = self.create_publisher(Point, "/buoy_position", 10)
        self.distance_publisher = self.create_publisher(Float32, "/buoy_distance", 10)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)

        model_path = self.p["model_path"] or default_model_path()
        self.get_logger().info(f"Loading buoy model from {model_path}")
        self.detector = YOLOBuoyDetector(
            model_path, self.p["detection_threshold"], self.p["show_frames"]
        )

        self.current_mode = "manual"
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "current_mode", self.mode_callback, latched)

        self.timer = self.create_timer(self.p["timer_period"], self.process_frame)
        self.get_logger().info("Buoy detector started")

    def process_frame(self):
        """Look for a buoy in the latest frame and publish its position if found."""
        if self.current_mode != "search":
            return

        success, frames = self.pipeline.try_wait_for_frames(self.p["frame_timeout_ms"])
        if not success:
            self.get_logger().warn(
                "No frames from the camera; retrying.", throttle_duration_sec=5.0
            )
            return
        stamp = self.get_clock().now().to_msg()

        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return
        color_image = np.asanyarray(color_frame.get_data())
        self.maybe_save(color_image)

        detection, _ = self.detector.process_frame(color_image)
        if detection is None:
            return

        depth = box_depth(
            np.asanyarray(depth_frame.get_data()),
            detection["box"],
            self.depth_scale,
            self.p["depth_roi_fraction"],
            self.p["min_depth"],
            self.p["max_depth"],
        )
        if depth is None:
            self.get_logger().info(
                "Buoy detected but no valid depth; dropping.", throttle_duration_sec=2.0
            )
            return

        intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        width = box_width(detection["box"], depth, intrinsics.fx)
        if not self.p["min_buoy_width"] <= width <= self.p["max_buoy_width"]:
            self.get_logger().info(
                f"Detection is {width:.2f} m wide, not buoy-sized; dropping.",
                throttle_duration_sec=2.0,
            )
            return

        # RealSense camera frame is x right, y down, z forward; publish x right, y forward, z down.
        point = Point()
        point.x, point.z, point.y = rs.rs2_deproject_pixel_to_point(
            intrinsics, list(detection["center"]), depth
        )

        msg = BuoyDetection()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera"
        msg.position = point
        msg.confidence = detection["confidence"]
        msg.width_m = float(width)
        self.detection_publisher.publish(msg)
        self.position_publisher.publish(point)
        self.distance_publisher.publish(Float32(data=math.hypot(point.x, point.y)))
        self.get_logger().info(
            f"Buoy at x={point.x:.1f} m, y={point.y:.1f} m, "
            f"confidence {detection['confidence']:.2f}"
        )

    def maybe_save(self, image):
        self.frame_count += 1
        if (
            not self.p["save_frames"]
            or self.frame_count % self.p["save_every_n_frames"]
        ):
            return
        name = f"image_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.jpg"
        if not cv2.imwrite(os.path.join(self.p["save_dir"], name), image):
            self.get_logger().warn(
                f"Failed to save frame to {self.p['save_dir']}",
                throttle_duration_sec=30.0,
            )

    def detection_threshold_callback(self, msg):
        if not 0.0 < msg.data < 1.0:
            self.get_logger().warn(
                f"Detection threshold must be between 0 and 1, got {msg.data}"
            )
            return
        self.detector.update_parameters(msg.data)
        self.get_logger().info(f"Detection threshold set to {msg.data}")

    def mode_callback(self, msg):
        if msg.data != self.current_mode:
            self.get_logger().info(f"Mode changed to '{msg.data}'.")
        self.current_mode = msg.data

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BuoyDetectorNode()
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
