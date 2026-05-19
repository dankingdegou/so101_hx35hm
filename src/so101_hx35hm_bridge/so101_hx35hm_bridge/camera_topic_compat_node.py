#!/usr/bin/env python3
from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraTopicCompatNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_topic_compat")

        self.declare_parameter("input_image_topic", "/static_camera/color/image_raw")
        self.declare_parameter("input_camera_info_topic", "/static_camera/color/camera_info")
        self.declare_parameter("output_image_topic", "/static_camera/image_raw")
        self.declare_parameter("output_camera_info_topic", "/static_camera/camera_info")

        input_image_topic = str(self.get_parameter("input_image_topic").value)
        input_camera_info_topic = str(self.get_parameter("input_camera_info_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        output_camera_info_topic = str(self.get_parameter("output_camera_info_topic").value)

        self.image_pub = self.create_publisher(Image, output_image_topic, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, output_camera_info_topic, qos_profile_sensor_data)

        self.create_subscription(Image, input_image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, input_camera_info_topic, self.on_camera_info, qos_profile_sensor_data)

        self.get_logger().info(
            f"Camera topic compatibility relay: '{input_image_topic}' -> '{output_image_topic}', "
            f"'{input_camera_info_topic}' -> '{output_camera_info_topic}'"
        )

    def on_image(self, msg: Image) -> None:
        self.image_pub.publish(deepcopy(msg))

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.info_pub.publish(deepcopy(msg))


def main() -> None:
    rclpy.init()
    node = CameraTopicCompatNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
