#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # noqa: F401


DICT_MAP = {
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector")

        self.declare_parameter("image_topic", "/static_camera/image_raw")
        self.declare_parameter("camera_info_topic", "/static_camera/camera_info")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("marker_id", 0)  # -1 means accept any detected id
        self.declare_parameter("marker_size_m", 0.02)
        self.declare_parameter("dictionary", "DICT_ARUCO_ORIGINAL")
        self.declare_parameter("auto_dictionary", True)
        self.declare_parameter("output_ns", "/vision/aruco")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.dictionary_name = str(self.get_parameter("dictionary").value)
        self.auto_dictionary = bool(self.get_parameter("auto_dictionary").value)
        self.output_ns = str(self.get_parameter("output_ns").value).rstrip("/")

        dict_id = DICT_MAP.get(self.dictionary_name, cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.dictionary = cv2.aruco.Dictionary_get(dict_id)
        self.dict_objs = {name: cv2.aruco.Dictionary_get(v) for name, v in DICT_MAP.items()}
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.latest_image: Optional[Image] = None
        self.latest_info: Optional[CameraInfo] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_cam_pub = self.create_publisher(PoseStamped, f"{self.output_ns}/pose_camera", 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, f"{self.output_ns}/pose_base", 10)
        self.debug_pub = self.create_publisher(Image, f"{self.output_ns}/debug_image", 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, f"{self.output_ns}/detected_ids", 10)

        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self.process)
        self.get_logger().info(
            f"Aruco detector started: image='{self.image_topic}', info='{self.camera_info_topic}', "
            f"dict='{self.dictionary_name}', marker_id={self.marker_id}, size={self.marker_size_m}m"
        )

    def on_image(self, msg: Image) -> None:
        self.latest_image = msg

    def on_info(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    def process(self) -> None:
        try:
            self._process_once()
        except Exception as exc:
            self.get_logger().error(f"Aruco process error (ignored, continue running): {exc}")

    def _process_once(self) -> None:
        if self.latest_image is None or self.latest_info is None:
            return

        bgr = self.to_bgr(self.latest_image)
        if bgr is None:
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        used_dict_name = self.dictionary_name
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params
        )
        if (ids is None or len(ids) == 0) and self.auto_dictionary:
            for name, d in self.dict_objs.items():
                c2, i2, r2 = cv2.aruco.detectMarkers(gray, d, parameters=self.detector_params)
                if i2 is not None and len(i2) > 0:
                    corners, ids, _rejected = c2, i2, r2
                    used_dict_name = name
                    break
        debug = bgr.copy()
        if ids is None or len(ids) == 0:
            cv2.putText(
                debug,
                "No markers detected",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            self.publish_debug(debug, self.latest_image.header.frame_id)
            return

        cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        ids_flat = ids.flatten().tolist()
        ids_msg = Int32MultiArray()
        ids_msg.data = [int(v) for v in ids_flat]
        self.ids_pub.publish(ids_msg)
        cv2.putText(
            debug,
            f"Detected IDs: {ids_flat} dict={used_dict_name}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        target_id = self.marker_id if self.marker_id >= 0 else ids_flat[0]
        if target_id not in ids_flat:
            cv2.putText(
                debug,
                f"Target ID {self.marker_id} not found",
                (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            self.publish_debug(debug, self.latest_image.header.frame_id)
            return

        idx = ids_flat.index(target_id)
        c = [corners[idx]]

        k = np.array(self.latest_info.k, dtype=np.float64).reshape(3, 3)
        d = np.array(self.latest_info.d, dtype=np.float64)
        rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(c, self.marker_size_m, k, d)
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        cv2.drawFrameAxes(debug, k, d, rvec, tvec, self.marker_size_m * 0.6)
        self.publish_debug(debug, self.latest_image.header.frame_id)

        pose_cam = self.build_pose_from_rt(rvec, tvec, self.latest_image.header.frame_id)
        self.pose_cam_pub.publish(pose_cam)

        try:
            pose_base = self.tf_buffer.transform(
                pose_cam,
                self.target_frame,
                timeout=Duration(seconds=0.05),
            )
            self.pose_base_pub.publish(pose_base)
        except TransformException:
            pass

    def build_pose_from_rt(self, rvec: np.ndarray, tvec: np.ndarray, frame_id: str) -> PoseStamped:
        rot, _ = cv2.Rodrigues(rvec)
        qw = float(np.sqrt(max(0.0, 1.0 + rot[0, 0] + rot[1, 1] + rot[2, 2])) / 2.0)
        qx = float((rot[2, 1] - rot[1, 2]) / (4.0 * qw + 1e-12))
        qy = float((rot[0, 2] - rot[2, 0]) / (4.0 * qw + 1e-12))
        qz = float((rot[1, 0] - rot[0, 1]) / (4.0 * qw + 1e-12))

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(tvec[0])
        msg.pose.position.y = float(tvec[1])
        msg.pose.position.z = float(tvec[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def publish_debug(self, bgr: np.ndarray, frame_id: str) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = int(bgr.shape[0])
        msg.width = int(bgr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(bgr.shape[1] * 3)
        msg.data = bgr.tobytes()
        self.debug_pub.publish(msg)

    def to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height <= 0 or msg.width <= 0:
            return None
        if msg.encoding not in ("rgb8", "bgr8"):
            return None
        row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        image = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image


def main() -> None:
    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
