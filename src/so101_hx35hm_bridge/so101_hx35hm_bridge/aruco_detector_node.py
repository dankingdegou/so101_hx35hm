#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray, String
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
        self.declare_parameter("max_message_age_s", 0.25)
        self.declare_parameter("min_marker_distance_m", 0.05)
        self.declare_parameter("max_marker_distance_m", 1.50)
        self.declare_parameter("smoothing_window", 5)
        self.declare_parameter("min_stable_samples", 3)
        self.declare_parameter("max_position_jump_m", 0.10)
        self.declare_parameter("output_ns", "/vision/aruco")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.marker_id = int(self.get_parameter("marker_id").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.dictionary_name = str(self.get_parameter("dictionary").value)
        self.auto_dictionary = bool(self.get_parameter("auto_dictionary").value)
        self.max_message_age_s = max(0.0, float(self.get_parameter("max_message_age_s").value))
        self.min_marker_distance_m = max(0.0, float(self.get_parameter("min_marker_distance_m").value))
        self.max_marker_distance_m = max(0.0, float(self.get_parameter("max_marker_distance_m").value))
        self.smoothing_window = max(1, int(self.get_parameter("smoothing_window").value))
        self.min_stable_samples = max(1, int(self.get_parameter("min_stable_samples").value))
        self.max_position_jump_m = max(0.0, float(self.get_parameter("max_position_jump_m").value))
        self.output_ns = str(self.get_parameter("output_ns").value).rstrip("/")

        dict_id = DICT_MAP.get(self.dictionary_name, cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.dictionary = cv2.aruco.Dictionary_get(dict_id)
        self.dict_objs = {name: cv2.aruco.Dictionary_get(v) for name, v in DICT_MAP.items()}
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.latest_image: Optional[Image] = None
        self.latest_info: Optional[CameraInfo] = None
        self.pose_history: deque[np.ndarray] = deque(maxlen=self.smoothing_window)
        self._last_status_text = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_cam_pub = self.create_publisher(PoseStamped, f"{self.output_ns}/pose_camera", 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, f"{self.output_ns}/pose_base", 10)
        self.debug_pub = self.create_publisher(Image, f"{self.output_ns}/debug_image", 10)
        self.ids_pub = self.create_publisher(Int32MultiArray, f"{self.output_ns}/detected_ids", 10)
        self.status_pub = self.create_publisher(String, f"{self.output_ns}/status", 10)

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
        debug_lines: List[Tuple[str, Tuple[int, int, int]]] = []
        if self.latest_image is None or self.latest_info is None:
            self.pose_history.clear()
            self.set_status("waiting for image/camera_info")
            return

        image_age = self.message_age_s(self.latest_image)
        info_age = self.message_age_s(self.latest_info)
        if self.max_message_age_s > 0.0:
            stale_reasons = []
            if image_age is not None and image_age > self.max_message_age_s:
                stale_reasons.append(f"image stale {image_age:.3f}s")
            if info_age is not None and info_age > self.max_message_age_s:
                stale_reasons.append(f"camera_info stale {info_age:.3f}s")
            if stale_reasons:
                self.pose_history.clear()
                self.set_status(", ".join(stale_reasons))
                return

        bgr = self.to_bgr(self.latest_image)
        if bgr is None:
            self.pose_history.clear()
            self.set_status("unsupported image encoding")
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
            self.pose_history.clear()
            self.set_status("no aruco markers detected")
            debug_lines.append(("No markers detected", (0, 0, 255)))
            self.draw_debug_lines(debug, debug_lines)
            self.publish_debug(debug, self.latest_image.header.frame_id)
            return

        cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        ids_flat = ids.flatten().tolist()
        ids_msg = Int32MultiArray()
        ids_msg.data = [int(v) for v in ids_flat]
        self.ids_pub.publish(ids_msg)
        debug_lines.append((f"Detected IDs: {ids_flat} dict={used_dict_name}", (0, 255, 0)))

        target_id = self.marker_id if self.marker_id >= 0 else ids_flat[0]
        if target_id not in ids_flat:
            self.pose_history.clear()
            self.set_status(f"target id {self.marker_id} not found")
            debug_lines.append((f"Target ID {self.marker_id} not found", (0, 0, 255)))
            self.draw_debug_lines(debug, debug_lines)
            self.publish_debug(debug, self.latest_image.header.frame_id)
            return

        idx = ids_flat.index(target_id)
        c = [corners[idx]]

        k = np.array(self.latest_info.k, dtype=np.float64).reshape(3, 3)
        d = np.array(self.latest_info.d, dtype=np.float64)
        rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(c, self.marker_size_m, k, d)
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]
        marker_distance_m = float(np.linalg.norm(tvec))

        if (
            self.min_marker_distance_m > 0.0
            and marker_distance_m < self.min_marker_distance_m
        ) or (
            self.max_marker_distance_m > 0.0
            and marker_distance_m > self.max_marker_distance_m
        ):
            self.pose_history.clear()
            self.set_status(f"marker distance rejected: {marker_distance_m:.3f}m")
            debug_lines.append(
                (f"Marker distance rejected: {marker_distance_m:.3f}m", (0, 0, 255))
            )
            self.draw_debug_lines(debug, debug_lines)
            self.publish_debug(debug, self.latest_image.header.frame_id)
            return

        cv2.drawFrameAxes(debug, k, d, rvec, tvec, self.marker_size_m * 0.6)
        pose_cam = self.build_pose_from_rt(
            rvec,
            tvec,
            self.latest_image.header.frame_id or self.latest_info.header.frame_id,
            self.latest_image.header.stamp,
        )
        self.pose_cam_pub.publish(pose_cam)

        try:
            pose_base = self.tf_buffer.transform(
                pose_cam,
                self.target_frame,
                timeout=Duration(seconds=0.05),
            )
            pos_base = np.array(
                [
                    float(pose_base.pose.position.x),
                    float(pose_base.pose.position.y),
                    float(pose_base.pose.position.z),
                ],
                dtype=np.float64,
            )
            smoothed = self.update_track(pos_base)
            if smoothed is not None:
                pose_base.pose.position.x = float(smoothed[0])
                pose_base.pose.position.y = float(smoothed[1])
                pose_base.pose.position.z = float(smoothed[2])
                self.pose_base_pub.publish(pose_base)
                self.set_status(
                    f"pose ok id={target_id} x={smoothed[0]:+.3f} y={smoothed[1]:+.3f} z={smoothed[2]:+.3f}"
                )
                debug_lines.append(
                    (
                        f"pose_base=({smoothed[0]:+.3f}, {smoothed[1]:+.3f}, {smoothed[2]:+.3f})",
                        (0, 255, 0),
                    )
                )
            else:
                self.set_status(
                    f"stabilizing marker {len(self.pose_history)}/{self.min_stable_samples}"
                )
                debug_lines.append(
                    (
                        f"Stabilizing marker ({len(self.pose_history)}/{self.min_stable_samples})",
                        (0, 255, 255),
                    )
                )
        except TransformException as exc:
            self.pose_history.clear()
            self.set_status(f"tf transform failed: {exc}")
            debug_lines.append((f"TF failed: {exc}", (0, 0, 255)))

        debug_lines.insert(
            0,
            (f"target_id={target_id} dist={marker_distance_m:.3f}m", (0, 255, 0)),
        )
        self.draw_debug_lines(debug, debug_lines)
        self.publish_debug(debug, self.latest_image.header.frame_id)

    def build_pose_from_rt(
        self,
        rvec: np.ndarray,
        tvec: np.ndarray,
        frame_id: str,
        stamp,
    ) -> PoseStamped:
        rot, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = self.quaternion_from_rotation(rot)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(tvec[0])
        msg.pose.position.y = float(tvec[1])
        msg.pose.position.z = float(tvec[2])
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def quaternion_from_rotation(self, rot: np.ndarray) -> Tuple[float, float, float, float]:
        trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (rot[2, 1] - rot[1, 2]) * s
            qy = (rot[0, 2] - rot[2, 0]) * s
            qz = (rot[1, 0] - rot[0, 1]) * s
            return float(qx), float(qy), float(qz), float(qw)

        if rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
            s = 2.0 * np.sqrt(max(1e-12, 1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]))
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
            qw = (rot[2, 1] - rot[1, 2]) / s
            return float(qx), float(qy), float(qz), float(qw)
        if rot[1, 1] > rot[2, 2]:
            s = 2.0 * np.sqrt(max(1e-12, 1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]))
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
            qw = (rot[0, 2] - rot[2, 0]) / s
            return float(qx), float(qy), float(qz), float(qw)

        s = 2.0 * np.sqrt(max(1e-12, 1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]))
        qx = (rot[0, 2] + rot[2, 0]) / s
        qy = (rot[1, 2] + rot[2, 1]) / s
        qz = 0.25 * s
        qw = (rot[1, 0] - rot[0, 1]) / s
        return float(qx), float(qy), float(qz), float(qw)

    def update_track(self, position: np.ndarray) -> Optional[np.ndarray]:
        if self.pose_history and self.max_position_jump_m > 0.0:
            prev = np.median(np.stack(self.pose_history, axis=0), axis=0)
            if np.linalg.norm(position - prev) > self.max_position_jump_m:
                self.pose_history.clear()
        self.pose_history.append(position)
        if len(self.pose_history) < self.min_stable_samples:
            return None
        return np.median(np.stack(self.pose_history, axis=0), axis=0)

    def message_age_s(self, msg) -> Optional[float]:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return None
        return max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e9)

    def set_status(self, text: str) -> None:
        if text == self._last_status_text:
            return
        self._last_status_text = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def draw_debug_lines(
        self,
        image: np.ndarray,
        lines: List[Tuple[str, Tuple[int, int, int]]],
    ) -> None:
        y = 28
        for text, color in lines:
            cv2.putText(
                image,
                text,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                color,
                2,
            )
            y += 28

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
