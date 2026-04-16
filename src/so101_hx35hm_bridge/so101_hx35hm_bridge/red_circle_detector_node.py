#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

import tf2_geometry_msgs  # noqa: F401


def _as_bgr(msg: Image) -> Optional[np.ndarray]:
    if msg.height <= 0 or msg.width <= 0:
        return None
    if msg.encoding not in ("rgb8", "bgr8"):
        return None
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def _depth_to_meters(msg: Image) -> Optional[np.ndarray]:
    if msg.height <= 0 or msg.width <= 0:
        return None

    if msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
        return arr[:, : msg.width].astype(np.float32)
    if msg.encoding == "16UC1":
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
        return (arr[:, : msg.width].astype(np.float32)) * 0.001
    return None


def _camera_intrinsics(info: CameraInfo) -> Tuple[float, float, float, float]:
    k = np.array(info.k, dtype=np.float64).reshape(3, 3)
    return float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])


class RedCircleDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("red_circle_detector")

        self.declare_parameter("image_topic", "/static_camera/image_raw")
        self.declare_parameter("depth_topic", "/static_camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/static_camera/depth/camera_info")
        self.declare_parameter("rgb_camera_info_topic", "")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("output_ns", "/vision/red_block")
        self.declare_parameter("min_area_px", 200.0)
        self.declare_parameter("min_circularity", 0.60)
        self.declare_parameter("depth_window_px", 5)
        self.declare_parameter("use_depth_median", True)
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 1.50)
        self.declare_parameter("min_z_m", -0.03)
        self.declare_parameter("max_z_m", 0.08)
        self.declare_parameter("smoothing_window", 5)
        self.declare_parameter("min_stable_samples", 3)
        self.declare_parameter("max_position_jump_m", 0.10)
        self.declare_parameter("h1_lower", [0, 120, 70])
        self.declare_parameter("h1_upper", [8, 255, 255])
        self.declare_parameter("h2_lower", [172, 120, 70])
        self.declare_parameter("h2_upper", [180, 255, 255])

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.rgb_camera_info_topic = str(self.get_parameter("rgb_camera_info_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.output_ns = str(self.get_parameter("output_ns").value).rstrip("/")
        self.min_area_px = float(self.get_parameter("min_area_px").value)
        self.min_circularity = float(self.get_parameter("min_circularity").value)
        self.depth_window_px = int(self.get_parameter("depth_window_px").value)
        self.use_depth_median = bool(self.get_parameter("use_depth_median").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_z_m = float(self.get_parameter("min_z_m").value)
        self.max_z_m = float(self.get_parameter("max_z_m").value)
        self.smoothing_window = max(1, int(self.get_parameter("smoothing_window").value))
        self.min_stable_samples = max(1, int(self.get_parameter("min_stable_samples").value))
        self.max_position_jump_m = max(0.0, float(self.get_parameter("max_position_jump_m").value))

        def _np_param(name: str) -> np.ndarray:
            return np.array(self.get_parameter(name).value, dtype=np.uint8)

        self.h1_lower = _np_param("h1_lower")
        self.h1_upper = _np_param("h1_upper")
        self.h2_lower = _np_param("h2_lower")
        self.h2_upper = _np_param("h2_upper")

        self.latest_rgb: Optional[Image] = None
        self.latest_depth: Optional[Image] = None
        self.latest_rgb_info: Optional[CameraInfo] = None
        self.latest_info: Optional[CameraInfo] = None
        self.pose_history: deque[np.ndarray] = deque(maxlen=self.smoothing_window)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_cam_pub = self.create_publisher(PoseStamped, f"{self.output_ns}/pose_camera", 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, f"{self.output_ns}/pose_base", 10)
        self.debug_pub = self.create_publisher(Image, f"{self.output_ns}/debug_image", 10)

        self.create_subscription(Image, self.image_topic, self.on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
        if self.rgb_camera_info_topic:
            self.create_subscription(CameraInfo, self.rgb_camera_info_topic, self.on_rgb_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self.process)
        self.get_logger().info(
            f"Red detector started: rgb='{self.image_topic}', depth='{self.depth_topic}', "
            f"info='{self.camera_info_topic}', rgb_info='{self.rgb_camera_info_topic or '-'}', "
            f"target_frame='{self.target_frame}'"
        )

    def on_rgb(self, msg: Image) -> None:
        self.latest_rgb = msg

    def on_depth(self, msg: Image) -> None:
        self.latest_depth = msg

    def on_rgb_info(self, msg: CameraInfo) -> None:
        self.latest_rgb_info = msg

    def on_info(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    def process(self) -> None:
        try:
            self._process_once()
        except Exception as exc:
            self.get_logger().error(f"Red detector process error (ignored, continue running): {exc}")

    def _process_once(self) -> None:
        if self.latest_rgb is None or self.latest_depth is None or self.latest_info is None:
            return

        bgr = _as_bgr(self.latest_rgb)
        depth = _depth_to_meters(self.latest_depth)
        if bgr is None or depth is None:
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.h1_lower, self.h1_upper)
        mask2 = cv2.inRange(hsv, self.h2_lower, self.h2_upper)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug = bgr.copy()
        if not contours:
            cv2.putText(debug, "No red target detected", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.publish_debug(debug)
            return

        def contour_score(cnt: np.ndarray) -> float:
            area = float(cv2.contourArea(cnt))
            if area < self.min_area_px:
                return -1.0
            peri = float(cv2.arcLength(cnt, True))
            if peri <= 1e-6:
                return -1.0
            circ = 4.0 * np.pi * area / (peri * peri)
            if circ < self.min_circularity:
                return -1.0
            return area * circ

        best = max(contours, key=contour_score)
        score = contour_score(best)
        if score < 0:
            cv2.putText(debug, "Red blob rejected", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.publish_debug(debug)
            return

        area = float(cv2.contourArea(best))
        (cx_f, cy_f), radius = cv2.minEnclosingCircle(best)
        rgb_cx = int(round(cx_f))
        rgb_cy = int(round(cy_f))
        if radius < 2.0:
            cv2.putText(debug, "Red blob too small", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.publish_debug(debug)
            return

        depth_cx, depth_cy = self.map_rgb_pixel_to_depth_pixel(rgb_cx, rgb_cy, bgr, depth)
        depth_val = self.sample_depth(depth, depth_cx, depth_cy)
        if depth_val is None:
            cv2.putText(debug, "Depth invalid", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.publish_debug(debug)
            return
        if not (self.min_depth_m <= depth_val <= self.max_depth_m):
            cv2.putText(
                debug,
                f"Depth rejected: {depth_val:.3f}m",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            self.publish_debug(debug)
            return

        fx, fy, cx0, cy0 = _camera_intrinsics(self.latest_info)
        x = (float(depth_cx) - cx0) * depth_val / fx
        y = (float(depth_cy) - cy0) * depth_val / fy
        z = depth_val

        pose_cam = PoseStamped()
        pose_cam.header.stamp = self.get_clock().now().to_msg()
        pose_cam.header.frame_id = self.latest_depth.header.frame_id or self.latest_info.header.frame_id
        pose_cam.pose.position.x = float(x)
        pose_cam.pose.position.y = float(y)
        pose_cam.pose.position.z = float(z)
        pose_cam.pose.orientation.w = 1.0
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
            z_base = float(pos_base[2])
            if self.min_z_m <= z_base <= self.max_z_m:
                smoothed = self.update_track(pos_base)
                if smoothed is not None:
                    pose_base.pose.position.x = float(smoothed[0])
                    pose_base.pose.position.y = float(smoothed[1])
                    pose_base.pose.position.z = float(smoothed[2])
                    self.pose_base_pub.publish(pose_base)
                else:
                    cv2.putText(
                        debug,
                        f"Stabilizing target ({len(self.pose_history)}/{self.min_stable_samples})",
                        (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2,
                    )
            else:
                self.pose_history.clear()
                cv2.putText(
                    debug,
                    f"Rejected by z gate: z={z_base:.3f}m",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
        except TransformException:
            pass

        cv2.circle(debug, (rgb_cx, rgb_cy), int(round(radius)), (0, 255, 0), 2)
        cv2.drawMarker(
            debug,
            (rgb_cx, rgb_cy),
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
        )
        cv2.drawMarker(
            debug,
            (depth_cx, depth_cy),
            (255, 255, 0),
            markerType=cv2.MARKER_TILTED_CROSS,
            markerSize=20,
            thickness=2,
        )
        cv2.putText(
            debug,
            f"red area={area:.0f} depth={depth_val:.3f}m rgb=({rgb_cx},{rgb_cy}) depth=({depth_cx},{depth_cy})",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 0),
            2,
        )
        self.publish_debug(debug)

    def map_rgb_pixel_to_depth_pixel(self, rgb_x: int, rgb_y: int, rgb: np.ndarray, depth: np.ndarray) -> Tuple[int, int]:
        rgb_h, rgb_w = rgb.shape[:2]
        depth_h, depth_w = depth.shape[:2]
        if rgb_w <= 1 or rgb_h <= 1 or depth_w <= 1 or depth_h <= 1:
            return rgb_x, rgb_y

        sx = float(depth_w - 1) / float(max(1, rgb_w - 1))
        sy = float(depth_h - 1) / float(max(1, rgb_h - 1))
        depth_x = int(round(rgb_x * sx))
        depth_y = int(round(rgb_y * sy))
        depth_x = int(np.clip(depth_x, 0, depth_w - 1))
        depth_y = int(np.clip(depth_y, 0, depth_h - 1))
        return depth_x, depth_y

    def update_track(self, position: np.ndarray) -> Optional[np.ndarray]:
        if self.pose_history and self.max_position_jump_m > 0.0:
            prev = np.median(np.stack(self.pose_history, axis=0), axis=0)
            if np.linalg.norm(position - prev) > self.max_position_jump_m:
                self.pose_history.clear()
        self.pose_history.append(position)
        if len(self.pose_history) < self.min_stable_samples:
            return None
        return np.median(np.stack(self.pose_history, axis=0), axis=0)

    def sample_depth(self, depth: np.ndarray, cx: int, cy: int) -> Optional[float]:
        h, w = depth.shape[:2]
        r = max(1, int(self.depth_window_px))
        x0 = max(0, cx - r)
        x1 = min(w, cx + r + 1)
        y0 = max(0, cy - r)
        y1 = min(h, cy + r + 1)
        window = depth[y0:y1, x0:x1].astype(np.float32)
        valid = window[np.isfinite(window) & (window > 0.0)]
        if valid.size == 0:
            return None
        if self.use_depth_median:
            return float(np.median(valid))
        return float(valid[valid.size // 2])

    def publish_debug(self, bgr: np.ndarray) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.latest_rgb.header.frame_id if self.latest_rgb else ""
        msg.height = int(bgr.shape[0])
        msg.width = int(bgr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(bgr.shape[1] * 3)
        msg.data = bgr.tobytes()
        self.debug_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RedCircleDetectorNode()
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
