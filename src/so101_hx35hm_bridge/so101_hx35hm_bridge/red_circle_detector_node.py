#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
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


def _normalize_quaternion(q_xyzw: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q_xyzw))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q_xyzw.astype(np.float64) / norm


def _quat_to_rotmat(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = _normalize_quaternion(q_xyzw)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


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
        self.declare_parameter("depth_search_radius_px", 8)
        self.declare_parameter("use_depth_median", True)
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 1.50)
        self.declare_parameter("min_z_m", -0.03)
        self.declare_parameter("max_z_m", 0.08)
        self.declare_parameter("pixel_mapping_mode", "auto")
        self.declare_parameter("depth_to_rgb_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("depth_to_rgb_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("registration_max_residual_px", 12.0)
        self.declare_parameter("enable_plane_fallback", False)
        self.declare_parameter("fallback_plane_z_m", 0.0)
        self.declare_parameter("max_message_age_s", 0.25)
        self.declare_parameter("max_rgb_depth_skew_s", 0.15)
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
        self.depth_search_radius_px = max(0, int(self.get_parameter("depth_search_radius_px").value))
        self.use_depth_median = bool(self.get_parameter("use_depth_median").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_z_m = float(self.get_parameter("min_z_m").value)
        self.max_z_m = float(self.get_parameter("max_z_m").value)
        self.pixel_mapping_mode = str(self.get_parameter("pixel_mapping_mode").value).strip().lower()
        self.depth_to_rgb_translation = np.array(
            self.get_parameter("depth_to_rgb_translation_m").value,
            dtype=np.float64,
        ).reshape(3)
        self.depth_to_rgb_quaternion = _normalize_quaternion(
            np.array(
                self.get_parameter("depth_to_rgb_quaternion_xyzw").value,
                dtype=np.float64,
            ).reshape(4)
        )
        self.registration_max_residual_px = max(
            0.0,
            float(self.get_parameter("registration_max_residual_px").value),
        )
        self.enable_plane_fallback = bool(self.get_parameter("enable_plane_fallback").value)
        self.fallback_plane_z_m = float(self.get_parameter("fallback_plane_z_m").value)
        self.max_message_age_s = max(0.0, float(self.get_parameter("max_message_age_s").value))
        self.max_rgb_depth_skew_s = max(0.0, float(self.get_parameter("max_rgb_depth_skew_s").value))
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
        self._warn_times: Dict[str, float] = {}
        self._last_status_text = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_cam_pub = self.create_publisher(PoseStamped, f"{self.output_ns}/pose_camera", 10)
        self.pose_base_pub = self.create_publisher(PoseStamped, f"{self.output_ns}/pose_base", 10)
        self.debug_pub = self.create_publisher(Image, f"{self.output_ns}/debug_image", 10)
        self.status_pub = self.create_publisher(String, f"{self.output_ns}/status", 10)

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
        debug_lines: List[Tuple[str, Tuple[int, int, int]]] = []
        have_rgb = self.latest_rgb is not None
        have_depth = self.latest_depth is not None and self.latest_info is not None
        have_plane_fallback = self.enable_plane_fallback and (
            self.latest_rgb_info is not None or self.latest_info is not None
        )
        if not have_rgb or (not have_depth and not have_plane_fallback):
            self.pose_history.clear()
            self.set_status("waiting for rgb/depth/camera_info")
            return

        rgb_age = self.message_age_s(self.latest_rgb)
        depth_age = self.message_age_s(self.latest_depth) if self.latest_depth is not None else None
        depth_info_age = self.message_age_s(self.latest_info) if self.latest_info is not None else None
        if self.max_message_age_s > 0.0:
            stale_reasons = []
            if rgb_age is not None and rgb_age > self.max_message_age_s:
                stale_reasons.append(f"rgb stale {rgb_age:.3f}s")
            if (
                have_depth
                and not have_plane_fallback
                and depth_age is not None
                and depth_age > self.max_message_age_s
            ):
                stale_reasons.append(f"depth stale {depth_age:.3f}s")
            if (
                have_depth
                and not have_plane_fallback
                and depth_info_age is not None
                and depth_info_age > self.max_message_age_s
            ):
                stale_reasons.append(f"depth_info stale {depth_info_age:.3f}s")
            if stale_reasons:
                self.pose_history.clear()
                self.set_status(", ".join(stale_reasons))
                return

        stamp_skew = self.timestamp_skew_s(self.latest_rgb, self.latest_depth) if self.latest_depth is not None else None
        if (
            have_depth
            and
            stamp_skew is not None
            and self.max_rgb_depth_skew_s > 0.0
            and stamp_skew > self.max_rgb_depth_skew_s
        ):
            self.pose_history.clear()
            self.set_status(f"rgb/depth skew too large: {stamp_skew:.3f}s")
            return

        bgr = _as_bgr(self.latest_rgb)
        depth = _depth_to_meters(self.latest_depth) if self.latest_depth is not None else None
        if bgr is None:
            self.pose_history.clear()
            self.set_status("unsupported image encoding")
            return

        mapping_mode = "rgb_only"
        if depth is not None:
            mapping_mode = self.resolve_pixel_mapping_mode(bgr, depth)
            debug_lines.append((f"pixel_map={mapping_mode}", (255, 255, 0)))
        if mapping_mode == "scale":
            self.warn_throttled(
                "red_scale_mapping",
                "Red detector is using approximate RGB->depth scaling. "
                "For best results, use an aligned depth image or a calibrated RGB-depth registration pipeline.",
            )

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
            self.set_status("no red target detected")
            debug_lines.append(("No red target detected", (0, 0, 255)))
            self.draw_debug_lines(debug, debug_lines)
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
            self.set_status("red target rejected by area/circularity gate")
            debug_lines.append(("Red blob rejected", (0, 0, 255)))
            self.draw_debug_lines(debug, debug_lines)
            self.publish_debug(debug)
            return

        area = float(cv2.contourArea(best))
        (cx_f, cy_f), radius = cv2.minEnclosingCircle(best)
        rgb_cx = int(round(cx_f))
        rgb_cy = int(round(cy_f))
        if radius < 2.0:
            self.set_status("red target too small")
            debug_lines.append(("Red blob too small", (0, 0, 255)))
            self.draw_debug_lines(debug, debug_lines)
            self.publish_debug(debug)
            return

        pose_cam: Optional[PoseStamped] = None
        pose_base: Optional[PoseStamped] = None
        depth_val: Optional[float] = None
        depth_cx = rgb_cx
        depth_cy = rgb_cy
        registration_error_px: Optional[float] = None
        used_plane_fallback = False

        if depth is not None:
            depth_cx, depth_cy, registration_error_px = self.map_rgb_pixel_to_depth_pixel(
                rgb_cx,
                rgb_cy,
                bgr,
                depth,
                mapping_mode,
            )
            depth_sample = self.sample_depth(depth, depth_cx, depth_cy)
            if depth_sample is not None:
                depth_val, depth_cx, depth_cy = depth_sample

        if depth_val is not None and (self.min_depth_m <= depth_val <= self.max_depth_m):
            camera_point, pose_frame_id = self.compute_camera_point_from_depth_sample(
                depth_cx,
                depth_cy,
                depth_val,
                mapping_mode,
            )
            if camera_point is None or not pose_frame_id:
                self.pose_history.clear()
                self.set_status("depth->camera projection failed")
                debug_lines.append(("Depth->camera projection failed", (0, 0, 255)))
                self.draw_debug_lines(debug, debug_lines)
                self.publish_debug(debug)
                return

            pose_cam = PoseStamped()
            pose_cam.header.stamp = self.select_pose_stamp(mapping_mode)
            pose_cam.header.frame_id = pose_frame_id
            pose_cam.pose.position.x = float(camera_point[0])
            pose_cam.pose.position.y = float(camera_point[1])
            pose_cam.pose.position.z = float(camera_point[2])
            pose_cam.pose.orientation.w = 1.0
            self.pose_cam_pub.publish(pose_cam)
        elif self.enable_plane_fallback:
            plane_projection = self.project_rgb_pixel_to_plane(rgb_cx, rgb_cy)
            if plane_projection is None:
                self.pose_history.clear()
                self.set_status("depth invalid and plane fallback failed")
                debug_lines.append(("Plane fallback failed", (0, 0, 255)))
                self.draw_debug_lines(debug, debug_lines)
                self.publish_debug(debug)
                return
            pose_cam, pose_base = plane_projection
            self.pose_cam_pub.publish(pose_cam)
            used_plane_fallback = True
            debug_lines.append((f"plane_fallback z={self.fallback_plane_z_m:.3f}", (0, 255, 255)))
        else:
            self.pose_history.clear()
            self.set_status("depth invalid near target")
            debug_lines.append(("Depth invalid", (0, 0, 255)))
            self.draw_debug_lines(debug, debug_lines)
            self.publish_debug(debug)
            return

        if depth_val is not None and not (self.min_depth_m <= depth_val <= self.max_depth_m):
            self.pose_history.clear()
            self.set_status(f"depth rejected {depth_val:.3f}m")
            debug_lines.append((f"Depth rejected: {depth_val:.3f}m", (0, 0, 255)))
            self.draw_debug_lines(debug, debug_lines)
            self.publish_debug(debug)
            return

        try:
            if pose_base is None:
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
                    status_prefix = "pose ok (plane)" if used_plane_fallback else "pose ok"
                    self.set_status(
                        f"{status_prefix} x={smoothed[0]:+.3f} y={smoothed[1]:+.3f} z={smoothed[2]:+.3f}"
                    )
                    debug_lines.append(
                        (
                            f"pose_base=({smoothed[0]:+.3f}, {smoothed[1]:+.3f}, {smoothed[2]:+.3f})",
                            (0, 255, 0),
                        )
                    )
                else:
                    self.set_status(
                        f"stabilizing target {len(self.pose_history)}/{self.min_stable_samples}"
                    )
                    debug_lines.append(
                        (
                            f"Stabilizing target ({len(self.pose_history)}/{self.min_stable_samples})",
                            (0, 255, 255),
                        )
                    )
            else:
                self.pose_history.clear()
                self.set_status(f"rejected by z gate: {z_base:.3f}m")
                debug_lines.append((f"Rejected by z gate: z={z_base:.3f}m", (0, 0, 255)))
        except TransformException as exc:
            self.pose_history.clear()
            self.set_status(f"tf transform failed: {exc}")
            debug_lines.append((f"TF failed: {exc}", (0, 0, 255)))

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
        if depth_val is not None:
            debug_lines.insert(
                0,
                (
                    f"red area={area:.0f} depth={depth_val:.3f}m rgb=({rgb_cx},{rgb_cy}) depth=({depth_cx},{depth_cy})",
                    (0, 255, 0),
                ),
            )
        else:
            debug_lines.insert(
                0,
                (
                    f"red area={area:.0f} rgb=({rgb_cx},{rgb_cy}) depth=invalid",
                    (0, 255, 255) if used_plane_fallback else (0, 0, 255),
                ),
            )
        if stamp_skew is not None:
            debug_lines.append((f"rgb-depth skew={stamp_skew:.3f}s", (255, 255, 0)))
        if registration_error_px is not None:
            debug_lines.append((f"registration_err={registration_error_px:.2f}px", (255, 255, 0)))
        self.draw_debug_lines(debug, debug_lines)
        self.publish_debug(debug)

    def map_rgb_pixel_to_depth_pixel(
        self,
        rgb_x: int,
        rgb_y: int,
        rgb: np.ndarray,
        depth: np.ndarray,
        mapping_mode: str,
    ) -> Tuple[int, int, Optional[float]]:
        rgb_h, rgb_w = rgb.shape[:2]
        depth_h, depth_w = depth.shape[:2]
        if rgb_w <= 1 or rgb_h <= 1 or depth_w <= 1 or depth_h <= 1:
            return rgb_x, rgb_y, None

        if mapping_mode == "aligned":
            depth_x = int(np.clip(rgb_x, 0, depth_w - 1))
            depth_y = int(np.clip(rgb_y, 0, depth_h - 1))
            return depth_x, depth_y, 0.0

        if mapping_mode == "projective":
            projective_match = self.projective_match_rgb_to_depth_pixel(rgb_x, rgb_y, rgb, depth)
            if projective_match is not None:
                return projective_match

        sx = float(depth_w - 1) / float(max(1, rgb_w - 1))
        sy = float(depth_h - 1) / float(max(1, rgb_h - 1))
        depth_x = int(round(rgb_x * sx))
        depth_y = int(round(rgb_y * sy))
        depth_x = int(np.clip(depth_x, 0, depth_w - 1))
        depth_y = int(np.clip(depth_y, 0, depth_h - 1))
        return depth_x, depth_y, None

    def projective_match_rgb_to_depth_pixel(
        self,
        rgb_x: int,
        rgb_y: int,
        rgb: np.ndarray,
        depth: np.ndarray,
    ) -> Optional[Tuple[int, int, float]]:
        if self.latest_rgb_info is None or self.latest_info is None:
            return None

        extrinsic = self.get_depth_to_rgb_extrinsic()
        if extrinsic is None:
            return None
        rot_depth_to_rgb, trans_depth_to_rgb = extrinsic

        rgb_h, rgb_w = rgb.shape[:2]
        depth_h, depth_w = depth.shape[:2]
        rgb_seed_x, rgb_seed_y, _ = self.map_rgb_pixel_to_depth_pixel(rgb_x, rgb_y, rgb, depth, "scale")
        search_r = max(1, int(self.depth_search_radius_px))

        x0 = max(0, rgb_seed_x - search_r)
        x1 = min(depth_w, rgb_seed_x + search_r + 1)
        y0 = max(0, rgb_seed_y - search_r)
        y1 = min(depth_h, rgb_seed_y + search_r + 1)
        patch = depth[y0:y1, x0:x1].astype(np.float64)
        valid_mask = np.isfinite(patch) & (patch > 0.0)
        if not np.any(valid_mask):
            return None

        py, px = np.nonzero(valid_mask)
        px_depth = (x0 + px).astype(np.float64)
        py_depth = (y0 + py).astype(np.float64)
        z_depth = patch[py, px]

        fx_d, fy_d, cx_d, cy_d = _camera_intrinsics(self.latest_info)
        fx_r, fy_r, cx_r, cy_r = _camera_intrinsics(self.latest_rgb_info)
        if min(abs(fx_d), abs(fy_d), abs(fx_r), abs(fy_r)) < 1e-9:
            return None

        x_depth = (px_depth - cx_d) * z_depth / fx_d
        y_depth = (py_depth - cy_d) * z_depth / fy_d
        points_depth = np.stack([x_depth, y_depth, z_depth], axis=0)
        points_rgb = rot_depth_to_rgb @ points_depth + trans_depth_to_rgb.reshape(3, 1)

        z_rgb = points_rgb[2, :]
        forward_mask = z_rgb > 1e-6
        if not np.any(forward_mask):
            return None

        px_depth = px_depth[forward_mask]
        py_depth = py_depth[forward_mask]
        points_rgb = points_rgb[:, forward_mask]
        z_rgb = z_rgb[forward_mask]

        u_rgb = fx_r * points_rgb[0, :] / z_rgb + cx_r
        v_rgb = fy_r * points_rgb[1, :] / z_rgb + cy_r
        residual_sq = (u_rgb - float(rgb_x)) ** 2 + (v_rgb - float(rgb_y)) ** 2
        best_idx = int(np.argmin(residual_sq))
        best_err_px = float(np.sqrt(residual_sq[best_idx]))
        if self.registration_max_residual_px > 0.0 and best_err_px > self.registration_max_residual_px:
            self.warn_throttled(
                "red_projective_registration",
                f"Projective RGB-D registration residual too large: {best_err_px:.2f}px "
                f"(limit={self.registration_max_residual_px:.2f}px). Falling back to approximate mapping.",
            )
            return None
        return int(round(px_depth[best_idx])), int(round(py_depth[best_idx])), best_err_px

    def update_track(self, position: np.ndarray) -> Optional[np.ndarray]:
        if self.pose_history and self.max_position_jump_m > 0.0:
            prev = np.median(np.stack(self.pose_history, axis=0), axis=0)
            if np.linalg.norm(position - prev) > self.max_position_jump_m:
                self.pose_history.clear()
        self.pose_history.append(position)
        if len(self.pose_history) < self.min_stable_samples:
            return None
        return np.median(np.stack(self.pose_history, axis=0), axis=0)

    def sample_depth(self, depth: np.ndarray, cx: int, cy: int) -> Optional[Tuple[float, int, int]]:
        h, w = depth.shape[:2]
        r = max(1, int(self.depth_window_px))
        search_r = max(r, int(self.depth_search_radius_px))

        sx0 = max(0, cx - search_r)
        sx1 = min(w, cx + search_r + 1)
        sy0 = max(0, cy - search_r)
        sy1 = min(h, cy + search_r + 1)
        candidates = depth[sy0:sy1, sx0:sx1].astype(np.float32)
        valid_mask = np.isfinite(candidates) & (candidates > 0.0)
        if not np.any(valid_mask):
            return None

        candidate_pixels = np.argwhere(valid_mask)
        center = np.array([cy - sy0, cx - sx0], dtype=np.float32)
        distances = np.linalg.norm(candidate_pixels.astype(np.float32) - center, axis=1)
        best_idx = int(np.argmin(distances))
        best_py, best_px = candidate_pixels[best_idx]
        best_x = int(sx0 + best_px)
        best_y = int(sy0 + best_py)

        x0 = max(0, best_x - r)
        x1 = min(w, best_x + r + 1)
        y0 = max(0, best_y - r)
        y1 = min(h, best_y + r + 1)
        window = depth[y0:y1, x0:x1].astype(np.float32)
        valid = window[np.isfinite(window) & (window > 0.0)]
        if valid.size == 0:
            return None
        if self.use_depth_median:
            depth_value = float(np.median(valid))
        else:
            depth_value = float(valid[valid.size // 2])
        return depth_value, best_x, best_y

    def resolve_pixel_mapping_mode(self, rgb: np.ndarray, depth: np.ndarray) -> str:
        if self.pixel_mapping_mode in ("aligned", "scale", "projective"):
            return self.pixel_mapping_mode

        rgb_shape = rgb.shape[:2]
        depth_shape = depth.shape[:2]
        same_size = rgb_shape == depth_shape
        same_frame = (
            bool(self.latest_rgb and self.latest_rgb.header.frame_id)
            and bool(self.latest_depth and self.latest_depth.header.frame_id)
            and self.latest_rgb.header.frame_id == self.latest_depth.header.frame_id
        )
        if same_size and (same_frame or self.latest_rgb_info is not None):
            return "aligned"
        if self.can_use_projective_registration():
            return "projective"
        return "scale"

    def select_projection_camera_info(self, mapping_mode: str) -> CameraInfo:
        if mapping_mode == "aligned" and self.latest_rgb_info is not None:
            return self.latest_rgb_info
        return self.latest_info

    def can_use_projective_registration(self) -> bool:
        return self.latest_rgb_info is not None and self.latest_info is not None

    def rgb_frame_id(self) -> str:
        return (
            (self.latest_rgb.header.frame_id if self.latest_rgb is not None else "")
            or (self.latest_rgb_info.header.frame_id if self.latest_rgb_info is not None else "")
            or (self.latest_info.header.frame_id if self.latest_info is not None else "")
        )

    def depth_frame_id(self) -> str:
        return (
            (self.latest_depth.header.frame_id if self.latest_depth is not None else "")
            or (self.latest_info.header.frame_id if self.latest_info is not None else "")
            or self.rgb_frame_id()
        )

    def compute_camera_point_from_depth_sample(
        self,
        depth_x: int,
        depth_y: int,
        depth_val: float,
        mapping_mode: str,
    ) -> Tuple[Optional[np.ndarray], str]:
        if self.latest_info is None:
            return None, ""

        if mapping_mode == "aligned":
            info = self.latest_rgb_info if self.latest_rgb_info is not None else self.latest_info
            fx, fy, cx0, cy0 = _camera_intrinsics(info)
            if min(abs(fx), abs(fy)) < 1e-9:
                return None, ""
            point = np.array(
                [
                    (float(depth_x) - cx0) * depth_val / fx,
                    (float(depth_y) - cy0) * depth_val / fy,
                    depth_val,
                ],
                dtype=np.float64,
            )
            return point, self.rgb_frame_id()

        fx_d, fy_d, cx_d, cy_d = _camera_intrinsics(self.latest_info)
        if min(abs(fx_d), abs(fy_d)) < 1e-9:
            return None, ""

        point_depth = np.array(
            [
                (float(depth_x) - cx_d) * depth_val / fx_d,
                (float(depth_y) - cy_d) * depth_val / fy_d,
                depth_val,
            ],
            dtype=np.float64,
        )

        if mapping_mode == "projective":
            extrinsic = self.get_depth_to_rgb_extrinsic()
            if extrinsic is None:
                return None, ""
            rot_depth_to_rgb, trans_depth_to_rgb = extrinsic
            point_rgb = rot_depth_to_rgb @ point_depth + trans_depth_to_rgb
            return point_rgb, self.rgb_frame_id()

        return point_depth, self.depth_frame_id()

    def get_depth_to_rgb_extrinsic(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        depth_frame = self.depth_frame_id()
        rgb_frame = self.rgb_frame_id()
        if depth_frame and rgb_frame and depth_frame != rgb_frame:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    rgb_frame,
                    depth_frame,
                    self.latest_depth.header.stamp if self.latest_depth is not None else rclpy.time.Time(),
                    timeout=Duration(seconds=0.02),
                )
                quat = np.array(
                    [
                        float(tf_msg.transform.rotation.x),
                        float(tf_msg.transform.rotation.y),
                        float(tf_msg.transform.rotation.z),
                        float(tf_msg.transform.rotation.w),
                    ],
                    dtype=np.float64,
                )
                trans = np.array(
                    [
                        float(tf_msg.transform.translation.x),
                        float(tf_msg.transform.translation.y),
                        float(tf_msg.transform.translation.z),
                    ],
                    dtype=np.float64,
                )
                return _quat_to_rotmat(quat), trans
            except TransformException:
                pass

        return _quat_to_rotmat(self.depth_to_rgb_quaternion), self.depth_to_rgb_translation

    def project_rgb_pixel_to_plane(self, rgb_x: int, rgb_y: int) -> Optional[Tuple[PoseStamped, PoseStamped]]:
        info = self.latest_rgb_info if self.latest_rgb_info is not None else self.latest_info
        if info is None:
            return None

        fx, fy, cx0, cy0 = _camera_intrinsics(info)
        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            return None

        camera_frame = (
            (self.latest_rgb.header.frame_id if self.latest_rgb is not None else "")
            or info.header.frame_id
            or (self.latest_info.header.frame_id if self.latest_info is not None else "")
        )
        if not camera_frame:
            return None

        stamp = self.latest_rgb.header.stamp if self.latest_rgb is not None else self.get_clock().now().to_msg()
        ray_cam = np.array(
            [
                (float(rgb_x) - cx0) / fx,
                (float(rgb_y) - cy0) / fy,
                1.0,
            ],
            dtype=np.float64,
        )

        origin_cam = PoseStamped()
        origin_cam.header.stamp = stamp
        origin_cam.header.frame_id = camera_frame
        origin_cam.pose.orientation.w = 1.0

        ray_point_cam = PoseStamped()
        ray_point_cam.header.stamp = stamp
        ray_point_cam.header.frame_id = camera_frame
        ray_point_cam.pose.position.x = float(ray_cam[0])
        ray_point_cam.pose.position.y = float(ray_cam[1])
        ray_point_cam.pose.position.z = float(ray_cam[2])
        ray_point_cam.pose.orientation.w = 1.0

        try:
            origin_base = self.tf_buffer.transform(
                origin_cam,
                self.target_frame,
                timeout=Duration(seconds=0.05),
            )
            ray_base = self.tf_buffer.transform(
                ray_point_cam,
                self.target_frame,
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None

        origin = np.array(
            [
                float(origin_base.pose.position.x),
                float(origin_base.pose.position.y),
                float(origin_base.pose.position.z),
            ],
            dtype=np.float64,
        )
        direction = np.array(
            [
                float(ray_base.pose.position.x) - origin[0],
                float(ray_base.pose.position.y) - origin[1],
                float(ray_base.pose.position.z) - origin[2],
            ],
            dtype=np.float64,
        )

        if abs(direction[2]) < 1e-9:
            return None

        scale = (self.fallback_plane_z_m - origin[2]) / direction[2]
        if scale <= 0.0 or not np.isfinite(scale):
            return None

        point_cam = ray_cam * scale
        point_base = origin + direction * scale

        pose_cam = PoseStamped()
        pose_cam.header.stamp = stamp
        pose_cam.header.frame_id = camera_frame
        pose_cam.pose.position.x = float(point_cam[0])
        pose_cam.pose.position.y = float(point_cam[1])
        pose_cam.pose.position.z = float(point_cam[2])
        pose_cam.pose.orientation.w = 1.0

        pose_base = PoseStamped()
        pose_base.header.stamp = stamp
        pose_base.header.frame_id = self.target_frame
        pose_base.pose.position.x = float(point_base[0])
        pose_base.pose.position.y = float(point_base[1])
        pose_base.pose.position.z = float(point_base[2])
        pose_base.pose.orientation.w = 1.0

        return pose_cam, pose_base

    def select_pose_frame_id(self, mapping_mode: str) -> str:
        if mapping_mode in ("aligned", "projective"):
            return self.rgb_frame_id()
        return self.depth_frame_id()

    def select_pose_stamp(self, mapping_mode: str):
        if mapping_mode in ("aligned", "projective") and self.latest_rgb is not None:
            return self.latest_rgb.header.stamp
        if self.latest_depth is not None:
            return self.latest_depth.header.stamp
        return self.get_clock().now().to_msg()

    def message_age_s(self, msg) -> Optional[float]:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return None
        return max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e9)

    def timestamp_skew_s(self, msg_a, msg_b) -> Optional[float]:
        stamp_a = getattr(getattr(msg_a, "header", None), "stamp", None)
        stamp_b = getattr(getattr(msg_b, "header", None), "stamp", None)
        if stamp_a is None or stamp_b is None:
            return None
        ns_a = int(stamp_a.sec) * 1_000_000_000 + int(stamp_a.nanosec)
        ns_b = int(stamp_b.sec) * 1_000_000_000 + int(stamp_b.nanosec)
        if ns_a <= 0 or ns_b <= 0:
            return None
        return abs(ns_a - ns_b) / 1e9

    def set_status(self, text: str) -> None:
        if text == self._last_status_text:
            return
        self._last_status_text = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def warn_throttled(self, key: str, message: str, interval_s: float = 10.0) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        last_s = self._warn_times.get(key, -1e9)
        if now_s - last_s >= interval_s:
            self._warn_times[key] = now_s
            self.get_logger().warn(message)

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
