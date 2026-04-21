#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformException, TransformListener


def _depth_to_meters(msg: Image) -> Optional[np.ndarray]:
    if msg.height <= 0 or msg.width <= 0:
        return None
    if msg.encoding == "32FC1":
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
        return arr[:, : msg.width].astype(np.float32)
    if msg.encoding == "16UC1":
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
        return arr[:, : msg.width].astype(np.float32) * 0.001
    return None


def _camera_intrinsics(info: CameraInfo) -> Tuple[float, float, float, float]:
    k = np.array(info.k, dtype=np.float64).reshape(3, 3)
    return float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])


class TablePlaneEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("table_plane_estimator")

        self.declare_parameter("depth_topic", "/static_camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/static_camera/depth/camera_info")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("output_topic", "/vision/table/top_z")
        self.declare_parameter("output_pose_topic", "/vision/table/pose_base")
        self.declare_parameter("status_topic", "/vision/table/status")
        self.declare_parameter("max_message_age_s", 0.5)
        self.declare_parameter("roi_u_min", 0.30)
        self.declare_parameter("roi_u_max", 0.70)
        self.declare_parameter("roi_v_min", 0.55)
        self.declare_parameter("roi_v_max", 0.92)
        self.declare_parameter("sample_step_px", 6)
        self.declare_parameter("min_depth_m", 0.10)
        self.declare_parameter("max_depth_m", 1.50)
        self.declare_parameter("min_points", 80)
        self.declare_parameter("z_trim_margin_m", 0.015)
        self.declare_parameter("smoothing_window", 5)
        self.declare_parameter("publish_period_s", 0.25)

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_pose_topic = str(self.get_parameter("output_pose_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.max_message_age_s = max(0.0, float(self.get_parameter("max_message_age_s").value))
        self.roi_u_min = float(self.get_parameter("roi_u_min").value)
        self.roi_u_max = float(self.get_parameter("roi_u_max").value)
        self.roi_v_min = float(self.get_parameter("roi_v_min").value)
        self.roi_v_max = float(self.get_parameter("roi_v_max").value)
        self.sample_step_px = max(1, int(self.get_parameter("sample_step_px").value))
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_points = max(10, int(self.get_parameter("min_points").value))
        self.z_trim_margin_m = max(0.0, float(self.get_parameter("z_trim_margin_m").value))
        self.smoothing_window = max(1, int(self.get_parameter("smoothing_window").value))
        self.publish_period_s = max(0.05, float(self.get_parameter("publish_period_s").value))

        self.latest_depth: Optional[Image] = None
        self.latest_info: Optional[CameraInfo] = None
        self.z_history: Deque[float] = deque(maxlen=self.smoothing_window)
        self._last_status = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.z_pub = self.create_publisher(Float64, self.output_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, qos_profile_sensor_data)
        self.timer = self.create_timer(self.publish_period_s, self.process)

        self.get_logger().info(
            f"Table estimator started: depth='{self.depth_topic}', info='{self.camera_info_topic}', "
            f"target_frame='{self.target_frame}', roi=({self.roi_u_min:.2f},{self.roi_v_min:.2f})-"
            f"({self.roi_u_max:.2f},{self.roi_v_max:.2f})"
        )

    def on_depth(self, msg: Image) -> None:
        self.latest_depth = msg

    def on_info(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    def process(self) -> None:
        try:
            self._process_once()
        except Exception as exc:
            self.get_logger().error(f"Table estimator error (ignored, continue running): {exc}")

    def _process_once(self) -> None:
        if self.latest_depth is None or self.latest_info is None:
            self.set_status("waiting for depth/camera_info")
            return

        age_s = self.message_age_s(self.latest_depth)
        if age_s is not None and self.max_message_age_s > 0.0 and age_s > self.max_message_age_s:
            self.set_status(f"depth stale {age_s:.3f}s")
            return

        depth = _depth_to_meters(self.latest_depth)
        if depth is None:
            self.set_status("unsupported depth encoding")
            return

        fx, fy, cx0, cy0 = _camera_intrinsics(self.latest_info)
        if abs(fx) < 1e-6 or abs(fy) < 1e-6:
            self.set_status("invalid intrinsics")
            return

        h, w = depth.shape[:2]
        u0 = max(0, min(w - 1, int(round(self.roi_u_min * (w - 1)))))
        u1 = max(0, min(w, int(round(self.roi_u_max * (w - 1))) + 1))
        v0 = max(0, min(h - 1, int(round(self.roi_v_min * (h - 1)))))
        v1 = max(0, min(h, int(round(self.roi_v_max * (h - 1))) + 1))
        if u1 - u0 < 2 or v1 - v0 < 2:
            self.set_status("invalid roi")
            return

        us = np.arange(u0, u1, self.sample_step_px, dtype=np.int32)
        vs = np.arange(v0, v1, self.sample_step_px, dtype=np.int32)
        if us.size == 0 or vs.size == 0:
            self.set_status("roi sampling empty")
            return

        uu, vv = np.meshgrid(us, vs)
        sampled_depth = depth[vv, uu]
        valid = np.isfinite(sampled_depth)
        valid &= sampled_depth >= self.min_depth_m
        valid &= sampled_depth <= self.max_depth_m
        if int(np.count_nonzero(valid)) < self.min_points:
            self.set_status(f"not enough valid depth points ({int(np.count_nonzero(valid))})")
            return

        sampled_depth = sampled_depth[valid].astype(np.float64)
        uu = uu[valid].astype(np.float64)
        vv = vv[valid].astype(np.float64)

        x_cam = (uu - cx0) * sampled_depth / fx
        y_cam = (vv - cy0) * sampled_depth / fy
        z_cam = sampled_depth
        points_cam = np.stack([x_cam, y_cam, z_cam], axis=1)

        frame_id = self.latest_depth.header.frame_id or self.latest_info.header.frame_id
        if not frame_id:
            self.set_status("missing depth frame id")
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                self.latest_depth.header.stamp,
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.set_status(f"tf failed: {exc}")
            return

        tx = float(tf_msg.transform.translation.x)
        ty = float(tf_msg.transform.translation.y)
        tz = float(tf_msg.transform.translation.z)
        qx = float(tf_msg.transform.rotation.x)
        qy = float(tf_msg.transform.rotation.y)
        qz = float(tf_msg.transform.rotation.z)
        qw = float(tf_msg.transform.rotation.w)

        q_norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if q_norm < 1e-9:
            self.set_status("invalid tf quaternion")
            return
        qx /= q_norm
        qy /= q_norm
        qz /= q_norm
        qw /= q_norm

        rot = np.array(
            [
                [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
                [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
                [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
            ],
            dtype=np.float64,
        )
        trans = np.array([tx, ty, tz], dtype=np.float64)
        points_base = (rot @ points_cam.T).T + trans
        z_base = points_base[:, 2]

        coarse_z = float(np.median(z_base))
        if self.z_trim_margin_m > 0.0:
            keep = np.abs(z_base - coarse_z) <= self.z_trim_margin_m
            if int(np.count_nonzero(keep)) >= self.min_points:
                points_base = points_base[keep]
                z_base = points_base[:, 2]

        if z_base.size < self.min_points:
            self.set_status(f"trim left too few points ({z_base.size})")
            return

        table_z = float(np.median(z_base))
        center_x = float(np.median(points_base[:, 0]))
        center_y = float(np.median(points_base[:, 1]))

        self.z_history.append(table_z)
        smoothed_z = float(np.median(np.array(self.z_history, dtype=np.float64)))

        z_msg = Float64()
        z_msg.data = smoothed_z
        self.z_pub.publish(z_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.latest_depth.header.stamp
        pose_msg.header.frame_id = self.target_frame
        pose_msg.pose.position.x = center_x
        pose_msg.pose.position.y = center_y
        pose_msg.pose.position.z = smoothed_z
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        self.set_status(
            f"table z={smoothed_z:+.3f}m points={z_base.size} center=({center_x:+.3f},{center_y:+.3f})"
        )

    def message_age_s(self, msg) -> Optional[float]:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return None
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return None
        return max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e9)

    def set_status(self, text: str) -> None:
        if text == self._last_status:
            return
        self._last_status = text
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TablePlaneEstimatorNode()
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
