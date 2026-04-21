#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def build_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    objp = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] = grid * square_size_m
    return objp


def save_camera_yaml(
    path: Path,
    camera_name: str,
    image_width: int,
    image_height: int,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rectification_matrix: np.ndarray,
    projection_matrix: np.ndarray,
) -> None:
    dist = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
    payload = {
        "image_width": int(image_width),
        "image_height": int(image_height),
        "camera_name": str(camera_name),
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(v) for v in camera_matrix.reshape(-1)],
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(dist.shape[0]),
            "data": [float(v) for v in dist.tolist()],
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [float(v) for v in rectification_matrix.reshape(-1)],
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": [float(v) for v in projection_matrix.reshape(-1)],
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def rotation_degrees(corners: np.ndarray) -> float:
    first = corners[0, 0]
    last = corners[-1, 0]
    dx = float(last[0] - first[0])
    dy = float(last[1] - first[1])
    return math.degrees(math.atan2(dy, dx))


class IntrinsicsCalibrator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("intrinsics_calibrator")
        self.args = args
        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_stamp_s = 0.0
        self.frame_lock = threading.Lock()
        self.samples: list[np.ndarray] = []
        self.objpoints: list[np.ndarray] = []
        self.imgpoints: list[np.ndarray] = []
        self.last_capture_signature: Optional[tuple[float, float, float, float]] = None
        self.calibrated = False
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.new_camera_matrix: Optional[np.ndarray] = None
        self.rms_error: Optional[float] = None
        self.mean_reproj_error: Optional[float] = None
        self.image_width = 0
        self.image_height = 0
        self.objp = build_object_points(args.cols, args.rows, args.square_size_mm / 1000.0)
        self.create_subscription(Image, args.image_topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        with self.frame_lock:
            self.latest_frame = frame
            self.latest_stamp_s = time.time()
            self.image_height, self.image_width = frame.shape[:2]

    def get_frame(self) -> Optional[np.ndarray]:
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def get_frame_age_s(self) -> float:
        with self.frame_lock:
            if self.latest_frame is None:
                return float("inf")
            return time.time() - self.latest_stamp_s

    def detect_board(self, frame: np.ndarray) -> tuple[bool, Optional[np.ndarray]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pattern = (self.args.cols, self.args.rows)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

        found = False
        corners = None
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(gray, pattern)
        if not found:
            found, corners = cv2.findChessboardCorners(gray, pattern, flags)
            if found and corners is not None:
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    40,
                    0.001,
                )
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        if not found or corners is None:
            return False, None
        return True, corners.astype(np.float32)

    def capture_sample(self, frame: np.ndarray) -> tuple[bool, str]:
        found, corners = self.detect_board(frame)
        if not found or corners is None:
            return False, "没有检测到棋盘格。"

        centroid = np.mean(corners[:, 0, :], axis=0)
        min_xy = np.min(corners[:, 0, :], axis=0)
        max_xy = np.max(corners[:, 0, :], axis=0)
        area = float(max(1.0, (max_xy[0] - min_xy[0]) * (max_xy[1] - min_xy[1])))
        angle = rotation_degrees(corners)
        signature = (
            round(float(centroid[0]) / 20.0, 1),
            round(float(centroid[1]) / 20.0, 1),
            round(area / 5000.0, 1),
            round(angle / 10.0, 1),
        )
        if signature == self.last_capture_signature:
            return False, "和上一帧视角太像了，请换一个更不同的角度再采。"

        self.objpoints.append(self.objp.copy())
        self.imgpoints.append(corners.copy())
        self.samples.append(frame.copy())
        self.last_capture_signature = signature
        return True, f"已采集样本 {len(self.samples)} 组。"

    def calibrate(self) -> tuple[bool, str]:
        if self.image_width <= 0 or self.image_height <= 0:
            return False, "还没有收到图像。"
        if len(self.imgpoints) < self.args.min_samples:
            return False, f"样本不足，至少需要 {self.args.min_samples} 组。"

        rms, k, d, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints,
            self.imgpoints,
            (self.image_width, self.image_height),
            None,
            None,
        )
        new_k, _roi = cv2.getOptimalNewCameraMatrix(
            k,
            d,
            (self.image_width, self.image_height),
            alpha=0.0,
            newImgSize=(self.image_width, self.image_height),
        )
        reproj_errors = []
        for objp, imgp, rvec, tvec in zip(self.objpoints, self.imgpoints, rvecs, tvecs):
            projected, _ = cv2.projectPoints(objp, rvec, tvec, k, d)
            err = cv2.norm(imgp, projected, cv2.NORM_L2) / len(projected)
            reproj_errors.append(float(err))

        self.camera_matrix = k
        self.dist_coeffs = d
        self.new_camera_matrix = new_k
        self.rms_error = float(rms)
        self.mean_reproj_error = float(np.mean(reproj_errors))
        self.calibrated = True

        proj = np.zeros((3, 4), dtype=np.float64)
        proj[:3, :3] = new_k
        save_camera_yaml(
            path=Path(self.args.output).expanduser().resolve(),
            camera_name=self.args.camera_name,
            image_width=self.image_width,
            image_height=self.image_height,
            camera_matrix=k,
            dist_coeffs=d,
            rectification_matrix=np.eye(3, dtype=np.float64),
            projection_matrix=proj,
        )
        return True, (
            f"标定完成，RMS={self.rms_error:.4f} px, "
            f"mean_reproj={self.mean_reproj_error:.4f} px, "
            f"已保存到 {Path(self.args.output).expanduser().resolve()}"
        )

    def draw_status(self, frame: np.ndarray) -> np.ndarray:
        vis = frame.copy()
        found, corners = self.detect_board(frame)
        if found and corners is not None:
            cv2.drawChessboardCorners(vis, (self.args.cols, self.args.rows), corners, True)

        lines = [
            f"topic={self.args.image_topic}  size={self.image_width}x{self.image_height}  age={self.get_frame_age_s():.2f}s",
            f"board={self.args.cols}x{self.args.rows}  square={self.args.square_size_mm:.1f}mm  samples={len(self.samples)}",
            "[S] capture  [C] calibrate+save  [U] undistort preview  [Q] quit",
        ]
        if self.calibrated and self.rms_error is not None and self.mean_reproj_error is not None:
            lines.append(f"RMS={self.rms_error:.4f}px  mean_reproj={self.mean_reproj_error:.4f}px")

        y = 28
        for line in lines:
            cv2.putText(vis, line, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (30, 230, 30), 2)
            y += 28
        return vis

    def maybe_undistort(self, frame: np.ndarray, enabled: bool) -> np.ndarray:
        if not enabled or not self.calibrated or self.camera_matrix is None or self.dist_coeffs is None:
            return frame
        return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, self.new_camera_matrix)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate ROS camera intrinsics from a live chessboard stream.")
    parser.add_argument("--image-topic", default="/static_camera/image_raw")
    parser.add_argument("--camera-name", default="cam_overhead")
    parser.add_argument("--cols", type=int, default=9, help="棋盘格内部角点列数")
    parser.add_argument("--rows", type=int, default=6, help="棋盘格内部角点行数")
    parser.add_argument("--square-size-mm", type=float, default=20.0, help="每个小格边长，单位 mm")
    parser.add_argument("--min-samples", type=int, default=18)
    parser.add_argument(
        "--output",
        default="~/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/cameras/cam_overhead_calib.yaml",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = IntrinsicsCalibrator(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cv2.namedWindow("camera_intrinsics_calibrator", cv2.WINDOW_NORMAL)
    undistort_preview = False
    print("")
    print("== 相机内参标定 ==")
    print(f"image_topic: {args.image_topic}")
    print(f"board: {args.cols}x{args.rows}, square={args.square_size_mm:.1f} mm")
    print(f"output: {Path(args.output).expanduser().resolve()}")
    print("热键: [S]采样  [C]标定并保存  [U]标定后查看去畸变预览  [Q]退出")
    print("")

    try:
        while rclpy.ok():
            frame = node.get_frame()
            if frame is None:
                waiting = np.zeros((480, 960, 3), dtype=np.uint8)
                cv2.putText(waiting, "Waiting for image topic ...", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                cv2.imshow("camera_intrinsics_calibrator", waiting)
            else:
                display = node.draw_status(frame)
                display = node.maybe_undistort(display, undistort_preview)
                cv2.imshow("camera_intrinsics_calibrator", display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S"), 32):
                frame = node.get_frame()
                if frame is None:
                    print("[WARN] 还没有收到图像。")
                    continue
                ok, msg = node.capture_sample(frame)
                print(("[OK] " if ok else "[WARN] ") + msg)
                time.sleep(0.12)
            if key in (ord("c"), ord("C")):
                ok, msg = node.calibrate()
                print(("[OK] " if ok else "[WARN] ") + msg)
                time.sleep(0.25)
            if key in (ord("u"), ord("U")):
                undistort_preview = not undistort_preview
                print(f"[INFO] undistort_preview={'ON' if undistort_preview else 'OFF'}")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
