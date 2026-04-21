#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class PoseData:
    frame_id: str
    position_xyz: list[float]
    quaternion_xyzw: list[float]


@dataclass
class Sample:
    index: int
    unix_time: float
    marker_in_camera: PoseData
    tool_in_base: PoseData


class ArucoHandeyeCollector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("aruco_handeye_collector")
        self.args = args

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_pose_lock = threading.Lock()
        self.latest_marker_pose_cam: Optional[PoseStamped] = None
        self.latest_marker_pose_recv_time: float = 0.0
        self.latest_debug_image: Optional[Image] = None
        self.samples: list[Sample] = []

        self.create_subscription(PoseStamped, args.pose_topic, self.on_pose, 10)
        self.create_subscription(Image, args.debug_image_topic, self.on_debug, qos_profile_sensor_data)

    def on_pose(self, msg: PoseStamped) -> None:
        with self.latest_pose_lock:
            self.latest_marker_pose_cam = msg
            self.latest_marker_pose_recv_time = time.time()

    def on_debug(self, msg: Image) -> None:
        self.latest_debug_image = msg

    def get_latest_pose(self) -> Optional[PoseStamped]:
        with self.latest_pose_lock:
            return self.latest_marker_pose_cam

    def get_latest_pose_age(self) -> float:
        with self.latest_pose_lock:
            if self.latest_marker_pose_cam is None:
                return float("inf")
            return time.time() - self.latest_marker_pose_recv_time

    def get_tool_in_base(self) -> Optional[PoseData]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.args.tool_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        return PoseData(
            frame_id=self.args.base_frame,
            position_xyz=[float(t.x), float(t.y), float(t.z)],
            quaternion_xyzw=[float(q.x), float(q.y), float(q.z), float(q.w)],
        )

    def save_sample(self) -> tuple[bool, str]:
        marker_pose = self.get_latest_pose()
        if marker_pose is None:
            return False, "没有收到 marker pose（检查 /vision/aruco/pose_camera）"
        pose_age = self.get_latest_pose_age()
        if pose_age > self.args.max_pose_age_s:
            return False, f"marker pose 过期（age={pose_age:.2f}s），请等检测恢复后再按 S。"

        tool_pose = self.get_tool_in_base()
        if tool_pose is None:
            return False, f"TF 不可用：{self.args.base_frame} <- {self.args.tool_frame}"

        p = marker_pose.pose.position
        q = marker_pose.pose.orientation
        marker_data = PoseData(
            frame_id=marker_pose.header.frame_id,
            position_xyz=[float(p.x), float(p.y), float(p.z)],
            quaternion_xyzw=[float(q.x), float(q.y), float(q.z), float(q.w)],
        )

        sample = Sample(
            index=len(self.samples) + 1,
            unix_time=time.time(),
            marker_in_camera=marker_data,
            tool_in_base=tool_pose,
        )
        self.samples.append(sample)
        return True, (
            f"sample#{sample.index} "
            f"marker_cam=({marker_data.position_xyz[0]:.3f},{marker_data.position_xyz[1]:.3f},{marker_data.position_xyz[2]:.3f}) "
            f"tool_base=({tool_pose.position_xyz[0]:.3f},{tool_pose.position_xyz[1]:.3f},{tool_pose.position_xyz[2]:.3f})"
        )

    def save_json(self) -> Path:
        out = Path(self.args.out).expanduser().resolve()
        payload = {
            "meta": {
                "tool": "collect_aruco_handeye_samples.py",
                "count": len(self.samples),
                "pose_topic": self.args.pose_topic,
                "debug_image_topic": self.args.debug_image_topic,
                "base_frame": self.args.base_frame,
                "tool_frame": self.args.tool_frame,
            },
            "samples": [asdict(s) for s in self.samples],
        }
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return out


def msg_image_to_bgr(msg: Image) -> Optional[np.ndarray]:
    if msg.height <= 0 or msg.width <= 0:
        return None
    if msg.encoding not in ("rgb8", "bgr8"):
        return None
    row = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = row[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Collect hand-eye samples from ArUco marker + robot TF.")
    parser.add_argument("--pose-topic", default="/vision/aruco/pose_camera")
    parser.add_argument("--debug-image-topic", default="/vision/aruco/debug_image")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--tool-frame", default="moving_jaw_so101_v1_link")
    parser.add_argument("--out", default="aruco_handeye_samples.json")
    parser.add_argument("--max-pose-age-s", type=float, default=0.30)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ArucoHandeyeCollector(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cv2.namedWindow("aruco_handeye_sampler", cv2.WINDOW_NORMAL)
    print("")
    print("== ArUco 手眼采样 ==")
    print("热键: [S]采样  [Q]保存并退出")
    print("建议: 至少 20 组，姿态/空间分布尽量丰富")
    print("")

    try:
        while rclpy.ok():
            frame = np.zeros((480, 800, 3), dtype=np.uint8)
            dbg = node.latest_debug_image
            if dbg is not None:
                bgr = msg_image_to_bgr(dbg)
                if bgr is not None:
                    frame = bgr

            ok_marker = node.get_latest_pose() is not None
            pose_age = node.get_latest_pose_age()
            ok_tf = node.get_tool_in_base() is not None
            status = (
                f"marker_pose={'OK' if ok_marker else 'NO'} age={pose_age:.2f}s "
                f"tool_tf={'OK' if ok_tf else 'NO'} samples={len(node.samples)}"
            )
            color = (0, 220, 0) if (ok_marker and ok_tf) else (0, 0, 255)
            cv2.putText(frame, status, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(frame, "[S]=capture  [Q]=save+quit", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
            cv2.imshow("aruco_handeye_sampler", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S"), 32, 13):
                ok, msg = node.save_sample()
                print(("[OK] " if ok else "[WARN] ") + msg)
                time.sleep(0.15)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        out = node.save_json()
        print(f"\n已保存 {len(node.samples)} 组样本到: {out}")
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
