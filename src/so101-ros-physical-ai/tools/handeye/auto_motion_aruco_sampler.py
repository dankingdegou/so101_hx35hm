#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Conservative upper-workspace sampling band.
# The arm sits on a table, so we keep the motion close to the folded safe pose
# and avoid large downward sweeps that could hit the tabletop.
HOME = np.array([0.00, -1.56, 1.58, 0.75, 0.00, 0.00], dtype=np.float64)
SAFE_DELTA = np.array([0.75, 0.18, 0.18, 0.16, 1.20, 0.00], dtype=np.float64)
SAFE_MIN = HOME - SAFE_DELTA
SAFE_MAX = HOME + SAFE_DELTA
GRIPPER_MIN = -0.20
GRIPPER_MAX = 1.60
ANCHOR_OFFSETS = [
    np.array([0.00, 0.00, 0.00, 0.00, 0.00, 0.00], dtype=np.float64),
    np.array([0.42, -0.06, 0.10, -0.10, 0.75, 0.00], dtype=np.float64),
    np.array([-0.42, -0.06, 0.10, -0.10, -0.75, 0.00], dtype=np.float64),
    np.array([0.52, 0.06, -0.08, 0.10, 1.05, 0.00], dtype=np.float64),
    np.array([-0.52, 0.06, -0.08, 0.10, -1.05, 0.00], dtype=np.float64),
    np.array([0.22, 0.10, -0.14, 0.12, 0.20, 0.00], dtype=np.float64),
    np.array([-0.22, 0.10, -0.14, 0.12, -0.20, 0.00], dtype=np.float64),
    np.array([0.58, -0.02, 0.02, -0.02, -0.95, 0.00], dtype=np.float64),
    np.array([-0.58, -0.02, 0.02, -0.02, 0.95, 0.00], dtype=np.float64),
]


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
    target_joint_rad: list[float]
    actual_joint_rad: list[float]
    marker_pose_age_s: float
    marker_bin: str


class AutoSampler(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("auto_motion_aruco_sampler")
        self.args = args

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Float64MultiArray, args.command_topic, 10)

        self.latest_joint_lock = threading.Lock()
        self.latest_joint_pos: dict[str, float] = {}
        self.latest_pose_lock = threading.Lock()
        self.latest_marker_pose: Optional[PoseStamped] = None
        self.latest_marker_recv_time = 0.0
        self.latest_marker_seq = 0
        self.samples: list[Sample] = []
        self.bin_counts: Counter[str] = Counter()

        self.create_subscription(JointState, args.joint_state_topic, self.on_joint_state, 10)
        self.create_subscription(PoseStamped, args.pose_topic, self.on_pose, 10)

    def on_joint_state(self, msg: JointState) -> None:
        with self.latest_joint_lock:
            for n, p in zip(msg.name, msg.position):
                self.latest_joint_pos[n] = float(p)

    def on_pose(self, msg: PoseStamped) -> None:
        with self.latest_pose_lock:
            self.latest_marker_pose = msg
            self.latest_marker_recv_time = time.time()
            self.latest_marker_seq += 1

    def get_current_joint_vec(self) -> Optional[np.ndarray]:
        with self.latest_joint_lock:
            try:
                arr = np.array([self.latest_joint_pos[n] for n in JOINT_NAMES], dtype=np.float64)
            except KeyError:
                return None
        return arr

    def get_latest_pose(self) -> Optional[PoseStamped]:
        with self.latest_pose_lock:
            return self.latest_marker_pose

    def get_latest_pose_age(self) -> float:
        with self.latest_pose_lock:
            if self.latest_marker_pose is None:
                return float("inf")
            return time.time() - self.latest_marker_recv_time

    def get_latest_pose_seq(self) -> int:
        with self.latest_pose_lock:
            return int(self.latest_marker_seq)

    def get_tool_in_base(self) -> Optional[PoseData]:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                self.args.tool_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.08),
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

    def send_joint_cmd(self, q: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q.tolist()]
        self.cmd_pub.publish(msg)

    def move_to(self, target: np.ndarray) -> bool:
        now = self.get_current_joint_vec()
        if now is None:
            self.get_logger().warn("No joint_states yet.")
            return False

        diff = target - now
        max_delta = float(np.max(np.abs(diff)))
        steps = max(1, int(math.ceil(max_delta / max(1e-4, self.args.max_step_rad))))
        for i in range(1, steps + 1):
            alpha = i / steps
            q = now + alpha * diff
            self.send_joint_cmd(q)
            time.sleep(self.args.step_dt_s)

        t0 = time.time()
        while time.time() - t0 < self.args.reach_timeout_s:
            cur = self.get_current_joint_vec()
            if cur is not None and float(np.max(np.abs(cur - target))) <= self.args.reach_tol_rad:
                return True
            self.send_joint_cmd(target)
            time.sleep(0.06)
        return False

    def wait_for_fresh_pose(self, min_seq: int, timeout_s: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self.get_latest_pose_seq() >= min_seq and self.get_latest_pose_age() <= self.args.max_pose_age_s:
                return True
            time.sleep(0.05)
        return False

    @staticmethod
    def _bin_index(value: float, lo: float, hi: float, bins: int) -> Optional[int]:
        if bins <= 0 or value < lo or value > hi:
            return None
        if bins == 1 or hi <= lo:
            return 0
        ratio = (value - lo) / (hi - lo)
        idx = int(ratio * bins)
        return max(0, min(bins - 1, idx))

    def get_marker_bin(self, marker_xyz: np.ndarray) -> Optional[str]:
        ix = self._bin_index(marker_xyz[0], self.args.marker_x_min, self.args.marker_x_max, self.args.marker_x_bins)
        iy = self._bin_index(marker_xyz[1], self.args.marker_y_min, self.args.marker_y_max, self.args.marker_y_bins)
        iz = self._bin_index(marker_xyz[2], self.args.marker_z_min, self.args.marker_z_max, self.args.marker_z_bins)
        if ix is None or iy is None or iz is None:
            return None
        return f"x{ix}_y{iy}_z{iz}"

    def marker_is_novel_enough(self, marker_xyz: np.ndarray) -> bool:
        if not self.samples:
            return True
        for s in self.samples:
            prev = np.asarray(s.marker_in_camera.position_xyz, dtype=np.float64)
            if float(np.linalg.norm(marker_xyz - prev)) < self.args.min_marker_separation_m:
                return False
        return True

    def tool_is_novel_enough(self, tool_xyz: np.ndarray) -> bool:
        if not self.samples:
            return True
        for s in self.samples:
            prev = np.asarray(s.tool_in_base.position_xyz, dtype=np.float64)
            if float(np.linalg.norm(tool_xyz - prev)) < self.args.min_tool_separation_m:
                return False
        return True

    def capture_sample(self, target: np.ndarray) -> tuple[bool, str]:
        pose = self.get_latest_pose()
        if pose is None:
            return False, "no marker pose"
        pose_age = self.get_latest_pose_age()
        if pose_age > self.args.max_pose_age_s:
            return False, f"pose stale age={pose_age:.2f}s"

        tool = self.get_tool_in_base()
        if tool is None:
            return False, "no tool tf"
        if tool.position_xyz[2] < self.args.min_tool_z:
            return False, f"tool z too low ({tool.position_xyz[2]:.3f}m < {self.args.min_tool_z:.3f}m)"

        actual = self.get_current_joint_vec()
        if actual is None:
            return False, "no joint states"

        p = pose.pose.position
        q = pose.pose.orientation
        marker = PoseData(
            frame_id=pose.header.frame_id,
            position_xyz=[float(p.x), float(p.y), float(p.z)],
            quaternion_xyzw=[float(q.x), float(q.y), float(q.z), float(q.w)],
        )
        marker_xyz = np.asarray(marker.position_xyz, dtype=np.float64)
        tool_xyz = np.asarray(tool.position_xyz, dtype=np.float64)
        marker_bin = self.get_marker_bin(marker_xyz)
        if marker_bin is None:
            return (
                False,
                (
                    "marker out of desired camera range "
                    f"(x={marker_xyz[0]:+.3f}, y={marker_xyz[1]:+.3f}, z={marker_xyz[2]:+.3f})"
                ),
            )
        if self.bin_counts[marker_bin] >= self.args.max_samples_per_bin:
            return False, f"camera bin {marker_bin} already full ({self.bin_counts[marker_bin]})"
        if not self.marker_is_novel_enough(marker_xyz):
            return False, "marker pose too close to previous samples"
        if not self.tool_is_novel_enough(tool_xyz):
            return False, "tool pose too close to previous samples"

        s = Sample(
            index=len(self.samples) + 1,
            unix_time=time.time(),
            marker_in_camera=marker,
            tool_in_base=tool,
            target_joint_rad=[float(v) for v in target.tolist()],
            actual_joint_rad=[float(v) for v in actual.tolist()],
            marker_pose_age_s=float(pose_age),
            marker_bin=marker_bin,
        )
        self.samples.append(s)
        self.bin_counts[marker_bin] += 1
        return (
            True,
            (
                f"sample#{s.index} bin={marker_bin} "
                f"marker=({marker.position_xyz[0]:+.3f},{marker.position_xyz[1]:+.3f},{marker.position_xyz[2]:+.3f}) "
                f"tool=({tool.position_xyz[0]:+.3f},{tool.position_xyz[1]:+.3f},{tool.position_xyz[2]:+.3f})"
            ),
        )

    def save_json(self) -> Path:
        out = Path(self.args.out).expanduser().resolve()
        payload = {
            "meta": {
                "tool": "auto_motion_aruco_sampler.py",
                "count": len(self.samples),
                "pose_topic": self.args.pose_topic,
                "joint_state_topic": self.args.joint_state_topic,
                "command_topic": self.args.command_topic,
                "base_frame": self.args.base_frame,
                "tool_frame": self.args.tool_frame,
                "marker_x_range_m": [self.args.marker_x_min, self.args.marker_x_max],
                "marker_y_range_m": [self.args.marker_y_min, self.args.marker_y_max],
                "marker_z_range_m": [self.args.marker_z_min, self.args.marker_z_max],
                "marker_bins": [
                    self.args.marker_x_bins,
                    self.args.marker_y_bins,
                    self.args.marker_z_bins,
                ],
                "max_samples_per_bin": self.args.max_samples_per_bin,
            },
            "samples": [asdict(s) for s in self.samples],
        }
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return out


def generate_candidate(
    rng: np.random.Generator,
    range_scale: float,
    fixed_gripper: float,
    recent_targets: list[np.ndarray],
    min_joint_separation_rad: float,
    anchor_index: int,
    noise_scale: float,
) -> np.ndarray:
    delta = SAFE_DELTA * float(range_scale)
    lo = HOME - delta
    hi = HOME + delta
    anchor = HOME + ANCHOR_OFFSETS[anchor_index % len(ANCHOR_OFFSETS)]
    anchor = np.clip(anchor, lo, hi)
    for _ in range(240):
        jitter = rng.normal(0.0, delta * float(noise_scale), size=HOME.shape)
        q = anchor + jitter
        q = np.clip(q, lo, hi)
        q[5] = float(np.clip(fixed_gripper, GRIPPER_MIN, GRIPPER_MAX))
        if all(float(np.linalg.norm(q[:5] - p[:5])) >= min_joint_separation_rad for p in recent_targets[-10:]):
            return q
    q = anchor.copy()
    q[5] = float(np.clip(fixed_gripper, GRIPPER_MIN, GRIPPER_MAX))
    return q


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Auto move robot through many joint poses and collect camera-distributed ArUco hand-eye samples."
    )
    p.add_argument("--command-topic", default="/follower/forward_controller/commands")
    p.add_argument("--joint-state-topic", default="/follower/joint_states")
    p.add_argument("--pose-topic", default="/vision/aruco/pose_camera")
    p.add_argument("--base-frame", default="base_link")
    p.add_argument("--tool-frame", default="moving_jaw_so101_v1_link")
    p.add_argument("--samples-target", type=int, default=30, help="有效样本目标数")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--out", default="aruco_handeye_samples.json")
    p.add_argument("--max-attempts", type=int, default=180, help="最大运动尝试次数")
    p.add_argument("--max-step-rad", type=float, default=0.08)
    p.add_argument("--step-dt-s", type=float, default=0.18)
    p.add_argument("--reach-timeout-s", type=float, default=8.0)
    p.add_argument("--reach-tol-rad", type=float, default=0.06)
    p.add_argument("--settle-s", type=float, default=0.90)
    p.add_argument("--max-pose-age-s", type=float, default=0.30)
    p.add_argument("--pose-wait-s", type=float, default=4.0)
    p.add_argument("--fresh-pose-count", type=int, default=2, help="每次运动后至少等待这么多帧新的 pose 更新")
    p.add_argument("--min-tool-z", type=float, default=0.11)
    p.add_argument("--range-scale", type=float, default=1.0)
    p.add_argument("--min-joint-separation-rad", type=float, default=0.22)
    p.add_argument("--anchor-noise-scale", type=float, default=0.18, help="锚点姿态周围的随机扰动比例")
    p.add_argument("--min-marker-separation-m", type=float, default=0.04)
    p.add_argument("--min-tool-separation-m", type=float, default=0.03)
    p.add_argument("--marker-x-min", type=float, default=-0.20)
    p.add_argument("--marker-x-max", type=float, default=0.10)
    p.add_argument("--marker-y-min", type=float, default=-0.16)
    p.add_argument("--marker-y-max", type=float, default=0.10)
    p.add_argument("--marker-z-min", type=float, default=0.16)
    p.add_argument("--marker-z-max", type=float, default=0.45)
    p.add_argument("--marker-x-bins", type=int, default=4)
    p.add_argument("--marker-y-bins", type=int, default=3)
    p.add_argument("--marker-z-bins", type=int, default=3)
    p.add_argument("--max-samples-per-bin", type=int, default=2)
    p.add_argument("--fixed-gripper-rad", type=float, default=None, help="固定夹爪开度；默认读取当前 gripper")
    p.add_argument("--max-invalid-streak", type=int, default=10, help="连续无效样本达到该次数后自动回 HOME")
    p.add_argument("--home-recover-wait-s", type=float, default=0.8, help="回 HOME 后额外等待时间")
    p.add_argument("--go-home-first", action="store_true", default=True)
    p.add_argument("--no-go-home-first", dest="go_home_first", action="store_false")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = AutoSampler(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("")
    print("== 自动多位置采样 ==")
    print(f"目标样本数: {args.samples_target}")
    print(f"输出文件: {Path(args.out).expanduser().resolve()}")
    print(
        "相机采样范围: "
        f"x[{args.marker_x_min:+.2f},{args.marker_x_max:+.2f}] "
        f"y[{args.marker_y_min:+.2f},{args.marker_y_max:+.2f}] "
        f"z[{args.marker_z_min:+.2f},{args.marker_z_max:+.2f}]"
    )
    print("")

    try:
        rng = np.random.default_rng(args.seed)
        invalid_streak = 0
        # Wait for basic streams.
        t0 = time.time()
        while time.time() - t0 < 12.0:
            if node.get_current_joint_vec() is not None:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError(f"joint_state_topic='{args.joint_state_topic}' 未就绪。")

        current = node.get_current_joint_vec()
        if current is None:
            raise RuntimeError("joint states unavailable after startup wait.")
        fixed_gripper = float(current[5]) if args.fixed_gripper_rad is None else float(args.fixed_gripper_rad)
        recent_targets: list[np.ndarray] = []
        home = HOME.copy()
        home[5] = fixed_gripper

        if args.go_home_first:
            print("[INFO] moving to HOME ...")
            node.move_to(home)
            time.sleep(0.6)

        for idx in range(1, args.max_attempts + 1):
            if len(node.samples) >= args.samples_target:
                break
            pose_seq_before = node.get_latest_pose_seq()
            q = generate_candidate(
                rng=rng,
                range_scale=args.range_scale,
                fixed_gripper=fixed_gripper,
                recent_targets=recent_targets,
                min_joint_separation_rad=args.min_joint_separation_rad,
                anchor_index=idx - 1,
                noise_scale=args.anchor_noise_scale,
            )
            recent_targets.append(q.copy())
            print(f"[INFO] attempt {idx}/{args.max_attempts}  samples={len(node.samples)}/{args.samples_target}")
            ok_move = node.move_to(q)
            if not ok_move:
                print("[WARN] reach timeout, continue to next waypoint.")
            time.sleep(args.settle_s)

            needed_seq = pose_seq_before + max(1, args.fresh_pose_count)
            got_fresh_pose = node.wait_for_fresh_pose(needed_seq, args.pose_wait_s)
            if not got_fresh_pose:
                print("[WARN] no fresh marker pose after motion settle")

            ok, msg = node.capture_sample(q)
            print(("[OK] " if ok else "[WARN] ") + msg)
            if ok:
                invalid_streak = 0
                print(f"[INFO] bin occupancy: {dict(sorted(node.bin_counts.items()))}")
            else:
                invalid_streak += 1
                if invalid_streak >= args.max_invalid_streak:
                    print(
                        f"[INFO] invalid streak reached {invalid_streak}, recovering to HOME and waiting {args.home_recover_wait_s:.2f}s ..."
                    )
                    node.move_to(home)
                    time.sleep(args.home_recover_wait_s)
                    invalid_streak = 0

        out = node.save_json()
        print("")
        print(f"已保存 {len(node.samples)} 组样本到: {out}")
        print(f"bin occupancy: {dict(sorted(node.bin_counts.items()))}")
    except (KeyboardInterrupt, ExternalShutdownException):
        out = node.save_json()
        print(f"\n中断，已保存 {len(node.samples)} 组样本到: {out}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
