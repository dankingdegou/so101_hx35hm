#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Auto-trim SO101 leader/follower teleop joint_offsets.

This script is intended for relative teleop mode. It does not change servo zero
positions or hardware calibration; it only edits so101_teleop/config/teleop.yaml
joint_offsets when --apply is provided.
"""

from __future__ import annotations

import argparse
import ast
import pathlib
import re
import statistics
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


DEFAULT_CONFIG = pathlib.Path(
    "/home/rog/ros2_ws/src/so101-ros-physical-ai/so101_teleop/config/teleop.yaml"
)
DEFAULT_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


@dataclass
class Sample:
    leader: Optional[float] = None
    follower: Optional[float] = None
    command: Optional[float] = None


class TrimSampler(Node):
    def __init__(self, joint: str, joint_index: int) -> None:
        super().__init__("so101_auto_trim_teleop_offset")
        self.joint = joint
        self.joint_index = joint_index
        self.latest = Sample()
        self.create_subscription(JointState, "/leader/joint_states", self._leader_cb, 10)
        self.create_subscription(JointState, "/follower/joint_states", self._follower_cb, 10)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Float64MultiArray,
            "/follower/forward_controller/commands",
            self._command_cb,
            command_qos,
        )

    def _leader_cb(self, msg: JointState) -> None:
        self.latest.leader = _joint_value(msg, self.joint)

    def _follower_cb(self, msg: JointState) -> None:
        self.latest.follower = _joint_value(msg, self.joint)

    def _command_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) > self.joint_index:
            self.latest.command = float(msg.data[self.joint_index])


def _joint_value(msg: JointState, joint: str) -> Optional[float]:
    try:
        idx = list(msg.name).index(joint)
    except ValueError:
        return None
    if len(msg.position) <= idx:
        return None
    return float(msg.position[idx])


def _load_joint_offsets(config: pathlib.Path) -> list[float]:
    text = config.read_text(encoding="utf-8")
    match = re.search(r"^(\s*joint_offsets:\s*)(\[[^\]]*\])", text, re.MULTILINE)
    if match is None:
        raise RuntimeError(f"joint_offsets not found in {config}")
    values = ast.literal_eval(match.group(2))
    if not isinstance(values, list) or not all(isinstance(v, (int, float)) for v in values):
        raise RuntimeError("joint_offsets must be a numeric YAML inline list")
    return [float(v) for v in values]


def _write_joint_offsets(config: pathlib.Path, offsets: list[float]) -> None:
    text = config.read_text(encoding="utf-8")
    formatted = "[" + ", ".join(f"{v:.6f}" for v in offsets) + "]"
    new_text, count = re.subn(
        r"^(\s*joint_offsets:\s*)\[[^\]]*\]",
        rf"\1{formatted}",
        text,
        count=1,
        flags=re.MULTILINE,
    )
    if count != 1:
        raise RuntimeError(f"Failed to update joint_offsets in {config}")
    config.write_text(new_text, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sample leader/follower teleop error and update one joint offset."
    )
    parser.add_argument("--config", type=pathlib.Path, default=DEFAULT_CONFIG)
    parser.add_argument("--joint", default="shoulder_pan", choices=DEFAULT_JOINTS)
    parser.add_argument(
        "--mode",
        choices=["match-leader", "match-command"],
        default="match-leader",
        help=(
            "match-leader trims follower toward leader joint value; "
            "match-command trims follower toward current relay command."
        ),
    )
    parser.add_argument("--samples", type=int, default=120)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--gain",
        type=float,
        default=0.6,
        help="Apply this fraction of the measured error to avoid overshoot.",
    )
    parser.add_argument(
        "--max-delta",
        type=float,
        default=0.20,
        help="Safety cap for a single offset update in radians.",
    )
    parser.add_argument("--apply", action="store_true", help="Write the new offset to teleop.yaml.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = args.config
    if not config.exists():
        raise SystemExit(f"Config not found: {config}")

    offsets = _load_joint_offsets(config)
    joint_index = DEFAULT_JOINTS.index(args.joint)
    if len(offsets) <= joint_index:
        raise SystemExit(f"joint_offsets has {len(offsets)} values, need at least {joint_index + 1}")

    rclpy.init()
    node = TrimSampler(args.joint, joint_index)

    errors: list[float] = []
    deadline = time.monotonic() + max(0.5, float(args.timeout))
    needed = max(5, int(args.samples))
    try:
        while rclpy.ok() and time.monotonic() < deadline and len(errors) < needed:
            rclpy.spin_once(node, timeout_sec=0.02)
            leader = node.latest.leader
            follower = node.latest.follower
            command = node.latest.command
            if follower is None:
                continue
            if args.mode == "match-leader":
                if leader is None:
                    continue
                error = leader - follower
            else:
                if command is None:
                    continue
                error = command - follower
            errors.append(float(error))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(errors) < 5:
        raise SystemExit(
            "Not enough samples. Make sure teleop is running and topics are publishing."
        )

    median_error = statistics.median(errors)
    delta = max(-float(args.max_delta), min(float(args.max_delta), median_error * float(args.gain)))
    old_offset = offsets[joint_index]
    new_offset = old_offset + delta
    new_offsets = list(offsets)
    new_offsets[joint_index] = new_offset

    print(f"config: {config}")
    print(f"joint: {args.joint} index={joint_index}")
    print(f"mode: {args.mode}")
    print(f"samples: {len(errors)}")
    print(f"median_error_rad: {median_error:+.6f}")
    print(f"gain: {float(args.gain):.3f}")
    print(f"applied_delta_rad: {delta:+.6f}")
    print(f"old_offset_rad: {old_offset:+.6f}")
    print(f"new_offset_rad: {new_offset:+.6f}")

    if args.apply:
        _write_joint_offsets(config, new_offsets)
        print("updated: yes")
    else:
        print("updated: no (--apply not provided)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
