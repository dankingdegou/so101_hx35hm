#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Diagnose SO101 HX35HM leader/follower teleop smoothness.

The script samples:
- /leader/joint_states
- /follower/forward_controller/commands
- /follower/joint_states

It reports message rates, effective value-change rates, and step sizes per
joint. This is meant to separate leader readback stair-steps from relay or
follower execution issues.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import statistics
import time
from dataclasses import dataclass, field
from typing import Dict, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


@dataclass
class TopicSamples:
    rows: list[tuple[float, list[float]]] = field(default_factory=list)


class TeleopSmoothnessSampler(Node):
    def __init__(self) -> None:
        super().__init__("so101_teleop_smoothness_sampler")
        self.samples: Dict[str, TopicSamples] = {
            "leader": TopicSamples(),
            "command": TopicSamples(),
            "follower": TopicSamples(),
        }
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/leader/joint_states", self._leader_cb, qos)
        self.create_subscription(Float64MultiArray, "/follower/forward_controller/commands", self._cmd_cb, qos)
        self.create_subscription(JointState, "/follower/joint_states", self._follower_cb, qos)

    def _leader_cb(self, msg: JointState) -> None:
        self._record_joint_state("leader", msg)

    def _follower_cb(self, msg: JointState) -> None:
        self._record_joint_state("follower", msg)

    def _cmd_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < len(JOINTS):
            return
        self.samples["command"].rows.append((time.monotonic(), [float(v) for v in msg.data[: len(JOINTS)]]))

    def _record_joint_state(self, key: str, msg: JointState) -> None:
        values: list[float] = []
        names = list(msg.name)
        for joint in JOINTS:
            try:
                idx = names.index(joint)
            except ValueError:
                return
            if idx >= len(msg.position):
                return
            values.append(float(msg.position[idx]))
        self.samples[key].rows.append((time.monotonic(), values))


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    idx = (len(ordered) - 1) * q
    low = math.floor(idx)
    high = math.ceil(idx)
    if low == high:
        return ordered[int(idx)]
    return ordered[low] * (high - idx) + ordered[high] * (idx - low)


def _safe_median(values: list[float]) -> float:
    return statistics.median(values) if values else float("nan")


def _topic_stats(rows: list[tuple[float, list[float]]], duration: float, eps: float) -> dict:
    result: dict = {
        "messages": len(rows),
        "message_hz": len(rows) / duration if duration > 0.0 else 0.0,
        "joints": {},
    }
    if len(rows) < 2:
        return result

    for joint_idx, joint in enumerate(JOINTS):
        change_times: list[float] = []
        steps: list[float] = []
        signed_steps: list[float] = []
        last_t, last_values = rows[0]
        last_v = last_values[joint_idx]
        for t, values in rows[1:]:
            v = values[joint_idx]
            delta = v - last_v
            if abs(delta) > eps:
                change_times.append(t)
                steps.append(abs(delta))
                signed_steps.append(delta)
                last_v = v
                last_t = t

        dts = [b - a for a, b in zip(change_times, change_times[1:]) if b > a]
        result["joints"][joint] = {
            "changes": len(steps),
            "change_hz": len(steps) / duration if duration > 0.0 else 0.0,
            "median_change_dt_s": _safe_median(dts),
            "median_step_rad": _safe_median(steps),
            "p95_step_rad": _percentile(steps, 0.95),
            "max_step_rad": max(steps) if steps else 0.0,
            "median_signed_step_rad": _safe_median(signed_steps),
        }
    return result


def _latest_error_stats(
    source_rows: list[tuple[float, list[float]]],
    target_rows: list[tuple[float, list[float]]],
    duration: float,
) -> dict:
    """Nearest-latest error: source(t) - latest target at or before t."""
    result = {"joints": {}}
    if not source_rows or not target_rows:
        return result

    target_idx = 0
    errors_by_joint = {joint: [] for joint in JOINTS}
    for source_t, source_values in source_rows:
        while target_idx + 1 < len(target_rows) and target_rows[target_idx + 1][0] <= source_t:
            target_idx += 1
        target_values = target_rows[target_idx][1]
        for idx, joint in enumerate(JOINTS):
            errors_by_joint[joint].append(source_values[idx] - target_values[idx])

    for joint, errors in errors_by_joint.items():
        abs_errors = [abs(e) for e in errors]
        result["joints"][joint] = {
            "median_abs_error_rad": _safe_median(abs_errors),
            "p95_abs_error_rad": _percentile(abs_errors, 0.95),
            "median_signed_error_rad": _safe_median(errors),
        }
    return result


def _print_summary(report: dict) -> None:
    print("SO101 teleop smoothness diagnostic")
    print(f"duration_s: {report['duration_s']:.2f}")
    print(f"eps_rad: {report['eps_rad']:.5f}")
    print("")

    for topic in ("leader", "command", "follower"):
        stats = report["topics"][topic]
        print(f"[{topic}] messages={stats['messages']} hz={stats['message_hz']:.1f}")
        print("joint                 change_hz  med_step_deg  p95_step_deg  max_step_deg")
        for joint in JOINTS:
            j = stats["joints"].get(joint, {})
            med = math.degrees(j.get("median_step_rad", float("nan")))
            p95 = math.degrees(j.get("p95_step_rad", float("nan")))
            max_step = math.degrees(j.get("max_step_rad", float("nan")))
            print(f"{joint:20s} {j.get('change_hz', 0.0):9.2f} {med:13.3f} {p95:13.3f} {max_step:13.3f}")
        print("")

    print("[command_vs_follower_latest_error]")
    print("joint                 med_abs_deg  p95_abs_deg  med_signed_deg")
    err = report["command_vs_follower_latest_error"]["joints"]
    for joint in JOINTS:
        j = err.get(joint, {})
        print(
            f"{joint:20s} "
            f"{math.degrees(j.get('median_abs_error_rad', float('nan'))):11.3f} "
            f"{math.degrees(j.get('p95_abs_error_rad', float('nan'))):11.3f} "
            f"{math.degrees(j.get('median_signed_error_rad', float('nan'))):14.3f}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sample and diagnose SO101 teleop smoothness.")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--eps-rad", type=float, default=0.0015)
    parser.add_argument(
        "--output",
        type=pathlib.Path,
        default=pathlib.Path("/tmp/so101_teleop_smoothness_report.json"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    duration = max(1.0, float(args.duration))
    rclpy.init()
    node = TeleopSmoothnessSampler()
    deadline = time.monotonic() + duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        samples = node.samples
        node.destroy_node()
        rclpy.shutdown()

    report = {
        "duration_s": duration,
        "eps_rad": float(args.eps_rad),
        "topics": {
            key: _topic_stats(value.rows, duration, float(args.eps_rad))
            for key, value in samples.items()
        },
        "command_vs_follower_latest_error": _latest_error_stats(
            samples["command"].rows,
            samples["follower"].rows,
            duration,
        ),
    }
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    _print_summary(report)
    print("")
    print(f"json_report: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
