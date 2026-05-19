#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Capture current HX-35HM servo positions and back-compute suggested joint_zero_positions
for the bridge config, assuming the arm is currently placed in a known SRDF pose.

Typical usage:
  python3 capture_zero_positions.py \
    --device /dev/so101_leader \
    --params /home/rog/ros2_ws/src/so101-ros-physical-ai/so101_bringup/config/hx35hm_leader_bridge_params.yaml \
    --pose rest

The arm must already be physically placed in the requested pose. This tool does
not move the arm; it only reads current servo positions and computes the
zero-position that would make the current pose evaluate to the requested SRDF pose.
"""

from __future__ import annotations

import argparse
import math
import statistics
import pathlib
import sys
import time
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple


JOINT_ID_MAP: Dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}


def _import_board():
    try:
        from ros_robot_controller.ros_robot_controller_sdk import Board  # type: ignore

        return Board
    except ModuleNotFoundError:
        this_file = pathlib.Path(__file__).resolve()
        ws_src = this_file.parents[2]
        candidate = ws_src / "ros_robot_controller-ros2" / "src" / "ros_robot_controller"
        sys.path.insert(0, str(candidate))
        from ros_robot_controller.ros_robot_controller_sdk import Board  # type: ignore

        return Board


def _load_yaml(path: pathlib.Path) -> dict:
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError("PyYAML is required for capture_zero_positions.py") from exc

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML root object in {path}")
    return data


def _default_srdf_path() -> pathlib.Path:
    this_file = pathlib.Path(__file__).resolve()
    ws_src = this_file.parents[2]
    return ws_src / "so101-ros-physical-ai" / "so101_moveit_config" / "config" / "so101_arm.srdf"


def _parse_srdf_group_states(srdf_path: pathlib.Path) -> Dict[str, Dict[str, float]]:
    if not srdf_path.exists():
        raise FileNotFoundError(f"SRDF not found: {srdf_path}")

    tree = ET.parse(srdf_path)
    root = tree.getroot()
    poses: Dict[str, Dict[str, float]] = {}
    for gs in root.findall("group_state"):
        name = gs.attrib.get("name", "")
        group = gs.attrib.get("group", "")
        if not name or group != "manipulator":
            continue
        joint_map: Dict[str, float] = {}
        for j in gs.findall("joint"):
            jname = j.attrib.get("name")
            value = j.attrib.get("value")
            if jname is None or value is None:
                continue
            try:
                joint_map[jname] = float(value)
            except ValueError:
                continue
        if joint_map:
            poses[name] = joint_map
    return poses


def _extract_params(cfg_path: pathlib.Path, namespace: str) -> Tuple[List[str], List[int], float, int, int]:
    cfg = _load_yaml(cfg_path)
    block = cfg.get(namespace, {})
    if not isinstance(block, dict):
        raise ValueError(f"Expected top-level key '{namespace}' in {cfg_path}")
    bridge = block.get("hx35hm_bridge", {})
    if not isinstance(bridge, dict):
        raise ValueError(f"Expected {namespace}.hx35hm_bridge in {cfg_path}")
    ros_params = bridge.get("ros__parameters", {})
    if not isinstance(ros_params, dict):
        raise ValueError(f"Expected {namespace}.hx35hm_bridge.ros__parameters in {cfg_path}")

    joint_names = list(ros_params.get("joint_names", list(JOINT_ID_MAP.keys())))
    directions = list(ros_params.get("joint_directions", [1, 1, 1, 1, 1, 1]))
    if len(directions) != len(joint_names):
        raise ValueError("joint_directions length does not match joint_names")

    servo_range_deg = float(ros_params.get("servo_range_deg", 240.0))
    servo_pos_min = int(ros_params.get("servo_pos_min", 0))
    servo_pos_max = int(ros_params.get("servo_pos_max", 1000))
    return joint_names, [int(x) for x in directions], servo_range_deg, servo_pos_min, servo_pos_max


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture current servo positions and compute suggested bridge joint_zero_positions."
    )
    parser.add_argument("--device", required=True, help="STM32 device, e.g. /dev/so101_leader")
    parser.add_argument("--params", type=pathlib.Path, required=True, help="Bridge params YAML to read directions from.")
    parser.add_argument("--namespace", required=True, help="Top-level namespace key in params YAML, e.g. leader or follower.")
    parser.add_argument("--pose", default="rest", help="Named SRDF pose that the arm is currently in.")
    parser.add_argument("--srdf", type=pathlib.Path, default=_default_srdf_path(), help="SRDF path.")
    parser.add_argument("--timeout", type=float, default=0.5, help="Read timeout seconds.")
    parser.add_argument("--delay", type=float, default=0.05, help="Inter-read delay seconds.")
    parser.add_argument("--rounds", type=int, default=5, help="Position read rounds per servo; median is used.")
    parser.add_argument(
        "--write-back",
        action="store_true",
        help="Write suggested joint_zero_positions back into the params YAML.",
    )
    return parser.parse_args()


def _read_position_median(board, servo_id: int, *, rounds: int, timeout: float, delay: float) -> float | None:
    samples: List[float] = []
    for _ in range(max(1, rounds)):
        pos = board.bus_servo_read_position(servo_id, timeout=float(timeout))
        if pos is not None:
            samples.append(float(pos[0]))
        time.sleep(float(delay))
    if not samples:
        return None
    return float(statistics.median(samples))


def _write_back_joint_zero_positions(
    cfg_path: pathlib.Path, namespace: str, joint_zero_positions: List[float]
) -> None:
    cfg = _load_yaml(cfg_path)
    block = cfg.get(namespace, {})
    if not isinstance(block, dict):
        raise ValueError(f"Expected top-level key '{namespace}' in {cfg_path}")
    bridge = block.get("hx35hm_bridge", {})
    if not isinstance(bridge, dict):
        raise ValueError(f"Expected {namespace}.hx35hm_bridge in {cfg_path}")
    ros_params = bridge.get("ros__parameters", {})
    if not isinstance(ros_params, dict):
        raise ValueError(f"Expected {namespace}.hx35hm_bridge.ros__parameters in {cfg_path}")

    ros_params["joint_zero_positions"] = [round(v, 2) if not math.isnan(v) else 500.0 for v in joint_zero_positions]

    try:
        import yaml  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError("PyYAML is required to write updated params files") from exc

    with cfg_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)


def main() -> int:
    args = parse_args()
    joint_names, directions, servo_range_deg, servo_pos_min, servo_pos_max = _extract_params(
        args.params, args.namespace
    )
    poses = _parse_srdf_group_states(args.srdf)
    if args.pose not in poses:
        raise SystemExit(f"Pose '{args.pose}' not found in {args.srdf}. Available: {', '.join(sorted(poses))}")

    target_pose = poses[args.pose]
    Board = _import_board()
    board = Board(device=str(args.device))
    board.enable_reception()
    time.sleep(0.2)

    span = float(servo_pos_max - servo_pos_min)
    if servo_range_deg <= 0.0 or span <= 0.0:
        raise SystemExit("Invalid servo range parameters in bridge config.")
    pos_per_deg = span / servo_range_deg

    rows = []
    suggested_zero_positions: List[float] = []
    for joint_name, direction in zip(joint_names, directions):
        if joint_name not in target_pose:
            # gripper is usually not in the named pose
            suggested_zero_positions.append(500.0)
            rows.append((joint_name, "-", "-", "-", "skipped"))
            continue

        servo_id = JOINT_ID_MAP[joint_name]
        current_pos = _read_position_median(
            board,
            servo_id,
            rounds=int(args.rounds),
            timeout=float(args.timeout),
            delay=float(args.delay),
        )
        if current_pos is None:
            suggested_zero_positions.append(float("nan"))
            rows.append((joint_name, servo_id, target_pose[joint_name], None, "read_failed"))
            continue

        target_deg = target_pose[joint_name] * 180.0 / math.pi
        zero_pos = current_pos - int(direction) * (target_deg * pos_per_deg)
        suggested_zero_positions.append(zero_pos)
        rows.append((joint_name, servo_id, target_pose[joint_name], current_pos, zero_pos))

    print(f"device: {args.device}")
    print(f"params: {args.params}")
    print(f"namespace: {args.namespace}")
    print(f"assumed_pose: {args.pose}")
    print("")
    print("per-joint capture:")
    for row in rows:
        joint_name, servo_id, target_rad, current_pos, zero_pos = row
        if zero_pos == "skipped":
            print(f"  {joint_name:14s} skipped (not present in SRDF pose)")
            continue
        if zero_pos == "read_failed":
            print(f"  {joint_name:14s} servo_id={servo_id} target_rad={target_rad:+.5f} read_failed")
            continue
        print(
            f"  {joint_name:14s} servo_id={servo_id} target_rad={float(target_rad):+.5f} "
            f"current_pos={float(current_pos):7.2f} -> suggested_zero_pos={float(zero_pos):7.2f}"
        )

    print("")
    print("suggested joint_zero_positions:")
    print("  [" + ", ".join("500.0" if math.isnan(x) else f"{x:.2f}" for x in suggested_zero_positions) + "]")

    if args.write_back:
        _write_back_joint_zero_positions(args.params, args.namespace, suggested_zero_positions)
        print("")
        print(f"updated params file: {args.params}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
