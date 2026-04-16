#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SO101 assembly helper for HX-35HM bus servos.

Features:
- Single servo positioning: `--servo-id X --pos Y`
- Named robot poses from MoveIt SRDF: `--pose rest|zero|extended`
  Reads joint values from `so101_moveit_config/config/so101_arm.srdf`.
- Uses a YAML config for mapping and calibration:
  `so101_hx35hm_bridge/config/assembly_calibration.yaml`

Safety defaults:
- Prints computed targets before sending
- Prompts for confirmation unless `--yes` is given
- Supports `--dry-run`

Designed to run directly from the source tree (no sourced overlay required),
but it also works after `colcon build` + `source install/setup.bash`.
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple


def _import_board():
    try:
        from ros_robot_controller.ros_robot_controller_sdk import Board  # type: ignore

        return Board
    except ModuleNotFoundError:
        # Allow running without a sourced overlay by adding the in-tree package path.
        this_file = pathlib.Path(__file__).resolve()
        ws_src = this_file.parents[2]  # .../ros2_ws/src
        candidate = ws_src / "ros_robot_controller-ros2" / "src" / "ros_robot_controller"
        sys.path.insert(0, str(candidate))
        from ros_robot_controller.ros_robot_controller_sdk import Board  # type: ignore

        return Board


def _load_yaml(path: pathlib.Path) -> dict:
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "PyYAML is not installed. Install it or run inside a ROS/Pixi environment with yaml."
        ) from exc

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML root object in {path}")
    return data


def _try_get_package_share_dir(package_name: str) -> Optional[pathlib.Path]:
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore
    except Exception:  # noqa: BLE001
        return None
    try:
        return pathlib.Path(get_package_share_directory(package_name))
    except Exception:  # noqa: BLE001
        return None


def _default_srdf_path() -> pathlib.Path:
    # 1) Prefer ament index if sourced.
    share = _try_get_package_share_dir("so101_moveit_config")
    if share is not None:
        candidate = share / "config" / "so101_arm.srdf"
        if candidate.exists():
            return candidate

    # 2) Fallback to in-tree layout: ~/ros2_ws/src/so101-ros-physical-ai/so101_moveit_config/config/so101_arm.srdf
    this_file = pathlib.Path(__file__).resolve()
    ws_src = this_file.parents[2]  # .../ros2_ws/src
    candidate = (
        ws_src
        / "so101-ros-physical-ai"
        / "so101_moveit_config"
        / "config"
        / "so101_arm.srdf"
    )
    return candidate


@dataclass(frozen=True)
class ServoSpec:
    device: str
    pos_min: int
    pos_max: int
    range_deg: float
    default_duration_s: float


@dataclass(frozen=True)
class JointCalib:
    servo_id: int
    direction: int
    zero_pos: float


@dataclass(frozen=True)
class GripperSpec:
    neutral_pos: int
    open_rad: float
    closed_rad: float


def _parse_config(cfg_path: pathlib.Path) -> Tuple[ServoSpec, Dict[str, JointCalib], GripperSpec]:
    cfg = _load_yaml(cfg_path)

    servo = cfg.get("servo", {})
    if not isinstance(servo, dict):
        raise ValueError("config: 'servo' must be a mapping")

    device = str(servo.get("device", "/dev/ros_robot_controller"))
    pos_min = int(servo.get("pos_min", 0))
    pos_max = int(servo.get("pos_max", 1000))
    range_deg = float(servo.get("range_deg", 240.0))
    default_duration_s = float(servo.get("default_duration_s", 1.0))
    servo_spec = ServoSpec(
        device=device,
        pos_min=pos_min,
        pos_max=pos_max,
        range_deg=range_deg,
        default_duration_s=default_duration_s,
    )

    joints_cfg = cfg.get("joints", {})
    if not isinstance(joints_cfg, dict) or not joints_cfg:
        raise ValueError("config: 'joints' must be a non-empty mapping")

    joints: Dict[str, JointCalib] = {}
    for joint_name, entry in joints_cfg.items():
        if not isinstance(entry, dict):
            raise ValueError(f"config: joints.{joint_name} must be a mapping")
        servo_id = int(entry.get("servo_id"))
        direction = int(entry.get("direction", 1))
        zero_pos = float(entry.get("zero_pos", 500.0))
        if direction not in (-1, 1):
            raise ValueError(f"config: joints.{joint_name}.direction must be +1 or -1")
        joints[str(joint_name)] = JointCalib(servo_id=servo_id, direction=direction, zero_pos=zero_pos)

    gripper_cfg = cfg.get("gripper", {})
    if not isinstance(gripper_cfg, dict):
        gripper_cfg = {}
    gripper = GripperSpec(
        neutral_pos=int(gripper_cfg.get("neutral_pos", 500)),
        open_rad=float(gripper_cfg.get("open_rad", 1.5)),
        closed_rad=float(gripper_cfg.get("closed_rad", -0.16)),
    )

    if servo_spec.pos_max <= servo_spec.pos_min:
        raise ValueError("config: servo.pos_max must be > servo.pos_min")
    if servo_spec.range_deg <= 0.0:
        raise ValueError("config: servo.range_deg must be > 0")

    return servo_spec, joints, gripper


def _parse_srdf_group_states(srdf_path: pathlib.Path) -> Dict[str, Dict[str, float]]:
    if not srdf_path.exists():
        raise FileNotFoundError(f"SRDF not found: {srdf_path}")

    tree = ET.parse(srdf_path)
    root = tree.getroot()

    # We care about group_state for group="manipulator" (arm joints).
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


def _rad_to_servo_pos(
    joint_rad: float, *, calib: JointCalib, servo_spec: ServoSpec
) -> int:
    span = float(servo_spec.pos_max - servo_spec.pos_min)
    pos_per_deg = span / servo_spec.range_deg
    joint_deg = joint_rad * 180.0 / math.pi
    pos = calib.zero_pos + calib.direction * (joint_deg * pos_per_deg)
    pos = max(float(servo_spec.pos_min), min(float(servo_spec.pos_max), pos))
    return int(round(pos))


def _format_table(rows: List[Tuple[str, int, float, int]]) -> str:
    # (joint, servo_id, rad, pos)
    joint_w = max(len(r[0]) for r in rows) if rows else 5
    header = f"{'joint'.ljust(joint_w)}  servo_id  rad        pos"
    lines = [header, "-" * len(header)]
    for joint, servo_id, rad, pos in rows:
        lines.append(f"{joint.ljust(joint_w)}  {str(servo_id).rjust(7)}  {rad: .5f}  {str(pos).rjust(4)}")
    return "\n".join(lines)


def _confirm_or_exit(prompt: str, *, assume_yes: bool) -> None:
    if assume_yes:
        return
    print(prompt)
    ans = input("Type YES to continue: ").strip()
    if ans != "YES":
        raise SystemExit("Canceled.")


def parse_args() -> argparse.Namespace:
    default_cfg = (
        pathlib.Path(__file__).resolve().parents[1] / "config" / "assembly_calibration.yaml"
    )
    parser = argparse.ArgumentParser(
        description="SO101 assembly helper for HX-35HM (single servo or SRDF pose)."
    )

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--pose", help="Named pose from SRDF (e.g. rest, zero, extended).")
    mode.add_argument("--servo-id", type=int, help="Single servo id to command.")

    parser.add_argument(
        "--pos",
        type=int,
        help="Single servo target pos (0..1000). Required with --servo-id.",
    )

    parser.add_argument(
        "--config",
        type=pathlib.Path,
        default=default_cfg,
        help="YAML config file (default: so101_hx35hm_bridge/config/assembly_calibration.yaml).",
    )
    parser.add_argument(
        "--srdf",
        type=pathlib.Path,
        default=_default_srdf_path(),
        help="SRDF file path (default: so101_moveit_config/config/so101_arm.srdf).",
    )
    parser.add_argument(
        "--list-poses",
        action="store_true",
        help="List available manipulator poses in SRDF and exit.",
    )
    parser.add_argument(
        "--only-joints",
        nargs="+",
        default=[],
        help="Only move these joints (names like shoulder_lift).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Move duration seconds (default: from config).",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Send the same command multiple times (default: 1).",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print but do not send.")
    parser.add_argument("--yes", action="store_true", help="Do not prompt for confirmation.")
    parser.add_argument(
        "--device",
        default=None,
        help="Override serial device (default: from config).",
    )
    parser.add_argument(
        "--gripper",
        choices=["skip", "neutral", "open", "closed"],
        default="skip",
        help="Optional gripper handling when using --pose (default: skip).",
    )
    return parser.parse_args()


def _cmd_single_servo(
    *,
    servo_id: int,
    pos: int,
    duration: float,
    repeat: int,
    device: str,
    pos_min: int,
    pos_max: int,
    dry_run: bool,
    yes: bool,
) -> int:
    pos = max(int(pos_min), min(int(pos_max), int(pos)))
    repeat = max(1, int(repeat))
    print(
        f"Single servo command: device={device} servo_id={servo_id} pos={pos} "
        f"(range [{pos_min}, {pos_max}]) duration={duration}s repeat={repeat}"
    )

    if dry_run:
        return 0

    _confirm_or_exit("About to send command to hardware.", assume_yes=yes)

    Board = _import_board()
    board = Board(device=device)
    board.enable_reception()
    payload = [[int(servo_id), int(pos)]]
    for i in range(repeat):
        board.bus_servo_set_position(duration, payload)
        if i != repeat - 1:
            time.sleep(0.05)
    return 0


def _cmd_pose(
    *,
    pose_name: str,
    srdf_path: pathlib.Path,
    servo_spec: ServoSpec,
    joints: Dict[str, JointCalib],
    gripper: GripperSpec,
    only_joints: Iterable[str],
    duration: float,
    repeat: int,
    device: str,
    gripper_mode: str,
    dry_run: bool,
    yes: bool,
) -> int:
    poses = _parse_srdf_group_states(srdf_path)
    if not poses:
        raise RuntimeError(f"No manipulator group_state found in SRDF: {srdf_path}")

    if pose_name not in poses:
        available = ", ".join(sorted(poses.keys()))
        raise ValueError(f"Pose '{pose_name}' not found in SRDF. Available: {available}")

    target_joints = poses[pose_name]
    requested_only = set(only_joints) if only_joints else None

    rows: List[Tuple[str, int, float, int]] = []
    bus_positions: List[List[int]] = []

    # Arm joints
    for joint_name, joint_rad in sorted(target_joints.items()):
        if requested_only is not None and joint_name not in requested_only:
            continue
        if joint_name not in joints:
            raise KeyError(
                f"Joint '{joint_name}' exists in SRDF pose '{pose_name}' but is missing in config 'joints'"
            )
        calib = joints[joint_name]
        pos = _rad_to_servo_pos(joint_rad, calib=calib, servo_spec=servo_spec)
        rows.append((joint_name, calib.servo_id, joint_rad, pos))
        bus_positions.append([calib.servo_id, pos])

    # Optional gripper
    if gripper_mode != "skip":
        gname = "gripper"
        if gname not in joints:
            raise KeyError("config is missing joints.gripper but --gripper was requested")
        calib = joints[gname]
        if gripper_mode == "neutral":
            gpos = int(gripper.neutral_pos)
            grad = float("nan")
        elif gripper_mode == "open":
            grad = float(gripper.open_rad)
            gpos = _rad_to_servo_pos(grad, calib=calib, servo_spec=servo_spec)
        elif gripper_mode == "closed":
            grad = float(gripper.closed_rad)
            gpos = _rad_to_servo_pos(grad, calib=calib, servo_spec=servo_spec)
        else:
            raise ValueError(f"Unknown gripper mode: {gripper_mode}")

        rows.append((gname, calib.servo_id, grad, gpos))
        bus_positions.append([calib.servo_id, gpos])

    if not rows:
        raise RuntimeError("No joints selected to move (check --only-joints).")

    print(f"SRDF: {srdf_path}")
    print(f"Pose: {pose_name}  (group=manipulator)")
    print(f"Device: {device}")
    print(f"Duration: {duration}s  Repeat: {repeat}  Dry-run: {dry_run}")
    print("")
    print(_format_table(rows))
    print("")
    print(f"bus_servo_set_position payload: {bus_positions}")

    if dry_run:
        return 0

    _confirm_or_exit("About to send pose command to hardware.", assume_yes=yes)

    Board = _import_board()
    board = Board(device=device)
    board.enable_reception()
    repeat = max(1, int(repeat))
    for i in range(repeat):
        board.bus_servo_set_position(duration, bus_positions)
        if i != repeat - 1:
            time.sleep(0.05)
    return 0


def main() -> int:
    args = parse_args()

    cfg_path: pathlib.Path = args.config
    if not cfg_path.exists():
        raise SystemExit(f"Config file not found: {cfg_path}")

    servo_spec, joints, gripper = _parse_config(cfg_path)
    device = str(args.device) if args.device is not None else servo_spec.device
    duration = float(args.duration) if args.duration is not None else servo_spec.default_duration_s
    if duration <= 0.0:
        duration = 0.1

    if args.list_poses:
        poses = _parse_srdf_group_states(args.srdf)
        print(f"SRDF: {args.srdf}")
        if not poses:
            print("No manipulator group_state found.")
            return 0
        print("Available manipulator poses:")
        for name in sorted(poses.keys()):
            joints_in_pose = ", ".join(sorted(poses[name].keys()))
            print(f"- {name}: {joints_in_pose}")
        return 0

    if args.servo_id is not None:
        if args.pos is None:
            raise SystemExit("--pos is required with --servo-id")
        return _cmd_single_servo(
            servo_id=int(args.servo_id),
            pos=int(args.pos),
            duration=duration,
            repeat=int(args.repeat),
            device=device,
            pos_min=servo_spec.pos_min,
            pos_max=servo_spec.pos_max,
            dry_run=bool(args.dry_run),
            yes=bool(args.yes),
        )

    pose_name = str(args.pose)
    return _cmd_pose(
        pose_name=pose_name,
        srdf_path=args.srdf,
        servo_spec=servo_spec,
        joints=joints,
        gripper=gripper,
        only_joints=list(args.only_joints),
        duration=duration,
        repeat=int(args.repeat),
        device=device,
        gripper_mode=str(args.gripper),
        dry_run=bool(args.dry_run),
        yes=bool(args.yes),
    )


if __name__ == "__main__":
    raise SystemExit(main())
