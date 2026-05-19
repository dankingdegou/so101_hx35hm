#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Move HX-35HM bus servos to the middle position for assembly.

Default:
- servo ids: 1 2 3 4 5 6
- target pos: 500 (about 120 degrees, middle)
- duration: 1.0 second

This script is designed to be runnable directly from the source tree.
Recommended usage is still: `colcon build` + `source install/setup.bash`.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time
from typing import List


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Return HX-35HM servos to middle position.")
    parser.add_argument(
        "--device",
        default="/dev/ros_robot_controller",
        help="Serial device for the STM32 controller (default: /dev/ros_robot_controller).",
    )
    parser.add_argument(
        "--servo-ids",
        nargs="+",
        type=int,
        default=[1, 2, 3, 4, 5, 6],
        help="Servo IDs to move (default: 1 2 3 4 5 6).",
    )
    parser.add_argument(
        "--pos",
        type=int,
        default=500,
        help="Target position in [0..1000] (default: 500, middle).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1.0,
        help="Move duration in seconds (default: 1.0).",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print command but do not send to hardware.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Send command multiple times with a short delay (default: 1).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    servo_ids: List[int] = list(args.servo_ids)
    if not servo_ids:
        print("No servo ids provided.")
        return 2

    pos = int(args.pos)
    pos = max(0, min(1000, pos))

    duration = float(args.duration)
    if duration <= 0.0:
        duration = 0.1

    repeat = int(args.repeat)
    if repeat <= 0:
        repeat = 1

    positions = [[sid, pos] for sid in servo_ids]
    print(f"device={args.device}")
    print(f"servo_ids={servo_ids}")
    print(f"target_pos={pos} (0->0deg, 500->120deg, 1000->240deg)")
    print(f"duration={duration}s repeat={repeat} dry_run={args.dry_run}")
    print(f"positions={positions}")

    if args.dry_run:
        return 0

    Board = _import_board()
    try:
        board = Board(device=args.device)
        board.enable_reception()
        for i in range(repeat):
            board.bus_servo_set_position(duration, positions)
            if i != repeat - 1:
                time.sleep(0.05)
    except Exception as exc:  # noqa: BLE001
        print(f"Failed to move servos: {exc}")
        return 1

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

