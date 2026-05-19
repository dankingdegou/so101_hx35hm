#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Set HX-35HM bus servo ID from the command line.

Examples:
  python3 set_servo_id.py --new-id 3
  python3 set_servo_id.py --device /dev/so101_leader --old-id 254 --new-id 6 --move-mid

Safety notes:
- Prefer connecting only one servo on the bus when using old-id=254.
- Prints verification readback after writing.
- Prompts for confirmation unless --yes is given.
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time


def _import_board():
    try:
        from ros_robot_controller.ros_robot_controller_sdk import Board  # type: ignore

        return Board
    except ModuleNotFoundError:
        this_file = pathlib.Path(__file__).resolve()
        ws_src = this_file.parents[2]  # .../ros2_ws/src
        candidate = ws_src / "ros_robot_controller-ros2" / "src" / "ros_robot_controller"
        sys.path.insert(0, str(candidate))
        from ros_robot_controller.ros_robot_controller_sdk import Board  # type: ignore

        return Board


def _confirm_or_exit(*, old_id: int, new_id: int, device: str, assume_yes: bool) -> None:
    if assume_yes:
        return
    print(
        f"About to set servo ID on {device}: old_id={old_id} -> new_id={new_id}\n"
        "If old_id=254, make sure only one servo is connected."
    )
    ans = input("Type YES to continue: ").strip()
    if ans != "YES":
        raise SystemExit("Canceled.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Set HX-35HM bus servo ID.")
    parser.add_argument(
        "--device",
        default="/dev/so101_leader",
        help="STM32 serial device (default: /dev/so101_leader).",
    )
    parser.add_argument(
        "--old-id",
        type=int,
        default=254,
        help="Current servo ID, or 254 for broadcast (default: 254).",
    )
    parser.add_argument(
        "--new-id",
        type=int,
        required=True,
        help="New servo ID to write.",
    )
    parser.add_argument(
        "--verify-timeout",
        type=float,
        default=1.0,
        help="Readback timeout in seconds (default: 1.0).",
    )
    parser.add_argument(
        "--move-mid",
        action="store_true",
        help="After successful write, move the new ID to pos=500.",
    )
    parser.add_argument(
        "--mid-pos",
        type=int,
        default=500,
        help="Middle position to send when --move-mid is used (default: 500).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.5,
        help="Move duration for --move-mid (default: 0.5s).",
    )
    parser.add_argument("--yes", action="store_true", help="Skip confirmation prompt.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    old_id = int(args.old_id)
    new_id = int(args.new_id)
    device = str(args.device)
    verify_timeout = float(args.verify_timeout)

    if not 0 <= new_id <= 253:
        raise SystemExit("--new-id must be in [0, 253]")
    if not 0 <= old_id <= 254:
        raise SystemExit("--old-id must be in [0, 254]")

    _confirm_or_exit(old_id=old_id, new_id=new_id, device=device, assume_yes=bool(args.yes))

    Board = _import_board()
    board = Board(device=device)
    board.enable_reception()

    print(f"Setting servo ID on {device}: {old_id} -> {new_id}")
    board.bus_servo_set_id(old_id, new_id)
    time.sleep(0.3)

    id_new = board.bus_servo_read_id(new_id, timeout=verify_timeout)
    pos_new = board.bus_servo_read_position(new_id, timeout=verify_timeout)
    id_254 = board.bus_servo_read_id(254, timeout=verify_timeout)

    print(f"read_id({new_id}) = {id_new}")
    print(f"read_pos({new_id}) = {pos_new}")
    print(f"read_id(254) = {id_254}")

    if args.move_mid:
        mid_pos = max(0, min(1000, int(args.mid_pos)))
        duration = max(0.1, float(args.duration))
        print(f"Moving servo {new_id} to mid position: pos={mid_pos} duration={duration}s")
        board.bus_servo_set_position(duration, [[new_id, mid_pos]])
        time.sleep(max(duration, 0.3))
        pos_after = board.bus_servo_read_position(new_id, timeout=verify_timeout)
        print(f"read_pos({new_id}) after move = {pos_after}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
