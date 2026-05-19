#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Enter or exit a low-risk leader drag mode for HX-35HM arms.

Typical usage:
  python3 leader_drag_mode.py --mode enter
  python3 leader_drag_mode.py --mode enter --method custom-unload
  python3 leader_drag_mode.py --mode verify-readback --rounds 10
  python3 leader_drag_mode.py --mode exit

This helper is intentionally conservative:
- defaults to the leader device
- targets servo IDs 1..6
- can verify whether position readback still works after torque is released
- supports custom STM32 firmware commands for native HX/Hiwonder load/unload
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
        ws_src = this_file.parents[2]
        candidate = ws_src / "ros_robot_controller-ros2" / "src" / "ros_robot_controller"
        sys.path.insert(0, str(candidate))
        from ros_robot_controller.ros_robot_controller_sdk import Board  # type: ignore

        return Board


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Toggle HX-35HM leader drag mode by disabling or enabling servo torque."
    )
    parser.add_argument(
        "--device",
        default="/dev/so101_leader",
        help="STM32 serial device (default: /dev/so101_leader).",
    )
    parser.add_argument(
        "--mode",
        choices=["enter", "exit", "status", "verify-readback"],
        required=True,
        help="enter=disable torque, exit=enable torque, status=read only, verify-readback=scan positions only.",
    )
    parser.add_argument(
        "--method",
        choices=["sdk-torque", "custom-unload", "custom-raw"],
        default="sdk-torque",
        help=(
            "Unload/load method for enter/exit. "
            "sdk-torque uses stock STM32 0x0C/0x0B. "
            "custom-unload uses custom STM32 0xF2/0xF3. "
            "custom-raw uses custom raw passthrough 0xF0."
        ),
    )
    parser.add_argument(
        "--ids",
        nargs="+",
        type=int,
        default=[1, 2, 3, 4, 5, 6],
        help="Servo IDs to operate on (default: 1 2 3 4 5 6).",
    )
    parser.add_argument(
        "--rounds",
        type=int,
        default=6,
        help="Verification rounds for status/verify-readback (default: 6).",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Read timeout seconds (default: 0.5).",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.05,
        help="Small delay between bus operations (default: 0.05).",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip confirmation prompt for enter/exit.",
    )
    return parser.parse_args()


def _confirm_or_exit(
    mode: str, device: str, ids: list[int], method: str, assume_yes: bool
) -> None:
    if assume_yes or mode in ("status", "verify-readback"):
        return
    action = "disable torque" if mode == "enter" else "enable torque"
    print(f"About to {action} on {device} for servo IDs {ids} using method={method}.")
    print("Type YES to continue:")
    ans = input().strip()
    if ans != "YES":
        raise SystemExit("Canceled.")


def _print_status(board, ids: list[int], timeout: float, delay: float) -> None:
    for sid in ids:
        torque = board.bus_servo_read_torque_state(int(sid), timeout=timeout)
        time.sleep(delay)
        pos = board.bus_servo_read_position(int(sid), timeout=timeout)
        time.sleep(delay)
        print(f"servo {sid}: torque_state={torque} pos={pos}")


def _verify_readback(board, ids: list[int], rounds: int, timeout: float, delay: float) -> int:
    hits = {sid: 0 for sid in ids}
    print(f"verify-readback: rounds={rounds} timeout={timeout}s")
    for round_idx in range(rounds):
        print(f"round {round_idx + 1}:")
        for sid in ids:
            pos = board.bus_servo_read_position(int(sid), timeout=timeout)
            time.sleep(delay)
            if pos is not None:
                hits[sid] += 1
            print(f"  servo {sid}: pos={pos}")
        print("")

    print("summary:")
    for sid in ids:
        print(f"  servo {sid}: readback_hits={hits[sid]}/{rounds}")
    return 0 if all(hits[sid] > 0 for sid in ids) else 1


def main() -> int:
    args = parse_args()
    ids = [int(sid) for sid in args.ids]
    _confirm_or_exit(args.mode, str(args.device), ids, str(args.method), bool(args.yes))

    Board = _import_board()
    board = Board(device=str(args.device))
    board.enable_reception()
    time.sleep(0.2)

    if args.mode == "status":
        _print_status(board, ids, float(args.timeout), float(args.delay))
        return 0

    if args.mode == "verify-readback":
        return _verify_readback(
            board,
            ids,
            rounds=max(1, int(args.rounds)),
            timeout=float(args.timeout),
            delay=float(args.delay),
        )

    enable = 0 if args.mode == "enter" else 1
    action = "Disabling/unloading" if enable == 0 else "Enabling/loading"
    print(f"{action} on {args.device} for servo IDs {ids} using method={args.method}")

    if args.method == "sdk-torque":
        for sid in ids:
            board.bus_servo_enable_torque(int(sid), enable)
            time.sleep(float(args.delay))
    elif args.method == "custom-unload":
        if enable == 0:
            board.bus_servo_unload_many(ids)
        else:
            board.bus_servo_load_many(ids)
    elif args.method == "custom-raw":
        for sid in ids:
            board.bus_servo_load_or_unload_raw(int(sid), enable)
            time.sleep(float(args.delay))
    else:
        raise RuntimeError(f"unsupported method: {args.method}")

    time.sleep(0.2)
    _print_status(board, ids, float(args.timeout), float(args.delay))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
