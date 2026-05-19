#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import pathlib
import sys
import time
from collections import defaultdict


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
        description="Scan HX35HM bus servo IDs and summarize stable/intermittent responders."
    )
    parser.add_argument("--device", default="/dev/so101_leader")
    parser.add_argument("--ids", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6, 254])
    parser.add_argument("--rounds", type=int, default=6)
    parser.add_argument("--timeout", type=float, default=0.3)
    parser.add_argument("--delay", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    Board = _import_board()
    board = Board(device=args.device)
    board.enable_reception()
    time.sleep(0.2)

    results: dict[int, dict[str, list[object]]] = defaultdict(lambda: {"id": [], "pos": []})

    print(f"device = {args.device}")
    print(f"ids = {list(args.ids)}")
    print(f"rounds = {int(args.rounds)} timeout = {float(args.timeout)}s")
    print("")

    for round_idx in range(int(args.rounds)):
        print(f"round {round_idx + 1}:")
        for sid in args.ids:
            rid = board.bus_servo_read_id(int(sid), timeout=float(args.timeout))
            time.sleep(float(args.delay))
            pos = board.bus_servo_read_position(int(sid), timeout=float(args.timeout))
            time.sleep(float(args.delay))
            results[int(sid)]["id"].append(rid)
            results[int(sid)]["pos"].append(pos)
            print(f"  servo {sid}: id={rid} pos={pos}")
        print("")

    print("summary:")
    for sid in args.ids:
        sid = int(sid)
        id_hits = sum(1 for x in results[sid]["id"] if x is not None)
        pos_hits = sum(1 for x in results[sid]["pos"] if x is not None)

        if pos_hits == int(args.rounds):
            status = "stable"
        elif pos_hits > 0 or id_hits > 0:
            status = "intermittent"
        else:
            status = "missing"

        print(
            f"  servo {sid}: status={status} "
            f"id_hits={id_hits}/{int(args.rounds)} pos_hits={pos_hits}/{int(args.rounds)} "
            f"id_samples={results[sid]['id']} pos_samples={results[sid]['pos']}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
