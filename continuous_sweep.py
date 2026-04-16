#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

from ros_robot_controller.ros_robot_controller_sdk import Board


# 根据你实际接的舵机 ID 修改这里（当前假定 1~6 都接了）
SERVO_IDS = [1, 2, 3, 4, 5, 6]


def main() -> None:
    board = Board(device="/dev/ros_robot_controller")
    board.enable_reception()
    print("START continuous sweep... (Ctrl+C 退出)")

    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            # 在 0~1000 之间正弦往复，中心约 500（≈120°），幅度 300（≈72°）
            center = 500
            amp = 300
            pos = int(center + amp * math.sin(0.5 * t))  # 0.5 rad/s，可适当调整转速

            positions = [[sid, pos] for sid in SERVO_IDS]
            board.bus_servo_set_position(0.1, positions)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("STOP")
        # 回到中位
        positions = [[sid, 500] for sid in SERVO_IDS]
        board.bus_servo_set_position(0.5, positions)


if __name__ == "__main__":
    main()

