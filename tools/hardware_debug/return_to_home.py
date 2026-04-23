#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ros_robot_controller.ros_robot_controller_sdk import Board


# 根据你实际使用的舵机 ID 修改这里
SERVO_IDS = [1, 2, 3, 4, 5, 6]

# HX‑35HM:
# - 0     → 0°
# - 500   → 120°（中位）
# - 1000  → 240°
TARGET_POS = 500  # 默认回到中位，如需回到 0° 可改成 0


def main() -> None:
    board = Board(device="/dev/ros_robot_controller")
    board.enable_reception()
    print(f"Move servos {SERVO_IDS} to pos={TARGET_POS} ...")

    positions = [[sid, TARGET_POS] for sid in SERVO_IDS]
    # 1.0 秒内缓慢移动到目标位置，可按需要调节时间
    board.bus_servo_set_position(1.0, positions)

    print("Done.")


if __name__ == "__main__":
    main()

