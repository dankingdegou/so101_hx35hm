#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Raw Hiwonder/HX bus-servo leader helper.

This script talks to a Hiwonder-compatible bus-servo adapter directly, not to
the ros_robot_controller STM32 wrapper. It is intended for BusLinker/USB-TTL
half-duplex adapters that pass raw servo protocol frames to the servo bus.

Supported protocol in this first version:
- Legacy Hiwonder/LewanSoul frame: 55 55 ID LENGTH CMD PARAM... CHECKSUM
- SERVO_LOAD_OR_UNLOAD_WRITE = 31, param 0 unloads torque
- SERVO_LOAD_OR_UNLOAD_READ = 32
- SERVO_POS_READ = 28

Use this only with the leader arm, and never while hx35hm_bridge is using the
same serial port.
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from typing import Iterable, Optional

import serial  # type: ignore


CMD_SERVO_POS_READ = 28
CMD_SERVO_LOAD_OR_UNLOAD_WRITE = 31
CMD_SERVO_LOAD_OR_UNLOAD_READ = 32


def checksum(payload: Iterable[int]) -> int:
    return (~sum(int(x) & 0xFF for x in payload)) & 0xFF


def build_packet(servo_id: int, cmd: int, params: Iterable[int] = ()) -> bytes:
    body = [int(servo_id) & 0xFF]
    params_list = [int(x) & 0xFF for x in params]
    # LENGTH is len(cmd + params + checksum).
    body.append(len(params_list) + 3)
    body.append(int(cmd) & 0xFF)
    body.extend(params_list)
    body.append(checksum(body))
    return bytes([0x55, 0x55, *body])


@dataclass(frozen=True)
class ServoReply:
    servo_id: int
    cmd: int
    params: bytes


class RawHiwonderBus:
    def __init__(self, device: str, baudrate: int, timeout: float) -> None:
        self.serial = serial.Serial(device, baudrate=baudrate, timeout=timeout)

    def close(self) -> None:
        self.serial.close()

    def write_cmd(self, servo_id: int, cmd: int, params: Iterable[int] = ()) -> None:
        pkt = build_packet(servo_id, cmd, params)
        self.serial.reset_input_buffer()
        self.serial.write(pkt)
        self.serial.flush()

    def read_reply(self, expected_id: int, expected_cmd: int) -> Optional[ServoReply]:
        deadline = time.monotonic() + float(self.serial.timeout or 0.2)
        buf = bytearray()
        while time.monotonic() < deadline:
            chunk = self.serial.read(1)
            if not chunk:
                continue
            buf.extend(chunk)
            while len(buf) >= 2 and not (buf[0] == 0x55 and buf[1] == 0x55):
                del buf[0]
            if len(buf) < 5:
                continue
            length = buf[3]
            frame_len = length + 3
            if len(buf) < frame_len:
                continue
            frame = bytes(buf[:frame_len])
            del buf[:frame_len]
            body = list(frame[2:-1])
            if checksum(body) != frame[-1]:
                continue
            servo_id = frame[2]
            cmd = frame[4]
            if expected_id != 254 and servo_id != expected_id:
                continue
            if cmd != expected_cmd:
                continue
            return ServoReply(servo_id=servo_id, cmd=cmd, params=frame[5:-1])
        return None

    def request(self, servo_id: int, cmd: int, params: Iterable[int] = ()) -> Optional[ServoReply]:
        self.write_cmd(servo_id, cmd, params)
        return self.read_reply(servo_id, cmd)

    def unload(self, servo_id: int) -> None:
        self.write_cmd(servo_id, CMD_SERVO_LOAD_OR_UNLOAD_WRITE, [0])

    def load(self, servo_id: int) -> None:
        self.write_cmd(servo_id, CMD_SERVO_LOAD_OR_UNLOAD_WRITE, [1])

    def read_load_state(self, servo_id: int) -> Optional[int]:
        reply = self.request(servo_id, CMD_SERVO_LOAD_OR_UNLOAD_READ)
        if reply is None or len(reply.params) < 1:
            return None
        return int(reply.params[0])

    def read_position(self, servo_id: int) -> Optional[int]:
        reply = self.request(servo_id, CMD_SERVO_POS_READ)
        if reply is None or len(reply.params) < 2:
            return None
        return struct.unpack("<h", reply.params[:2])[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Raw Hiwonder bus-servo unload/readback helper for a direct BusLinker/USB-TTL adapter."
    )
    parser.add_argument("--device", required=True, help="Raw bus adapter serial device, e.g. /dev/ttyUSB0.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Raw servo baudrate, default 115200.")
    parser.add_argument("--timeout", type=float, default=0.2, help="Read timeout seconds.")
    parser.add_argument("--ids", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6])
    parser.add_argument(
        "--mode",
        choices=["unload", "load", "status", "watch"],
        required=True,
        help="unload/load torque, read status once, or continuously watch positions.",
    )
    parser.add_argument("--rounds", type=int, default=20, help="Watch rounds.")
    parser.add_argument("--delay", type=float, default=0.05, help="Delay between operations.")
    parser.add_argument("--yes", action="store_true", help="Skip confirmation for load/unload.")
    return parser.parse_args()


def confirm(mode: str, device: str, ids: list[int], yes: bool) -> None:
    if yes or mode in ("status", "watch"):
        return
    print(f"About to {mode} raw bus servos {ids} on {device}.")
    print("Type YES to continue:")
    if input().strip() != "YES":
        raise SystemExit("Canceled.")


def print_status(bus: RawHiwonderBus, ids: list[int], delay: float) -> None:
    for sid in ids:
        load = bus.read_load_state(sid)
        time.sleep(delay)
        pos = bus.read_position(sid)
        time.sleep(delay)
        print(f"servo {sid}: load_state={load} pos={pos}")


def main() -> int:
    args = parse_args()
    ids = [int(x) for x in args.ids]
    confirm(str(args.mode), str(args.device), ids, bool(args.yes))

    bus = RawHiwonderBus(str(args.device), int(args.baudrate), float(args.timeout))
    try:
        if args.mode == "unload":
            for sid in ids:
                bus.unload(sid)
                time.sleep(float(args.delay))
            time.sleep(0.2)
            print_status(bus, ids, float(args.delay))
            return 0

        if args.mode == "load":
            for sid in ids:
                bus.load(sid)
                time.sleep(float(args.delay))
            time.sleep(0.2)
            print_status(bus, ids, float(args.delay))
            return 0

        if args.mode == "status":
            print_status(bus, ids, float(args.delay))
            return 0

        for idx in range(max(1, int(args.rounds))):
            print(f"round {idx + 1}:")
            print_status(bus, ids, float(args.delay))
            print("")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
