#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import threading
import tkinter as tk
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


@dataclass
class SliderConfig:
    minimum: float
    maximum: float
    resolution: float
    unit: str


SLIDERS: Dict[str, SliderConfig] = {
    "x": SliderConfig(-0.050, 0.050, 0.0005, "m"),
    "y": SliderConfig(-0.050, 0.050, 0.0005, "m"),
    "z": SliderConfig(-0.050, 0.050, 0.0005, "m"),
    "roll_deg": SliderConfig(-12.0, 12.0, 0.1, "deg"),
    "pitch_deg": SliderConfig(-12.0, 12.0, 0.1, "deg"),
    "yaw_deg": SliderConfig(-12.0, 12.0, 0.1, "deg"),
}


class DepthRgbTunerNode(Node):
    def __init__(self, parent_frame: str, child_frame: str, publish_hz: float) -> None:
        super().__init__("so101_depth_rgb_tuner")
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self._values = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
        }
        self.latest_status = "waiting for /vision/red_block/status"
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(String, "/vision/red_block/status", self.on_status, 10)
        self.create_timer(max(0.02, 1.0 / max(1.0, publish_hz)), self.publish_transform)

    def set_values(self, values: Dict[str, float]) -> None:
        self._values.update(values)

    def on_status(self, msg: String) -> None:
        self.latest_status = msg.data

    def publish_transform(self) -> None:
        roll = math.radians(self._values["roll_deg"])
        pitch = math.radians(self._values["pitch_deg"])
        yaw = math.radians(self._values["yaw_deg"])
        qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.parent_frame
        tf_msg.child_frame_id = self.child_frame
        tf_msg.transform.translation.x = float(self._values["x"])
        tf_msg.transform.translation.y = float(self._values["y"])
        tf_msg.transform.translation.z = float(self._values["z"])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.broadcaster.sendTransform(tf_msg)

    def cli_snippet(self) -> str:
        roll = math.radians(self._values["roll_deg"])
        pitch = math.radians(self._values["pitch_deg"])
        yaw = math.radians(self._values["yaw_deg"])
        return (
            f"depth_to_rgb_x:={self._values['x']:.6f} "
            f"depth_to_rgb_y:={self._values['y']:.6f} "
            f"depth_to_rgb_z:={self._values['z']:.6f} "
            f"depth_to_rgb_roll:={roll:.6f} "
            f"depth_to_rgb_pitch:={pitch:.6f} "
            f"depth_to_rgb_yaw:={yaw:.6f}"
        )


class DepthRgbTunerApp:
    def __init__(self, node: DepthRgbTunerNode, initial_values: Dict[str, float]) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("SO101 Depth->RGB Tuner")
        self.root.geometry("880x520")
        self.status_var = tk.StringVar(master=self.root, value="starting...")
        self.cli_var = tk.StringVar(master=self.root, value="")
        self.value_labels: Dict[str, tk.Label] = {}
        self.scales: Dict[str, tk.Scale] = {}
        self._build_ui(initial_values)
        self._push_values_to_node()
        self._schedule_refresh()

    def _build_ui(self, initial_values: Dict[str, float]) -> None:
        top = tk.Frame(self.root)
        top.pack(fill=tk.X, padx=10, pady=8)
        tk.Button(top, text="Copy launch args", command=self.copy_cli).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Print current args", command=self.print_cli).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Reset identity", command=self.reset_identity).pack(side=tk.LEFT, padx=4)

        body = tk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        for idx, (name, cfg) in enumerate(SLIDERS.items()):
            row = tk.Frame(body)
            row.grid(row=idx, column=0, sticky="ew", pady=4)
            row.columnconfigure(1, weight=1)

            tk.Label(row, text=name, width=12, anchor="w").grid(row=0, column=0, sticky="w")
            scale = tk.Scale(
                row,
                from_=cfg.minimum,
                to=cfg.maximum,
                resolution=cfg.resolution,
                orient=tk.HORIZONTAL,
                length=520,
                command=lambda _v, key=name: self.on_slider(key),
            )
            scale.set(initial_values.get(name, 0.0))
            scale.grid(row=0, column=1, sticky="ew")
            self.scales[name] = scale

            lbl = tk.Label(row, text=f"{scale.get():+.4f} {cfg.unit}", width=16)
            lbl.grid(row=0, column=2, padx=8)
            self.value_labels[name] = lbl

        tk.Label(self.root, textvariable=self.status_var, anchor="w").pack(fill=tk.X, padx=10, pady=6)
        tk.Label(self.root, textvariable=self.cli_var, anchor="w", justify=tk.LEFT, wraplength=840).pack(
            fill=tk.X, padx=10, pady=6
        )

    def values(self) -> Dict[str, float]:
        return {name: float(scale.get()) for name, scale in self.scales.items()}

    def on_slider(self, key: str) -> None:
        cfg = SLIDERS[key]
        self.value_labels[key].config(text=f"{float(self.scales[key].get()):+.4f} {cfg.unit}")
        self._push_values_to_node()

    def _push_values_to_node(self) -> None:
        values = self.values()
        self.node.set_values(values)
        self.cli_var.set(self.node.cli_snippet())

    def copy_cli(self) -> None:
        text = self.node.cli_snippet()
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.status_var.set(f"{self.node.latest_status} | copied launch args")

    def print_cli(self) -> None:
        print(self.node.cli_snippet(), flush=True)
        self.status_var.set(f"{self.node.latest_status} | printed current args")

    def reset_identity(self) -> None:
        for name in self.scales:
            self.scales[name].set(0.0)
            self.on_slider(name)

    def _schedule_refresh(self) -> None:
        self.status_var.set(self.node.latest_status)
        self.root.after(200, self._schedule_refresh)

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SO101 live tuner for cam_overhead_depth -> cam_overhead")
    parser.add_argument("--parent-frame", default="cam_overhead")
    parser.add_argument("--child-frame", default="cam_overhead_depth")
    parser.add_argument("--publish-hz", type=float, default=20.0)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0, help="initial roll in radians")
    parser.add_argument("--pitch", type=float, default=0.0, help="initial pitch in radians")
    parser.add_argument("--yaw", type=float, default=0.0, help="initial yaw in radians")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    initial_values = {
        "x": float(args.x),
        "y": float(args.y),
        "z": float(args.z),
        "roll_deg": math.degrees(float(args.roll)),
        "pitch_deg": math.degrees(float(args.pitch)),
        "yaw_deg": math.degrees(float(args.yaw)),
    }

    rclpy.init()
    node = DepthRgbTunerNode(args.parent_frame, args.child_frame, args.publish_hz)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        app = DepthRgbTunerApp(node, initial_values)
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
