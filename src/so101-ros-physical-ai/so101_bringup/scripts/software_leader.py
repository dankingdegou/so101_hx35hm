#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import threading
import tkinter as tk
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


REST_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -1.57,
    "elbow_flex": 1.57,
    "wrist_flex": 0.75,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}


@dataclass(frozen=True)
class JointLimit:
    lo: float
    hi: float


JOINT_LIMITS: Dict[str, JointLimit] = {
    "shoulder_pan": JointLimit(-1.91986, 1.91986),
    "shoulder_lift": JointLimit(-1.74533, 1.74533),
    "elbow_flex": JointLimit(-1.69, 1.69),
    "wrist_flex": JointLimit(-1.65806, 1.65806),
    "wrist_roll": JointLimit(-2.74385, 2.84121),
    "gripper": JointLimit(-0.523599, 1.74533),
}


class SoftwareLeaderNode(Node):
    def __init__(self, joint_names: List[str], publish_rate_hz: float) -> None:
        super().__init__("software_leader")
        self.joint_names = joint_names
        self.positions = [float(REST_POSE.get(name, 0.0)) for name in self.joint_names]
        self._lock = threading.Lock()
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.02
        self._timer = self.create_timer(period, self._publish)

    def set_positions(self, values: List[float]) -> None:
        with self._lock:
            self.positions = list(values)

    def _publish(self) -> None:
        with self._lock:
            positions = list(self.positions)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = list(self.joint_names)
        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)
        self._pub.publish(msg)


class SoftwareLeaderGui:
    def __init__(self, node: SoftwareLeaderNode, autosend_ms: int) -> None:
        self.node = node
        self.autosend_ms = max(20, int(autosend_ms))
        self.root = tk.Tk()
        self.root.title("SO101 Software Leader")
        self.root.geometry("760x520")
        self.scales: Dict[str, tk.Scale] = {}
        self.labels: Dict[str, tk.Label] = {}
        self.status_var = tk.StringVar(master=self.root, value="Publishing /leader/joint_states")
        self._build_ui()
        self._schedule_publish()

    def _build_ui(self) -> None:
        top = tk.Frame(self.root)
        top.pack(fill=tk.X, padx=10, pady=8)

        tk.Button(top, text="Rest", command=self.set_rest).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Zero", command=self.set_zero).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="Quit", command=self.root.destroy).pack(side=tk.RIGHT, padx=4)

        body = tk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        for row_idx, name in enumerate(self.node.joint_names):
            lim = JOINT_LIMITS.get(name, JointLimit(-math.pi, math.pi))
            row = tk.Frame(body)
            row.grid(row=row_idx, column=0, sticky="ew", pady=4)
            row.columnconfigure(1, weight=1)

            tk.Label(row, text=name, width=14, anchor="w").grid(row=0, column=0, sticky="w")
            scale = tk.Scale(
                row,
                from_=lim.lo,
                to=lim.hi,
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=500,
                command=lambda _v, joint=name: self._on_slider(joint),
            )
            scale.set(float(REST_POSE.get(name, 0.0)))
            scale.grid(row=0, column=1, sticky="ew")
            self.scales[name] = scale

            label = tk.Label(row, text=f"{scale.get():+.2f} rad", width=12)
            label.grid(row=0, column=2, padx=8)
            self.labels[name] = label

        status = tk.Label(self.root, textvariable=self.status_var, anchor="w")
        status.pack(fill=tk.X, padx=10, pady=8)

    def _on_slider(self, joint: str) -> None:
        value = float(self.scales[joint].get())
        self.labels[joint].config(text=f"{value:+.2f} rad")
        self._publish_now()

    def set_rest(self) -> None:
        for name in self.node.joint_names:
            self.scales[name].set(float(REST_POSE.get(name, 0.0)))
            self.labels[name].config(text=f"{self.scales[name].get():+.2f} rad")
        self._publish_now()
        self.status_var.set("Rest pose")

    def set_zero(self) -> None:
        for name in self.node.joint_names:
            self.scales[name].set(0.0)
            self.labels[name].config(text="+0.00 rad")
        self._publish_now()
        self.status_var.set("Zero pose")

    def _values(self) -> List[float]:
        return [float(self.scales[name].get()) for name in self.node.joint_names]

    def _publish_now(self) -> None:
        self.node.set_positions(self._values())

    def _schedule_publish(self) -> None:
        self._publish_now()
        self.root.after(self.autosend_ms, self._schedule_publish)

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish a GUI-driven /leader/joint_states stream.")
    parser.add_argument("--publish-rate-hz", type=float, default=50.0)
    parser.add_argument("--autosend-ms", type=int, default=100)
    parser.add_argument("--no-gui", action="store_true", help="Publish the rest pose without opening Tk.")
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = SoftwareLeaderNode(JOINT_NAMES, publish_rate_hz=float(args.publish_rate_hz))

    if args.no_gui:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        gui = SoftwareLeaderGui(node, autosend_ms=int(args.autosend_ms))
        gui.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
