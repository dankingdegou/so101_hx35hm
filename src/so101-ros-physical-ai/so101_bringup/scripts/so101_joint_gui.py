#!/usr/bin/env python3
from __future__ import annotations

import argparse
import threading
import tkinter as tk
from dataclasses import dataclass
from typing import Dict, List

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


@dataclass
class JointLimit:
    lo: float
    hi: float


DEFAULT_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

DEFAULT_LIMITS: Dict[str, JointLimit] = {
    "shoulder_pan": JointLimit(-1.57, 1.57),
    "shoulder_lift": JointLimit(-1.20, 1.20),
    "elbow_flex": JointLimit(-1.60, 1.60),
    "wrist_flex": JointLimit(-1.60, 1.60),
    "wrist_roll": JointLimit(-3.14, 3.14),
    "gripper": JointLimit(-0.20, 1.60),
}


class JointGuiNode(Node):
    def __init__(
        self,
        command_topic: str,
        joint_state_topic: str,
        preview_scene_topic: str,
        joint_names: List[str],
    ) -> None:
        super().__init__("so101_joint_gui")
        self.joint_names = joint_names
        self.latest_js: Dict[str, float] = {}

        self.pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.sub = self.create_subscription(JointState, joint_state_topic, self.on_joint_state, 10)
        self.preview_scene_pub = self.create_publisher(PlanningScene, preview_scene_topic, 10)

    def on_joint_state(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            self.latest_js[n] = float(p)

    def send(self, values: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(values)
        self.pub.publish(msg)

    def publish_preview(self, values: List[float]) -> None:
        msg = PlanningScene()
        msg.is_diff = True
        msg.robot_state.is_diff = True
        msg.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        msg.robot_state.joint_state.name = list(self.joint_names)
        msg.robot_state.joint_state.position = list(values)
        self.preview_scene_pub.publish(msg)


class JointGuiApp:
    def __init__(self, node: JointGuiNode, joint_limits: Dict[str, JointLimit], autosend_ms: int) -> None:
        self.node = node
        self.joint_limits = joint_limits
        self.autosend_ms = autosend_ms

        self.root = tk.Tk()
        self.root.title("SO101 Joint GUI")
        self.root.geometry("760x520")
        self.autosend_enabled = tk.BooleanVar(master=self.root, value=False)
        self.preview_enabled = tk.BooleanVar(master=self.root, value=True)
        self.status_var = tk.StringVar(master=self.root, value="Ready")

        self.scales: Dict[str, tk.Scale] = {}
        self.value_labels: Dict[str, tk.Label] = {}
        self._build_ui()
        self._schedule_status_refresh()
        self._schedule_autosend()

    def _build_ui(self) -> None:
        top = tk.Frame(self.root)
        top.pack(fill=tk.X, padx=10, pady=8)

        tk.Button(top, text="读取当前姿态", command=self.load_current_pose).pack(side=tk.LEFT, padx=4)
        tk.Button(top, text="发送一次", command=self.send_once).pack(side=tk.LEFT, padx=4)
        tk.Checkbutton(top, text="RViz预览(不执行)", variable=self.preview_enabled).pack(side=tk.LEFT, padx=8)
        tk.Checkbutton(top, text="自动发送", variable=self.autosend_enabled).pack(side=tk.LEFT, padx=8)
        tk.Button(top, text="回零位", command=self.set_home).pack(side=tk.LEFT, padx=4)

        body = tk.Frame(self.root)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        for idx, name in enumerate(self.node.joint_names):
            lim = self.joint_limits[name]
            row = tk.Frame(body)
            row.grid(row=idx, column=0, sticky="ew", pady=4)
            row.columnconfigure(1, weight=1)

            tk.Label(row, text=name, width=14, anchor="w").grid(row=0, column=0, sticky="w")
            scale = tk.Scale(
                row,
                from_=lim.lo,
                to=lim.hi,
                resolution=0.01,
                orient=tk.HORIZONTAL,
                length=480,
                command=lambda _v, n=name: self._on_slider(n),
            )
            scale.set(0.0)
            scale.grid(row=0, column=1, sticky="ew")
            self.scales[name] = scale

            lbl = tk.Label(row, text="0.00 rad", width=10)
            lbl.grid(row=0, column=2, padx=8)
            self.value_labels[name] = lbl

        status = tk.Label(self.root, textvariable=self.status_var, anchor="w")
        status.pack(fill=tk.X, padx=10, pady=8)

    def _on_slider(self, name: str) -> None:
        val = float(self.scales[name].get())
        self.value_labels[name].config(text=f"{val:+.2f} rad")
        if self.preview_enabled.get():
            self.node.publish_preview(self.get_values())

    def set_home(self) -> None:
        for n in self.node.joint_names:
            self.scales[n].set(0.0)
            self._on_slider(n)
        self.status_var.set("已设置到零位（未发送）")

    def load_current_pose(self) -> None:
        loaded = 0
        for n in self.node.joint_names:
            if n in self.node.latest_js:
                val = self.node.latest_js[n]
                lim = self.joint_limits[n]
                val = max(lim.lo, min(lim.hi, val))
                self.scales[n].set(val)
                self._on_slider(n)
                loaded += 1
        if self.preview_enabled.get():
            self.node.publish_preview(self.get_values())
        self.status_var.set(f"已读取当前姿态: {loaded}/{len(self.node.joint_names)}")

    def get_values(self) -> List[float]:
        return [float(self.scales[n].get()) for n in self.node.joint_names]

    def send_once(self) -> None:
        vals = self.get_values()
        self.node.send(vals)
        pretty = ", ".join(f"{n}={v:+.2f}" for n, v in zip(self.node.joint_names, vals))
        self.status_var.set(f"已发送: {pretty}")

    def _schedule_status_refresh(self) -> None:
        if self.node.latest_js:
            text = " | ".join(
                f"{n}:{self.node.latest_js.get(n, 0.0):+.2f}" for n in self.node.joint_names
            )
            self.root.title(f"SO101 Joint GUI   [joint_states] {text}")
        self.root.after(300, self._schedule_status_refresh)

    def _schedule_autosend(self) -> None:
        if self.autosend_enabled.get():
            self.node.send(self.get_values())
        self.root.after(self.autosend_ms, self._schedule_autosend)

    def run(self) -> None:
        self.root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SO101 joint slider GUI")
    parser.add_argument("--command-topic", default="/follower/forward_controller/commands")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--preview-scene-topic", default="/planning_scene")
    parser.add_argument("--autosend-ms", type=int, default=300)
    parser.add_argument("--joint-names", nargs="*", default=DEFAULT_JOINTS)
    return parser.parse_args()


def build_limits(joint_names: List[str]) -> Dict[str, JointLimit]:
    result: Dict[str, JointLimit] = {}
    for n in joint_names:
        result[n] = DEFAULT_LIMITS.get(n, JointLimit(-1.57, 1.57))
    return result


def main() -> None:
    args = parse_args()
    joint_names = list(args.joint_names)
    limits = build_limits(joint_names)

    rclpy.init()
    node = JointGuiNode(
        args.command_topic,
        args.joint_state_topic,
        args.preview_scene_topic,
        joint_names,
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        app = JointGuiApp(node=node, joint_limits=limits, autosend_ms=max(50, int(args.autosend_ms)))
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
