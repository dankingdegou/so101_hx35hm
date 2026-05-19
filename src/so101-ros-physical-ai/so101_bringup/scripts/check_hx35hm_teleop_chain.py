#!/usr/bin/env python3

from __future__ import annotations

import argparse
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class TeleopChainCheck(Node):
    def __init__(self, leader_ns: str, follower_ns: str) -> None:
        super().__init__("check_hx35hm_teleop_chain")
        self._counts: Dict[str, int] = {
            "leader_joint_states": 0,
            "follower_joint_states": 0,
            "follower_forward_commands": 0,
        }
        self._last: Dict[str, object] = {
            "leader_joint_states": None,
            "follower_joint_states": None,
            "follower_forward_commands": None,
        }

        self._leader_topic = f"/{leader_ns}/joint_states"
        self._follower_state_topic = f"/{follower_ns}/joint_states"
        self._follower_cmd_topic = f"/{follower_ns}/forward_controller/commands"

        self.create_subscription(
            JointState,
            self._leader_topic,
            self._on_leader_joint_states,
            20,
        )
        self.create_subscription(
            JointState,
            self._follower_state_topic,
            self._on_follower_joint_states,
            20,
        )
        self.create_subscription(
            Float64MultiArray,
            self._follower_cmd_topic,
            self._on_follower_forward_commands,
            20,
        )

    def _on_leader_joint_states(self, msg: JointState) -> None:
        self._counts["leader_joint_states"] += 1
        self._last["leader_joint_states"] = {
            "names": list(msg.name),
            "positions": list(msg.position),
        }

    def _on_follower_joint_states(self, msg: JointState) -> None:
        self._counts["follower_joint_states"] += 1
        self._last["follower_joint_states"] = {
            "names": list(msg.name),
            "positions": list(msg.position),
        }

    def _on_follower_forward_commands(self, msg: Float64MultiArray) -> None:
        self._counts["follower_forward_commands"] += 1
        self._last["follower_forward_commands"] = list(msg.data)

    def snapshot(self) -> Dict[str, object]:
        active_nodes = sorted(self.get_node_names())
        topics = sorted(name for name, _types in self.get_topic_names_and_types())
        return {
            "leader_topic": self._leader_topic,
            "follower_state_topic": self._follower_state_topic,
            "follower_cmd_topic": self._follower_cmd_topic,
            "counts": dict(self._counts),
            "last": dict(self._last),
            "nodes": active_nodes,
            "topics": topics,
        }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check leader->teleop->follower ROS chain for HX35HM teleoperation."
    )
    parser.add_argument("--leader-namespace", default="leader")
    parser.add_argument("--follower-namespace", default="follower")
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = TeleopChainCheck(args.leader_namespace, args.follower_namespace)

    deadline = time.time() + max(0.1, float(args.duration))
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    snap = node.snapshot()

    print("=== HX35HM Teleop Chain Check ===")
    print(f"leader_topic: {snap['leader_topic']}")
    print(f"follower_state_topic: {snap['follower_state_topic']}")
    print(f"follower_cmd_topic: {snap['follower_cmd_topic']}")
    print("")
    print("counts:")
    for key, value in snap["counts"].items():
        print(f"  {key}: {value}")
    print("")
    print("last samples:")
    for key, value in snap["last"].items():
        print(f"  {key}: {value}")
    print("")
    print("key nodes present:")
    for name in snap["nodes"]:
        if "hx35hm_bridge" in name or "teleop" in name or "relay" in name:
            print(f"  {name}")
    print("")
    print("key topics present:")
    for topic in snap["topics"]:
        if (
            topic.endswith("/joint_states")
            or topic.endswith("/forward_controller/commands")
            or topic.endswith("/trajectory_controller/joint_trajectory")
        ):
            print(f"  {topic}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
