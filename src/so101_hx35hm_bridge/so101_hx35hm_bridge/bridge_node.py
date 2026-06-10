#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import pathlib
import sys
import time
from typing import Dict, List

import fcntl
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Time
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import ParallelGripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    from ros_robot_controller.ros_robot_controller_sdk import (  # pyright: ignore[reportMissingImports]
        Board,
    )
except ModuleNotFoundError:
    # Allow running this file directly from the source tree without a sourced overlay.
    # Recommended usage is still `colcon build` + `source install/setup.bash` + `ros2 run ...`.
    this_file = pathlib.Path(__file__).resolve()
    ws_src = this_file.parents[2]  # .../ros2_ws/src
    candidate = ws_src / "ros_robot_controller-ros2" / "src" / "ros_robot_controller"
    sys.path.insert(0, str(candidate))
    from ros_robot_controller.ros_robot_controller_sdk import Board  # type: ignore


JOINT_ID_MAP: Dict[str, int] = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}

JOINT_LIMITS_RAD: Dict[str, tuple[float, float]] = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.523599, 1.74533),
}


class Hx35hmBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("hx35hm_bridge")

        # Single-instance guard for the serial device. Multiple hx35hm_bridge
        # processes writing to the same /dev/tty* will cause readback timeouts
        # and severe servo jitter due to interleaved commands.
        self._device_lock_fp = None

        self.declare_parameter("device", "/dev/ros_robot_controller")
        self.declare_parameter("joint_names", list(JOINT_ID_MAP.keys()))
        self.declare_parameter("command_topic", "/follower/forward_controller/commands")
        self.declare_parameter("enable_command_subscription", True)
        self.declare_parameter("learning_action_topic", "")
        self.declare_parameter("learning_action_keepalive_s", 0.0)
        self.declare_parameter("move_duration", 0.2)
        self.declare_parameter("stream_command_duration", 0.04)
        self.declare_parameter("stream_command_async_write", False)
        self.declare_parameter("stream_write_rate_hz", 75.0)
        self.declare_parameter("stream_target_smoothing", False)
        self.declare_parameter("stream_continuous_follow", True)
        self.declare_parameter("stream_max_velocity_rad_s", 3.0)
        self.declare_parameter("command_position_deadband_rad", 0.002)
        self.declare_parameter("gripper_command_deadband_rad", 0.01)
        self.declare_parameter("suspend_readback_after_stream_command_s", 0.0)
        self.declare_parameter("log_gripper_mapping", False)
        self.declare_parameter("disable_torque_on_startup", False)
        self.declare_parameter("restore_torque_on_shutdown", False)
        self.declare_parameter("torque_servo_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("torque_command_retries", 3)
        self.declare_parameter("torque_command_interval_s", 0.05)
        self.declare_parameter("maintain_torque_disabled", False)
        self.declare_parameter("torque_disable_keepalive_rate_hz", 2.0)
        self.declare_parameter("publish_joint_states_topic", "/joint_states")
        self.declare_parameter("state_publish_rate_hz", 50.0)
        self.declare_parameter("trajectory_command_rate_hz", 50.0)
        self.declare_parameter("trajectory_min_command_interval_s", 0.015)
        self.declare_parameter("trajectory_min_segment_duration_s", 0.02)
        self.declare_parameter("trajectory_min_total_duration_s", 0.60)
        self.declare_parameter("trajectory_final_settle_s", 0.05)
        self.declare_parameter("suspend_readback_during_trajectory", True)
        # 是否开启 FollowJointTrajectory 动作服务（用于 MoveIt）
        self.declare_parameter("enable_follow_joint_trajectory", True)
        # 是否开启 ParallelGripperCommand 动作服务（用于 MoveIt gripper execution）
        self.declare_parameter("enable_gripper_action", True)
        # 是否周期性回读舵机真实位置，并用其发布 joint_states（推荐开启以提升 MoveIt 状态准确性）
        self.declare_parameter("enable_position_readback", True)
        # 回读周期（Hz）。注意：当前 SDK 为逐舵机请求-应答；频率过高会加重总线负载。
        # 默认使用 round_robin 模式时建议 50~100Hz（6 个关节约 8~16Hz/关节）。
        self.declare_parameter("position_readback_rate_hz", 60.0)
        # 回读模式：
        # - "round_robin": 每次只读 1 个舵机（默认，避免单次回调阻塞太久）
        # - "all": 每次读完所有关节（更实时，但在异常时可能阻塞更久）
        self.declare_parameter("position_readback_mode", "round_robin")
        # Leader teleop may need raw encoder-derived angles beyond follower/MoveIt limits.
        # Keep enabled for follower state publishing; disable for read-only leader arms.
        self.declare_parameter("clamp_readback_to_joint_limits", True)
        # 单次回读等待超时（秒）。用于避免串口异常时阻塞回调线程。
        self.declare_parameter("position_readback_timeout_s", 0.05)
        # HX-35HM 位置映射参数（默认: 0 rad -> pos=500, 240deg span -> 0..1000）
        self.declare_parameter("servo_pos_min", 0)
        self.declare_parameter("servo_pos_max", 1000)
        self.declare_parameter("servo_range_deg", 240.0)
        # 0 rad 对应的舵机位置（默认 500 ≈ 120deg 中位）
        self.declare_parameter("servo_zero_pos", 500.0)
        # 可选：每个关节的方向（1 或 -1），与 joint_names 顺序对齐
        # shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
        # 注意：elbow_flex方向已反转以匹配物理运动
        self.declare_parameter("joint_directions", [-1, -1, -1, -1, -1, -1])
        # 可选：每个关节 0 rad 对应的舵机位置（覆盖 servo_zero_pos），与 joint_names 顺序对齐
        self.declare_parameter("joint_zero_positions", [500.0, 500.0, 500.0, 500.0, 500.0, 500.0])

        device = self.get_parameter("device").get_parameter_value().string_value
        cmd_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        enable_command_subscription = bool(
            self.get_parameter("enable_command_subscription").get_parameter_value().bool_value
        )
        learning_action_topic = (
            self.get_parameter("learning_action_topic").get_parameter_value().string_value
        )
        self.learning_action_keepalive_s = float(
            self.get_parameter("learning_action_keepalive_s").get_parameter_value().double_value
        )
        joint_names_param = (
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        if joint_names_param:
            self.joint_names: List[str] = list(joint_names_param)
        else:
            self.joint_names = list(JOINT_ID_MAP.keys())

        self.move_duration = (
            self.get_parameter("move_duration").get_parameter_value().double_value
        )
        self.stream_command_duration = float(
            self.get_parameter("stream_command_duration").get_parameter_value().double_value
        )
        self.stream_command_async_write = bool(
            self.get_parameter("stream_command_async_write").get_parameter_value().bool_value
        )
        self.stream_write_rate_hz = float(
            self.get_parameter("stream_write_rate_hz").get_parameter_value().double_value
        )
        self.stream_target_smoothing = bool(
            self.get_parameter("stream_target_smoothing").get_parameter_value().bool_value
        )
        self.stream_continuous_follow = bool(
            self.get_parameter("stream_continuous_follow").get_parameter_value().bool_value
        )
        self.stream_max_velocity_rad_s = float(
            self.get_parameter("stream_max_velocity_rad_s").get_parameter_value().double_value
        )
        self.command_position_deadband_rad = float(
            self.get_parameter("command_position_deadband_rad").get_parameter_value().double_value
        )
        self.gripper_command_deadband_rad = float(
            self.get_parameter("gripper_command_deadband_rad").get_parameter_value().double_value
        )
        self.suspend_readback_after_stream_command_s = float(
            self.get_parameter("suspend_readback_after_stream_command_s")
            .get_parameter_value()
            .double_value
        )
        self.log_gripper_mapping = bool(
            self.get_parameter("log_gripper_mapping").get_parameter_value().bool_value
        )
        self.disable_torque_on_startup = bool(
            self.get_parameter("disable_torque_on_startup").get_parameter_value().bool_value
        )
        self.restore_torque_on_shutdown = bool(
            self.get_parameter("restore_torque_on_shutdown").get_parameter_value().bool_value
        )
        torque_servo_ids_param = (
            self.get_parameter("torque_servo_ids").get_parameter_value().integer_array_value
        )
        self.torque_command_retries = max(
            1, int(self.get_parameter("torque_command_retries").get_parameter_value().integer_value)
        )
        self.torque_command_interval_s = float(
            self.get_parameter("torque_command_interval_s").get_parameter_value().double_value
        )
        self.maintain_torque_disabled = bool(
            self.get_parameter("maintain_torque_disabled").get_parameter_value().bool_value
        )
        self.torque_disable_keepalive_rate_hz = float(
            self.get_parameter("torque_disable_keepalive_rate_hz")
            .get_parameter_value()
            .double_value
        )
        state_topic = (
            self.get_parameter("publish_joint_states_topic")
            .get_parameter_value()
            .string_value
        )
        state_rate = (
            self.get_parameter("state_publish_rate_hz").get_parameter_value().double_value
        )
        self.trajectory_command_rate_hz = float(
            self.get_parameter("trajectory_command_rate_hz").get_parameter_value().double_value
        )
        self.trajectory_min_command_interval_s = float(
            self.get_parameter("trajectory_min_command_interval_s").get_parameter_value().double_value
        )
        self.trajectory_min_segment_duration_s = float(
            self.get_parameter("trajectory_min_segment_duration_s").get_parameter_value().double_value
        )
        self.trajectory_min_total_duration_s = float(
            self.get_parameter("trajectory_min_total_duration_s").get_parameter_value().double_value
        )
        self.trajectory_final_settle_s = float(
            self.get_parameter("trajectory_final_settle_s").get_parameter_value().double_value
        )
        self.suspend_readback_during_trajectory = bool(
            self.get_parameter("suspend_readback_during_trajectory")
            .get_parameter_value()
            .bool_value
        )
        enable_fjt = (
            self.get_parameter("enable_follow_joint_trajectory")
            .get_parameter_value()
            .bool_value
        )
        enable_gripper_action = (
            self.get_parameter("enable_gripper_action").get_parameter_value().bool_value
        )
        enable_readback = (
            self.get_parameter("enable_position_readback").get_parameter_value().bool_value
        )
        self.clamp_readback_to_joint_limits = bool(
            self.get_parameter("clamp_readback_to_joint_limits").get_parameter_value().bool_value
        )
        readback_rate = (
            self.get_parameter("position_readback_rate_hz").get_parameter_value().double_value
        )
        self.readback_mode = str(self.get_parameter("position_readback_mode").value)
        self.readback_timeout_s = float(
            self.get_parameter("position_readback_timeout_s").get_parameter_value().double_value
        )

        self.servo_pos_min = int(
            self.get_parameter("servo_pos_min").get_parameter_value().integer_value
        )
        self.servo_pos_max = int(
            self.get_parameter("servo_pos_max").get_parameter_value().integer_value
        )
        self.servo_range_deg = float(
            self.get_parameter("servo_range_deg").get_parameter_value().double_value
        )
        self.servo_zero_pos = float(
            self.get_parameter("servo_zero_pos").get_parameter_value().double_value
        )

        directions_param = (
            self.get_parameter("joint_directions").get_parameter_value().integer_array_value
        )
        zero_positions_param = (
            self.get_parameter("joint_zero_positions")
            .get_parameter_value()
            .double_array_value
        )

        if directions_param and len(directions_param) != len(self.joint_names):
            self.get_logger().warn(
                "joint_directions length does not match joint_names; falling back to all +1"
            )
            directions_param = []
        if zero_positions_param and len(zero_positions_param) != len(self.joint_names):
            self.get_logger().warn(
                "joint_zero_positions length does not match joint_names; falling back to servo_zero_pos"
            )
            zero_positions_param = []

        self.joint_directions: Dict[str, int] = {
            name: int(directions_param[i]) if directions_param else 1
            for i, name in enumerate(self.joint_names)
        }
        self.joint_zero_positions: Dict[str, float] = {
            name: float(zero_positions_param[i]) if zero_positions_param else self.servo_zero_pos
            for i, name in enumerate(self.joint_names)
        }
        self.torque_servo_ids: List[int] = (
            [int(i) for i in torque_servo_ids_param]
            if torque_servo_ids_param
            else [JOINT_ID_MAP[name] for name in self.joint_names if name in JOINT_ID_MAP]
        )

        # Acquire a non-blocking exclusive lock derived from the device real path.
        # If another bridge already holds it, abort early to protect the bus.
        try:
            real_dev = os.path.realpath(device)
            lock_name = real_dev.replace("/", "_").replace(":", "_")
            lock_path = f"/tmp/so101_hx35hm_bridge{lock_name}.lock"
            self._device_lock_fp = open(lock_path, "w")  # noqa: PTH123
            fcntl.flock(self._device_lock_fp.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.get_logger().error(
                f"Serial device already in use by another hx35hm_bridge: {device}. "
                "Stop other hx35hm_bridge processes first (otherwise readback will timeout and servos will jitter)."
            )
            raise SystemExit(2)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Could not acquire device lock for {device}: {exc}")

        cmd_desc = cmd_topic if enable_command_subscription else "<disabled/read-only>"
        self.get_logger().info(f"Connecting Board on {device}, command input: {cmd_desc}")
        self.board = Board(device=device)
        self.board.enable_reception()

        # Warn once per joint if we have to clip commanded positions into servo_pos_min/max.
        self._clip_warned = set()
        self._joint_limit_warned = set()

        # 当前关节的"已知姿态"（用于发布 joint_states），初始设为 0 rad，
        # 后续在 send_positions 中更新。
        self.current_positions: List[float] = [0.0 for _ in self.joint_names]
        self._last_sent_positions_rad: Dict[str, float] = {}
        self._pending_stream_joint_names: List[str] | None = None
        self._pending_stream_positions: List[float] | None = None
        self._pending_stream_dirty = False
        self._stream_output_positions: Dict[str, float] = {}
        self._last_stream_flush_time = time.monotonic()
        self._write_fail_count = 0
        self._shutting_down = False
        self._torque_disabled_by_startup = False
        self._torque_keepalive_timer = None
        self._last_learning_action: List[float] | None = None
        
        # Store targets for readback (must be defined before _do_initial_readback)
        self._readback_targets = [
            (i, name, JOINT_ID_MAP[name])
            for i, name in enumerate(self.joint_names)
            if name in JOINT_ID_MAP
        ]
        
        # 启动时立即读取所有舵机位置，避免MoveIt使用错误的状态
        self._initial_readback_done = False
        self._do_initial_readback()

        if self.disable_torque_on_startup:
            self._set_torque_enabled(False, context="startup")
            self._torque_disabled_by_startup = True
            self._start_torque_disable_keepalive()
        
        # 初始化时间戳
        self.last_update_time = self.get_clock().now()

        # 简单命令接口：直接订阅 forward_controller 的 Float64MultiArray 命令。
        # Leader/read-only mode deliberately disables this path so no ROS command
        # can accidentally load torque or send a target to the hand-drag arm.
        self.cmd_sub = None
        if enable_command_subscription:
            command_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.cmd_sub = self.create_subscription(
                Float64MultiArray, cmd_topic, self.command_callback, command_qos
            )
        else:
            self.get_logger().info("Command subscription disabled; bridge is read-only")

        self.learning_action_pub = None
        if learning_action_topic:
            learning_action_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.learning_action_pub = self.create_publisher(
                Float64MultiArray, learning_action_topic, learning_action_qos
            )
            self.get_logger().info(
                f"Learning action mirror enabled at '{learning_action_topic}'"
            )

        self.learning_action_timer = None
        if self.learning_action_pub is not None and self.learning_action_keepalive_s > 0.0:
            self.learning_action_timer = self.create_timer(
                self.learning_action_keepalive_s, self._publish_learning_action_keepalive
            )

        self.stream_write_timer = None
        if enable_command_subscription and self.stream_command_async_write:
            write_period = 1.0 / max(1.0, self.stream_write_rate_hz)
            self.stream_write_timer = self.create_timer(write_period, self.flush_latest_stream_command)
            self.get_logger().info(
                f"Async stream writer enabled at {1.0 / write_period:.1f} Hz"
            )
            if self.stream_target_smoothing:
                self.get_logger().info(
                    f"Stream target smoothing enabled: max_velocity={self.stream_max_velocity_rad_s:.3f} rad/s"
                )
            if self.stream_continuous_follow:
                self.get_logger().info("Continuous stream follow enabled")

        # JointState 发布器
        self.joint_state_pub = self.create_publisher(JointState, state_topic, 10)

        if state_rate > 0.0:
            period = 1.0 / state_rate
        else:
            period = 0.02

        self.state_timer = self.create_timer(period, self.publish_joint_states)

        # Real position readback: update current_positions from servo feedback.
        # Publish timer reads current_positions (no bus read in publish callback).
        self.readback_timer = None
        self._readback_fail_count = 0
        self._readback_success_count = 0
        self._consecutive_zero_update_cycles = 0
        self._readback_rr_idx = 0
        self._suspend_readback_until = 0.0
        if enable_readback:
            if readback_rate <= 0.0:
                self.get_logger().warn(
                    "enable_position_readback is true but position_readback_rate_hz <= 0; disabling readback"
                )
            else:
                readback_period = 1.0 / float(readback_rate)
                self.readback_timer = self.create_timer(
                    readback_period, self.update_positions_from_readback
                )
                if self.readback_mode not in ("round_robin", "all"):
                    self.get_logger().warn(
                        "position_readback_mode must be 'round_robin' or 'all'; using 'round_robin'"
                    )
                    self.readback_mode = "round_robin"

                if self.readback_mode == "all" and readback_rate > 20.0:
                    self.get_logger().warn(
                        "position_readback_mode=all with high rate may overload the bus; consider 5-15Hz"
                    )
                if self.readback_mode == "round_robin" and readback_rate > 200.0:
                    self.get_logger().warn(
                        "position_readback_rate_hz is very high; consider 50-100Hz for round_robin"
                    )

        # MoveIt 轨迹执行：FollowJointTrajectory 动作服务
        self.fjt_action_server = None
        if enable_fjt:
            # 注意：如果节点运行在 namespace "follower" 下，这里的动作名会变成
            # /follower/arm_trajectory_controller/follow_joint_trajectory
            self.fjt_action_server = ActionServer(
                self,
                FollowJointTrajectory,
                "arm_trajectory_controller/follow_joint_trajectory",
                execute_callback=self.execute_trajectory_callback,
            )
            self.get_logger().info(
                "FollowJointTrajectory action server ready at "
                "'arm_trajectory_controller/follow_joint_trajectory'"
            )

        # MoveIt gripper execution: control_msgs/ParallelGripperCommand
        # MoveItSimpleControllerManager config expects:
        #   controller: follower/gripper_controller
        #   action_ns: gripper_cmd
        # With node namespace "follower", the full action name becomes:
        #   /follower/gripper_controller/gripper_cmd
        self.gripper_action_server = None
        if enable_gripper_action:
            self.gripper_action_server = ActionServer(
                self,
                ParallelGripperCommand,
                "gripper_controller/gripper_cmd",
                execute_callback=self.execute_gripper_callback,
            )
            self.get_logger().info(
                "ParallelGripperCommand action server ready at 'gripper_controller/gripper_cmd'"
            )

    # 将一组关节角（rad）转换为总线舵机位置并下发
    def send_positions(self, joint_names: List[str], positions_rad: List[float], duration: float) -> None:
        if len(joint_names) != len(positions_rad):
            self.get_logger().warn(
                f"send_positions: length mismatch {len(joint_names)} vs {len(positions_rad)}"
            )
            return

        bus_positions = []
        if self.servo_range_deg <= 0.0:
            self.get_logger().error("servo_range_deg must be > 0.0")
            return

        servo_span = float(self.servo_pos_max - self.servo_pos_min)
        if servo_span <= 0.0:
            self.get_logger().error("servo_pos_max must be > servo_pos_min")
            return

        # deg -> pos
        pos_per_deg = servo_span / self.servo_range_deg
        for name, angle_rad in zip(joint_names, positions_rad):
            if name not in JOINT_ID_MAP:
                continue
            deadband = (
                self.gripper_command_deadband_rad
                if name == "gripper"
                else self.command_position_deadband_rad
            )
            last_sent = self._last_sent_positions_rad.get(name)
            if last_sent is not None and abs(angle_rad - last_sent) <= deadband:
                continue
            servo_id = JOINT_ID_MAP[name]
            angle_deg = angle_rad * 180.0 / math.pi
            direction = int(self.joint_directions.get(name, 1))
            if direction not in (-1, 1):
                direction = 1
            zero_pos = float(self.joint_zero_positions.get(name, self.servo_zero_pos))

            # 默认: 0 rad -> pos=500(中位), +/-120deg -> 0..1000
            pos_unclamped = zero_pos + direction * (angle_deg * pos_per_deg)
            pos = max(float(self.servo_pos_min), min(float(self.servo_pos_max), pos_unclamped))
            pos_int = int(round(pos))
            bus_positions.append([servo_id, pos_int])

            if name == "gripper":
                log_fn = self.get_logger().info if self.log_gripper_mapping else self.get_logger().debug
                log_fn(
                    "Gripper command mapping: "
                    f"target_rad={angle_rad:+.3f}, direction={direction}, "
                    f"zero_pos={zero_pos:.1f}, servo_pos={pos_int}, duration={duration:.3f}s"
                )

            # 记录当前姿态，供 publish_joint_states 使用
            try:
                idx = self.joint_names.index(name)
                # If we had to clip, update published joint angle to match the clipped command.
                if pos != pos_unclamped and name not in self._clip_warned:
                    self.get_logger().warn(
                        f"Clipped joint '{name}' command: rad={angle_rad:.3f} -> "
                        f"pos={pos_unclamped:.1f} clamped to [{self.servo_pos_min}, {self.servo_pos_max}] -> {pos:.1f}. "
                        "Check joint_directions/joint_zero_positions/servo_range_deg and mechanical limits."
                    )
                    self._clip_warned.add(name)

                effective_angle_rad = angle_rad
                if pos != pos_unclamped and pos_per_deg > 0.0:
                    # invert: angle_deg = (pos - zero_pos) / (direction * pos_per_deg)
                    effective_angle_deg = (pos - zero_pos) / (direction * pos_per_deg)
                    effective_angle_rad = effective_angle_deg * math.pi / 180.0

                self.current_positions[idx] = self._clamp_joint_position(name, effective_angle_rad)
            except ValueError:
                pass

        if not bus_positions:
            return

        try:
            self.board.bus_servo_set_position(duration, bus_positions)
            self._write_fail_count = 0
            for name, angle_rad in zip(joint_names, positions_rad):
                if name in JOINT_ID_MAP:
                    self._last_sent_positions_rad[name] = angle_rad
        except Exception as exc:  # noqa: BLE001
            self._write_fail_count += 1
            if self._shutting_down or not rclpy.ok():
                return
            if self._write_fail_count == 1 or self._write_fail_count % 20 == 0:
                self.get_logger().error(
                    "Failed to send bus_servo_set_position "
                    f"(count={self._write_fail_count}): {exc}"
                )

    def execute_gripper_callback(self, goal_handle):
        """ParallelGripperCommand 动作执行回调（用于 MoveIt gripper_controller）."""
        command = goal_handle.request.command
        target = 0.0
        if command.position:
            target = float(command.position[0])

        duration = float(self.move_duration)
        self.get_logger().info(
            f"Received ParallelGripperCommand goal: target={target:+.3f} rad, "
            f"duration={duration:.3f}s"
        )
        self.send_positions(["gripper"], [target], duration)

        # Best-effort wait.
        start = time.time()
        timeout_s = max(1.0, duration * 3.0)
        reached = False
        try:
            idx = self.joint_names.index("gripper")
        except ValueError:
            idx = None

        while time.time() - start < timeout_s:
            if idx is not None:
                cur = float(self.current_positions[idx])
                if abs(cur - target) < 0.05:
                    reached = True
                    break
            time.sleep(0.02)

        if idx is not None:
            cur = float(self.current_positions[idx])
            self.get_logger().info(
                f"Gripper goal complete: target={target:+.3f} rad, "
                f"current={cur:+.3f} rad, reached={reached}"
            )
        else:
            self.get_logger().warn("Gripper joint is not present in joint_names; reporting best-effort success")

        goal_handle.succeed()

        result = ParallelGripperCommand.Result()
        result.state.name = ["gripper"]
        result.state.position = [target]
        result.state.velocity = [0.0]
        result.state.effort = [0.0]
        result.stalled = False
        result.reached_goal = reached
        return result

    def command_callback(self, msg: Float64MultiArray) -> None:
        # 用于简单 forward_controller 指令
        if len(msg.data) == len(self.joint_names) - 1 and "gripper" in self.joint_names:
            # Arm-only commands from cartesian_motion_node intentionally omit the
            # gripper so the gripper action server remains the single owner.
            joint_names = [name for name in self.joint_names if name != "gripper"]
        elif len(msg.data) == len(self.joint_names):
            joint_names = self.joint_names
        else:
            self.get_logger().warn(
                f"Command length {len(msg.data)} does not match arm-only "
                f"({len(self.joint_names) - 1}) or full ({len(self.joint_names)}) joint command length"
            )
            return

        duration = self.stream_command_duration
        if duration <= 0.0:
            duration = self.move_duration
        if self.suspend_readback_after_stream_command_s > 0.0:
            self._suspend_readback_until = max(
                self._suspend_readback_until,
                time.monotonic() + self.suspend_readback_after_stream_command_s,
            )
        positions = list(msg.data)
        self.publish_learning_action(joint_names, positions)
        if self.stream_command_async_write:
            self._pending_stream_joint_names = joint_names
            self._pending_stream_positions = positions
            self._pending_stream_dirty = True
            return
        self.send_positions(joint_names, positions, duration)

    def flush_latest_stream_command(self) -> None:
        if not self._pending_stream_dirty:
            return
        joint_names = self._pending_stream_joint_names
        positions = self._pending_stream_positions
        if joint_names is None or positions is None:
            self._pending_stream_dirty = False
            return
        self._pending_stream_dirty = False
        duration = self.stream_command_duration
        if duration <= 0.0:
            duration = self.move_duration
        output_positions = self._smooth_stream_targets(list(joint_names), list(positions))
        self.send_positions(list(joint_names), output_positions, duration)
        if self.stream_continuous_follow:
            max_residual = 0.0
            for target, output in zip(positions, output_positions):
                max_residual = max(max_residual, abs(float(target) - float(output)))
            self._pending_stream_dirty = max_residual > self.command_position_deadband_rad
        else:
            self._pending_stream_dirty = False

    def _smooth_stream_targets(self, joint_names: List[str], target_positions: List[float]) -> List[float]:
        if not self.stream_target_smoothing or self.stream_max_velocity_rad_s <= 0.0:
            return target_positions

        now = time.monotonic()
        dt = max(1.0 / max(1.0, self.stream_write_rate_hz), now - self._last_stream_flush_time)
        self._last_stream_flush_time = now
        max_step = self.stream_max_velocity_rad_s * dt

        output_positions: List[float] = []
        for name, target in zip(joint_names, target_positions):
            previous = self._stream_output_positions.get(name, float(target))
            delta = float(target) - previous
            if delta > max_step:
                output = previous + max_step
            elif delta < -max_step:
                output = previous - max_step
            else:
                output = float(target)
            self._stream_output_positions[name] = output
            output_positions.append(output)
        return output_positions

    def execute_trajectory_callback(self, goal_handle):
        """FollowJointTrajectory 动作执行回调（用于 MoveIt）."""
        self.get_logger().info("Received FollowJointTrajectory goal")

        traj = goal_handle.request.trajectory
        if not traj.points:
            self.get_logger().warn("Trajectory has no points, aborting")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # 确定本次执行使用的关节顺序：trajectory 中的 joint_names
        joint_names = list(traj.joint_names)
        # 过滤出我们实际支持的关节
        valid_indices: List[int] = []
        mapped_joint_names: List[str] = []
        for idx, name in enumerate(joint_names):
            if name in JOINT_ID_MAP:
                valid_indices.append(idx)
                mapped_joint_names.append(name)

        if not valid_indices:
            self.get_logger().error("No valid joints in trajectory for HX-35HM bridge")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        filtered_points = []
        for point in traj.points:
            assert isinstance(point, JointTrajectoryPoint)
            if len(point.positions) < len(joint_names):
                self.get_logger().warn("Trajectory point has fewer positions than joint_names")
                continue

            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            positions_rad = [float(point.positions[i]) for i in valid_indices]
            velocities_rad = None
            if len(point.velocities) >= len(joint_names):
                velocities_rad = [float(point.velocities[i]) for i in valid_indices]
            filtered_points.append((t, positions_rad, velocities_rad))

        if not filtered_points:
            self.get_logger().error("No valid trajectory points after filtering")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        total_duration = filtered_points[-1][0]
        if len(filtered_points) == 1 or total_duration <= 1e-6:
            single_point_duration = max(
                self.trajectory_min_total_duration_s,
                self.trajectory_final_settle_s,
                self.move_duration,
                0.2,
            )
            try:
                current_by_name = {
                    name: self.current_positions[self.joint_names.index(name)]
                    for name in mapped_joint_names
                    if name in self.joint_names
                }
                final_by_name = {
                    name: filtered_points[-1][1][idx]
                    for idx, name in enumerate(mapped_joint_names)
                }
                current_summary = ", ".join(
                    f"{name}={current_by_name.get(name, 0.0):+.3f}" for name in mapped_joint_names
                )
                final_summary = ", ".join(
                    f"{name}={final_by_name.get(name, 0.0):+.3f}" for name in mapped_joint_names
                )
                self.get_logger().info(f"Single-point current joints: {current_summary}")
                self.get_logger().info(f"Single-point final joints:   {final_summary}")
            except Exception:
                pass
            self.get_logger().info(
                f"Executing single-point trajectory on {mapped_joint_names} over "
                f"{single_point_duration:.3f}s"
            )
            self.publish_learning_action(mapped_joint_names, filtered_points[-1][1])
            self.send_positions(mapped_joint_names, filtered_points[-1][1], single_point_duration)
            settle_deadline = time.monotonic() + single_point_duration
            while time.monotonic() < settle_deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result()
                time.sleep(0.01)

            self.get_logger().info(
                f"FollowJointTrajectory goal succeeded with {len(traj.points)} points"
            )
            goal_handle.succeed()
            return FollowJointTrajectory.Result()

        if self.trajectory_min_total_duration_s > 0.0 and total_duration < self.trajectory_min_total_duration_s:
            raw_duration = max(total_duration, 1e-6)
            scale = self.trajectory_min_total_duration_s / raw_duration
            filtered_points = [
                (t * scale, positions, velocities)
                for (t, positions, velocities) in filtered_points
            ]
            total_duration = filtered_points[-1][0]
            self.get_logger().info(
                f"Stretching short trajectory from {raw_duration:.3f}s to {total_duration:.3f}s "
                f"(scale={scale:.2f}) for physical execution"
            )

        sample_dt = 0.0
        if self.trajectory_command_rate_hz > 0.0:
            sample_dt = 1.0 / self.trajectory_command_rate_hz
        sample_dt = max(sample_dt, self.trajectory_min_command_interval_s)
        sample_dt = min(sample_dt, self.trajectory_min_segment_duration_s) if self.trajectory_min_segment_duration_s > 0.0 else sample_dt
        if sample_dt <= 0.0:
            sample_dt = 0.02

        self.get_logger().info(
            f"Executing trajectory on {mapped_joint_names} with {len(filtered_points)} points "
            f"over {total_duration:.3f}s (sample_dt={sample_dt:.3f}s)"
        )
        final_positions = filtered_points[-1][1]
        self.publish_learning_action(mapped_joint_names, final_positions)
        try:
            current_by_name = {
                name: self.current_positions[self.joint_names.index(name)]
                for name in mapped_joint_names
                if name in self.joint_names
            }
            final_by_name = {
                name: final_positions[idx]
                for idx, name in enumerate(mapped_joint_names)
            }
            current_summary = ", ".join(
                f"{name}={current_by_name.get(name, 0.0):+.3f}" for name in mapped_joint_names
            )
            final_summary = ", ".join(
                f"{name}={final_by_name.get(name, 0.0):+.3f}" for name in mapped_joint_names
            )
            self.get_logger().info(f"Trajectory current joints: {current_summary}")
            self.get_logger().info(f"Trajectory final joints:   {final_summary}")
        except Exception:
            pass

        if self.suspend_readback_during_trajectory:
            # Writing and reading on the same serial bus during dense trajectory
            # execution tends to introduce visible micro-stalls.
            self._suspend_readback_until = time.monotonic() + max(total_duration + 0.5, 1.0)

        start_wall_time = time.monotonic()
        sample_times = self._build_sample_times(total_duration, sample_dt)
        for sample_index, sample_time in enumerate(sample_times):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            wake_time = start_wall_time + sample_time
            remaining = wake_time - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

            target_positions = self._sample_trajectory_positions(filtered_points, sample_time)
            next_time = max(
                self.trajectory_min_command_interval_s,
                self._next_sample_delta(sample_times, sample_index),
            )
            self.send_positions(mapped_joint_names, target_positions, next_time)

        # Make sure the final point is resent with a small settle time.
        final_settle = max(self.trajectory_min_command_interval_s, self.trajectory_final_settle_s, 0.03)
        self.send_positions(mapped_joint_names, final_positions, final_settle)
        settle_deadline = time.monotonic() + final_settle
        while time.monotonic() < settle_deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            time.sleep(0.01)

        self.get_logger().info(
            f"FollowJointTrajectory goal succeeded with {len(traj.points)} points"
        )
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def _build_sample_times(self, total_duration: float, sample_dt: float) -> List[float]:
        if total_duration <= 0.0:
            return [0.0]
        if sample_dt <= 0.0:
            return [0.0, total_duration]

        sample_times: List[float] = [0.0]
        t = sample_dt
        while t < total_duration:
            sample_times.append(t)
            t += sample_dt
        if sample_times[-1] != total_duration:
            sample_times.append(total_duration)
        return sample_times

    def _next_sample_delta(self, sample_times: List[float], sample_index: int) -> float:
        if sample_index + 1 < len(sample_times):
            return sample_times[sample_index + 1] - sample_times[sample_index]
        return self.trajectory_min_command_interval_s

    def _sample_trajectory_positions(
        self, filtered_points: List[tuple[float, List[float], List[float] | None]], sample_time: float
    ) -> List[float]:
        if sample_time <= filtered_points[0][0]:
            return list(filtered_points[0][1])

        for idx in range(1, len(filtered_points)):
            t1, p1, v1 = filtered_points[idx]
            if sample_time <= t1:
                t0, p0, v0 = filtered_points[idx - 1]
                dt = t1 - t0
                if dt <= 1e-6:
                    return list(p1)
                alpha = max(0.0, min(1.0, (sample_time - t0) / dt))

                # Prefer cubic Hermite interpolation when trajectory velocities are available.
                if v0 is not None and v1 is not None and len(v0) == len(p0) and len(v1) == len(p1):
                    a2 = alpha * alpha
                    a3 = a2 * alpha
                    h00 = 2.0 * a3 - 3.0 * a2 + 1.0
                    h10 = a3 - 2.0 * a2 + alpha
                    h01 = -2.0 * a3 + 3.0 * a2
                    h11 = a3 - a2
                    return [
                        h00 * p0[j] + h10 * dt * v0[j] + h01 * p1[j] + h11 * dt * v1[j]
                        for j in range(len(p0))
                    ]

                return [p0[j] + alpha * (p1[j] - p0[j]) for j in range(len(p0))]

        return list(filtered_points[-1][1])

    def _clamp_joint_position(self, joint_name: str, angle_rad: float) -> float:
        limits = JOINT_LIMITS_RAD.get(joint_name)
        if limits is None:
            return angle_rad

        lower, upper = limits
        # Keep a tiny margin inside the MoveIt bounds so start-state checks do not
        # reject a pose due to floating-point noise or encoder jitter near the limit.
        epsilon = 1e-4
        clamped = min(max(angle_rad, lower + epsilon), upper - epsilon)
        if clamped != angle_rad and joint_name not in self._joint_limit_warned:
            self.get_logger().warn(
                f"Clamped readback for joint '{joint_name}' from {angle_rad:.4f} rad "
                f"into MoveIt limits [{lower:.4f}, {upper:.4f}]"
            )
            self._joint_limit_warned.add(joint_name)
        return clamped

    def _do_initial_readback(self) -> None:
        """启动时立即读取所有舵机位置，确保MoveIt获得正确的初始状态"""
        servo_span = float(self.servo_pos_max - self.servo_pos_min)
        if servo_span <= 0.0 or self.servo_range_deg <= 0.0:
            self.get_logger().warn("Cannot do initial readback: invalid servo parameters")
            return

        pos_per_deg = servo_span / self.servo_range_deg
        
        self.get_logger().info("Performing initial servo position readback...")
        
        for idx, joint_name, servo_id in self._readback_targets:
            try:
                state = self.board.bus_servo_read_position(servo_id, timeout=0.5)
                if state:
                    pos = float(state[0])
                    direction = int(self.joint_directions.get(joint_name, 1))
                    zero_pos = float(self.joint_zero_positions.get(joint_name, self.servo_zero_pos))
                    
                    angle_deg = (pos - zero_pos) / (direction * pos_per_deg)
                    angle_rad = angle_deg * math.pi / 180.0
                    if self.clamp_readback_to_joint_limits:
                        angle_rad = self._clamp_joint_position(joint_name, angle_rad)
                    self.current_positions[idx] = angle_rad
                    
                    self.get_logger().info(
                        f"Initial readback: {joint_name} servo_id={servo_id} pos={pos} -> {angle_rad:.4f} rad"
                    )
            except Exception as exc:
                self.get_logger().warn(f"Failed initial readback for {joint_name}: {exc}")
        
        self._initial_readback_done = True
        self.get_logger().info("Initial readback complete")

    def _set_torque_enabled(self, enable: bool, *, context: str) -> None:
        if not self.torque_servo_ids:
            self.get_logger().warn(f"No torque_servo_ids configured; skipping torque change ({context})")
            return

        action = "Enabling" if enable else "Disabling"
        self.get_logger().info(
            f"{action} torque for servo IDs {self.torque_servo_ids} during {context}"
        )
        for attempt in range(self.torque_command_retries):
            for servo_id in self.torque_servo_ids:
                try:
                    self.board.bus_servo_enable_torque(int(servo_id), 1 if enable else 0)
                    time.sleep(0.02)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f"Failed to change torque state for servo {servo_id} during {context}: {exc}"
                    )
            if attempt != self.torque_command_retries - 1:
                time.sleep(max(0.0, self.torque_command_interval_s))

    def _start_torque_disable_keepalive(self) -> None:
        if not self.maintain_torque_disabled:
            return
        if self.torque_disable_keepalive_rate_hz <= 0.0:
            self.get_logger().warn(
                "maintain_torque_disabled is true but torque_disable_keepalive_rate_hz <= 0; "
                "skipping torque disable keepalive"
            )
            return
        if self._torque_keepalive_timer is not None:
            return

        period = 1.0 / self.torque_disable_keepalive_rate_hz
        self.get_logger().info(
            "Starting torque-disable keepalive "
            f"at {self.torque_disable_keepalive_rate_hz:.2f} Hz"
        )
        self._torque_keepalive_timer = self.create_timer(period, self._torque_disable_keepalive_cb)

    def _torque_disable_keepalive_cb(self) -> None:
        if self._shutting_down or not self._torque_disabled_by_startup:
            return
        try:
            self._set_torque_enabled(False, context="keepalive")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Torque-disable keepalive failed: {exc}")

    def publish_joint_states(self) -> None:
        # Publish the latest estimate of joint positions.
        # If position readback is enabled, this is updated from servo feedback.
        # Otherwise it mirrors the last commanded positions.
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.header.frame_id = "base_link"  # Add frame_id to avoid TF warnings
        js.name = list(self.joint_names)
        js.position = list(self.current_positions)
        js.velocity = [0.0] * len(self.current_positions)  # Add velocity estimates
        js.effort = [0.0] * len(self.current_positions)   # Add effort estimates
        self.joint_state_pub.publish(js)

    def publish_learning_action(self, joint_names: List[str], positions_rad: List[float]) -> None:
        if self.learning_action_pub is None:
            return

        by_name = {
            name: float(position)
            for name, position in zip(joint_names, positions_rad)
            if name in JOINT_ID_MAP
        }
        if not by_name:
            return

        action = []
        for name in self.joint_names:
            if name in by_name:
                action.append(by_name[name])
            else:
                try:
                    action.append(float(self.current_positions[self.joint_names.index(name)]))
                except ValueError:
                    action.append(0.0)

        msg = Float64MultiArray()
        msg.data = action
        self.learning_action_pub.publish(msg)
        self._last_learning_action = action

    def _publish_learning_action_keepalive(self) -> None:
        if self.learning_action_pub is None:
            return

        action = self._last_learning_action
        if action is None:
            action = [float(position) for position in self.current_positions]
            self._last_learning_action = action

        msg = Float64MultiArray()
        msg.data = list(action)
        self.learning_action_pub.publish(msg)

    def update_positions_from_readback(self) -> None:
        # Read servo positions sequentially with a timeout (avoid blocking the executor).
        if time.monotonic() < self._suspend_readback_until:
            return

        servo_span = float(self.servo_pos_max - self.servo_pos_min)
        if servo_span <= 0.0 or self.servo_range_deg <= 0.0:
            return

        pos_per_deg = servo_span / self.servo_range_deg
        if pos_per_deg <= 0.0:
            return

        # Use the already stored targets from initialization
        targets = self._readback_targets

        updated = 0
        if self.readback_mode == "all":
            read_targets = list(targets)
        else:
            # round_robin
            self._readback_rr_idx = (self._readback_rr_idx + 1) % len(targets)
            read_targets = [targets[self._readback_rr_idx]]

        for idx, joint_name, servo_id in read_targets:

            state = None
            try:
                # Official SDK supports timeout parameter, so use it directly
                state = self.board.bus_servo_read_position(servo_id, timeout=self.readback_timeout_s)
            except Exception as exc:  # noqa: BLE001
                # Serial issues should not kill the node; keep last known position.
                self._readback_fail_count += 1
                if self._readback_fail_count % 50 == 1:
                    self.get_logger().warn(f"bus_servo_read_position failed for id={servo_id}: {exc}")
                continue

            if not state:
                self._readback_fail_count += 1
                continue

            try:
                pos = float(state[0])
            except Exception:  # noqa: BLE001
                self._readback_fail_count += 1
                continue

            direction = int(self.joint_directions.get(joint_name, 1))
            if direction not in (-1, 1):
                direction = 1
            zero_pos = float(self.joint_zero_positions.get(joint_name, self.servo_zero_pos))

            # invert mapping: angle_deg = (pos - zero_pos) / (direction * pos_per_deg)
            angle_deg = (pos - zero_pos) / (direction * pos_per_deg)
            angle_rad = angle_deg * math.pi / 180.0
            if self.clamp_readback_to_joint_limits:
                angle_rad = self._clamp_joint_position(joint_name, angle_rad)
            self.current_positions[idx] = angle_rad
            updated += 1
            self._readback_success_count += 1

        if updated > 0:
            self._consecutive_zero_update_cycles = 0
            return

        self._consecutive_zero_update_cycles += 1
        # Round-robin mode often sees transient misses on individual polls; only
        # escalate after a sustained run of zero-update cycles to avoid noisy logs.
        warn_every = 50 if self.readback_mode == "round_robin" else 10
        if self._consecutive_zero_update_cycles % warn_every != 1:
            return

        try:
            import os

            device_param = self.get_parameter("device").get_parameter_value().string_value
            if not os.path.exists(device_param):
                return
        except Exception:
            return

        level = self.get_logger().warn if self._readback_success_count > 0 else self.get_logger().info
        level(
            "Position readback updated 0 joints for multiple cycles. "
            "If some servos are intentionally absent this can be benign; "
            "otherwise check wiring, power, servo IDs, and baudrate."
        )


def main() -> None:
    rclpy.init()
    node = Hx35hmBridgeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._shutting_down = True
        executor.shutdown()
        if node.restore_torque_on_shutdown and node._torque_disabled_by_startup:
            try:
                node._set_torque_enabled(True, context="shutdown")
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
