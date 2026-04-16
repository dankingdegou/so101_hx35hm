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
        self.declare_parameter("move_duration", 0.2)
        self.declare_parameter("publish_joint_states_topic", "/joint_states")
        self.declare_parameter("state_publish_rate_hz", 50.0)
        self.declare_parameter("trajectory_command_rate_hz", 30.0)
        self.declare_parameter("trajectory_min_command_interval_s", 0.03)
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

        self.get_logger().info(
            f"Connecting Board on {device}, subscribing commands from {cmd_topic}"
        )
        self.board = Board(device=device)
        self.board.enable_reception()

        # Warn once per joint if we have to clip commanded positions into servo_pos_min/max.
        self._clip_warned = set()

        # 当前关节的"已知姿态"（用于发布 joint_states），初始设为 0 rad，
        # 后续在 send_positions 中更新。
        self.current_positions: List[float] = [0.0 for _ in self.joint_names]
        
        # Store targets for readback (must be defined before _do_initial_readback)
        self._readback_targets = [
            (i, name, JOINT_ID_MAP[name])
            for i, name in enumerate(self.joint_names)
            if name in JOINT_ID_MAP
        ]
        
        # 启动时立即读取所有舵机位置，避免MoveIt使用错误的状态
        self._initial_readback_done = False
        self._do_initial_readback()
        
        # 初始化时间戳
        self.last_update_time = self.get_clock().now()

        # 简单命令接口：直接订阅 forward_controller 的 Float64MultiArray 命令
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, cmd_topic, self.command_callback, 10
        )

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

                self.current_positions[idx] = effective_angle_rad
            except ValueError:
                pass

        if not bus_positions:
            return

        try:
            self.board.bus_servo_set_position(duration, bus_positions)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to send bus_servo_set_position: {exc}")

    def execute_gripper_callback(self, goal_handle):
        """ParallelGripperCommand 动作执行回调（用于 MoveIt gripper_controller）."""
        command = goal_handle.request.command
        target = 0.0
        if command.position:
            target = float(command.position[0])

        duration = float(self.move_duration)
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
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(
                f"Command length {len(msg.data)} does not match joint_names length {len(self.joint_names)}"
            )
            return

        self.send_positions(self.joint_names, list(msg.data), self.move_duration)

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
            filtered_points.append((t, positions_rad))

        if not filtered_points:
            self.get_logger().error("No valid trajectory points after filtering")
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        total_duration = filtered_points[-1][0]
        sample_dt = 0.0
        if self.trajectory_command_rate_hz > 0.0:
            sample_dt = 1.0 / self.trajectory_command_rate_hz
        sample_dt = max(sample_dt, self.trajectory_min_command_interval_s)

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
            next_time = max(sample_dt, self._next_sample_delta(sample_times, sample_index))
            self.send_positions(mapped_joint_names, target_positions, next_time)

        # Make sure the final point is resent with a small settle time.
        final_positions = filtered_points[-1][1]
        self.send_positions(mapped_joint_names, final_positions, max(sample_dt, 0.05))

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
        self, filtered_points: List[tuple[float, List[float]]], sample_time: float
    ) -> List[float]:
        if sample_time <= filtered_points[0][0]:
            return list(filtered_points[0][1])

        for idx in range(1, len(filtered_points)):
            t1, p1 = filtered_points[idx]
            if sample_time <= t1:
                t0, p0 = filtered_points[idx - 1]
                dt = t1 - t0
                if dt <= 1e-6:
                    return list(p1)
                alpha = (sample_time - t0) / dt
                return [p0[j] + alpha * (p1[j] - p0[j]) for j in range(len(p0))]

        return list(filtered_points[-1][1])

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
                    self.current_positions[idx] = angle_rad
                    
                    self.get_logger().info(
                        f"Initial readback: {joint_name} servo_id={servo_id} pos={pos} -> {angle_rad:.4f} rad"
                    )
            except Exception as exc:
                self.get_logger().warn(f"Failed initial readback for {joint_name}: {exc}")
        
        self._initial_readback_done = True
        self.get_logger().info("Initial readback complete")

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
            self.current_positions[idx] = angle_rad
            updated += 1
            self._readback_success_count += 1

        if updated == 0 and self._readback_fail_count % 50 == 1:
            # Only warn if device is expected to be connected (not if it's disconnected)
            try:
                # Test if device is accessible
                import os
                device_param = self.get_parameter("device").get_parameter_value().string_value
                if os.path.exists(device_param):
                    self.get_logger().warn("Position readback updated 0 joints (timeouts?). Check wiring/baudrate/power.")
                else:
                    # Device file doesn't exist, don't spam warnings
                    pass
            except:
                # If we can't check, just continue
                pass


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
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
