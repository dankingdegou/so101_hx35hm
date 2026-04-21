# Copyright 2026 Dmitri Manajev
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Cartesian motion node.

Three ways to drive the arm:

- /go_to_pose service — plans a trajectory to a Cartesian pose (IK under
  the hood) and streams it to the controller until the final point is hit.
- /go_to_joints service — plans a quintic trajectory directly in joint
  space to the requested joint target (no IK).
- /servo_target topic — single-step IK toward the latest PoseStamped, when
  no trajectory is active.

Trajectory mode wins absolutely over servo mode.

Services:
    /go_to_pose    (so101_kinematics_msgs/srv/GoToPose)
    /go_to_joints  (so101_kinematics_msgs/srv/GoToJoints)

Subscribes:
    /follower/joint_states  (sensor_msgs/JointState)
    /servo_target           (geometry_msgs/PoseStamped) — interactive IK target

Publishes (50 Hz while a trajectory or fresh servo target is active):
    /follower/forward_controller/commands  (std_msgs/Float64MultiArray)

Notes
-----
- target.header.frame_id (both service and topic) must match `base_frame`
  (default follower/base_link). Other frames are rejected; TF transforming
  can be added later if needed.
- /servo_target poses go stale after `_servo_stale_sec` (0.2 s). If the
  publisher dies, the timer stops issuing commands.
- Incoming /servo_target poses are dropped while a trajectory is executing;
  the planned motion always wins.
- The gripper joint is not commanded by this node — it is held at the last
  measured position so other nodes (or a future gripper service) can own it.
  /go_to_joints may include "gripper" explicitly if the caller wants to
  move it as part of the trajectory.
"""

import tempfile
import time
import threading
from typing import Optional
from pathlib import Path

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import get_package_share_directory
import xacro
from scipy.spatial.transform import Rotation

from robokin.placo import PlacoKinematics, PlacoConfig
from robokin.trajectory_executor import TrajectoryExecutor
from so101_kinematics.motion_planner import MotionPlanner

from so101_kinematics_msgs.srv import GoToPose, GoToJoints


EE_FRAME = "gripper_frame_link"
DT = 1.0 / 50.0


def _pose_to_matrix(pose) -> np.ndarray:
    q = pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    return T


class CartesianMotionNode(Node):
    def __init__(self):
        super().__init__("cartesian_motion_node")

        # ── Parameters ──
        self.declare_parameter("joints_topic", "/follower/joint_states")
        self.declare_parameter("cmd_topic", "/follower/forward_controller/commands")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("robot_description_package", "so101_description")
        self.declare_parameter("robot_description_xacro", "urdf/so101_arm.urdf.xacro")
        self.declare_parameter("robot_variant", "follower")
        self.declare_parameter("goal_wait_timeout_s", 1.2)
        self.declare_parameter("goal_position_tolerance_m", 0.02)
        self.declare_parameter("goal_orientation_tolerance_rad", 0.20)
        self.declare_parameter("fail_on_goal_tolerance", False)

        self._base_frame = str(self.get_parameter("base_frame").value)
        self._goal_wait_timeout_s = float(self.get_parameter("goal_wait_timeout_s").value)
        self._goal_position_tolerance_m = float(
            self.get_parameter("goal_position_tolerance_m").value
        )
        self._goal_orientation_tolerance_rad = float(
            self.get_parameter("goal_orientation_tolerance_rad").value
        )
        self._fail_on_goal_tolerance = bool(
            self.get_parameter("fail_on_goal_tolerance").value
        )

        # ── Solver / planner ──
        urdf_path = self._render_robot_urdf(
            package_name=str(self.get_parameter("robot_description_package").value),
            xacro_relpath=str(self.get_parameter("robot_description_xacro").value),
            variant=str(self.get_parameter("robot_variant").value),
        )
        self.solver = PlacoKinematics(
            urdf_path=str(urdf_path),
            ee_frame=EE_FRAME,
            cfg=PlacoConfig(dt=DT),
        )
        self.joint_names = self.solver.joint_names
        self.gripper_index = self.joint_names.index("gripper")
        self.planner = MotionPlanner(self.solver)
        self.traj_executor = TrajectoryExecutor()

        # ── State ──
        self._q_measured: Optional[np.ndarray] = None
        self._q_held: Optional[np.ndarray] = None
        self._traj_lock = threading.Lock()  # serialize service calls

        # Active-goal handoff between service handler (producer) and timer
        # (consumer). The handler hands off the trajectory and waits on
        # _goal_done; the timer drives the executor and signals _goal_done
        # when the trajectory's final sample has been published.
        self._goal_active = False
        self._goal_commands_gripper = False
        self._goal_done = threading.Event()

        # Servo target: latest PoseStamped from /servo_target. Only consumed
        # when no trajectory is active. Cleared when a trajectory starts.
        self._servo_T: Optional[np.ndarray] = None
        self._servo_stamp_ns: Optional[int] = None
        self._servo_stale_sec = 0.2  # drop servo cmd if target older than this
        self._wrong_frame_warn_ns: int = 0  # rate-limit wrong-frame warnings

        # ── ROS I/O ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        cb_group = ReentrantCallbackGroup()
        self.joint_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joints_topic").value),
            self._on_joints, sensor_qos, callback_group=cb_group,
        )
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("cmd_topic").value),
            10,
        )
        self.servo_sub = self.create_subscription(
            PoseStamped, "servo_target", self._on_servo_target, 1,
            callback_group=cb_group,
        )
        self.timer = self.create_timer(DT, self._tick, callback_group=cb_group)
        self.go_to_pose_srv = self.create_service(
            GoToPose, "go_to_pose", self._on_go_to_pose,
            callback_group=cb_group,
        )
        self.go_to_joints_srv = self.create_service(
            GoToJoints, "go_to_joints", self._on_go_to_joints,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"cartesian_motion_node up — service /go_to_pose, "
            f"cmd_topic={self.get_parameter('cmd_topic').value}"
        )

    def _render_robot_urdf(
        self,
        package_name: str,
        xacro_relpath: str,
        variant: str,
    ) -> str:
        package_share = Path(get_package_share_directory(package_name))
        xacro_path = package_share / xacro_relpath
        if not xacro_path.exists():
            raise FileNotFoundError(
                f"Robot xacro not found: {xacro_path}"
            )

        doc = xacro.process_file(
            str(xacro_path),
            mappings={
                "variant": variant,
                "use_ros2_control": "false",
                "add_transmissions": "false",
            },
        )
        tmp = tempfile.NamedTemporaryFile(
            prefix="so101_kinematics_",
            suffix=".urdf",
            delete=False,
            mode="w",
            encoding="utf-8",
        )
        try:
            tmp.write(doc.toprettyxml(indent="  "))
            tmp.flush()
        finally:
            tmp.close()

        self.get_logger().info(
            f"Using generated URDF for robokin: {tmp.name}"
        )
        return tmp.name

    # ── callbacks ──

    def _on_joints(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        q = self._q_measured.copy() if self._q_measured is not None \
            else np.zeros(len(self.joint_names))
        for i, name in enumerate(self.joint_names):
            if name in name_to_pos:
                q[i] = float(name_to_pos[name])
        self._q_measured = q
        if self._q_held is None:
            self._q_held = q.copy()
        elif not self._goal_active:
            # The gripper is usually owned by its action server, not this Cartesian
            # node. Keep the held gripper value synced while idle so a later arm-only
            # /go_to_pose does not overwrite a just-closed gripper with an old value.
            self._q_held[self.gripper_index] = q[self.gripper_index]

    def _on_servo_target(self, msg: PoseStamped):
        frame = msg.header.frame_id or self._base_frame
        if frame != self._base_frame:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._wrong_frame_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"servo_target frame '{frame}' != base_frame "
                    f"'{self._base_frame}' — dropped"
                )
                self._wrong_frame_warn_ns = now_ns
            return
        # Trajectory absolutely wins — drop incoming poses while active.
        if self._goal_active:
            return
        self._servo_T = _pose_to_matrix(msg.pose)
        self._servo_stamp_ns = self.get_clock().now().nanoseconds

    def _tick(self):
        if self._q_held is None or self._q_measured is None:
            return

        # Trajectory mode.
        if self._goal_active:
            if not self.traj_executor.is_active():
                return
            q_cmd, reached_end = self.traj_executor.sample()
            self._q_held = q_cmd.copy()
            self._publish(q_cmd, include_gripper=self._goal_commands_gripper)
            if reached_end:
                self.traj_executor.cancel()
                self._goal_active = False
                self._goal_done.set()
            return

        # Servo mode — single-step IK toward latest /servo_target.
        if self._servo_T is None or self._servo_stamp_ns is None:
            return
        age_sec = (self.get_clock().now().nanoseconds - self._servo_stamp_ns) / 1e9
        if age_sec > self._servo_stale_sec:
            return
        q_cmd = self.solver.servo_step(self._q_measured, self._servo_T)
        q_cmd[self.gripper_index] = self._q_held[self.gripper_index]
        self._q_held = q_cmd.copy()
        self._publish(q_cmd, include_gripper=False)

    def _on_go_to_pose(self, request: GoToPose.Request, response: GoToPose.Response):
        if not self._traj_lock.acquire(blocking=False):
            response.success = False
            response.message = "Another trajectory is already executing"
            return response
        try:
            return self._execute_pose_goal(request, response)
        finally:
            self._traj_lock.release()

    def _on_go_to_joints(self, request: GoToJoints.Request, response: GoToJoints.Response):
        if not self._traj_lock.acquire(blocking=False):
            response.success = False
            response.message = "Another trajectory is already executing"
            return response
        try:
            return self._execute_joint_goal(request, response)
        finally:
            self._traj_lock.release()

    def _execute_pose_goal(self, request: GoToPose.Request, response: GoToPose.Response):
        if self._q_measured is None:
            response.success = False
            response.message = "No joint_states received yet"
            return response

        target_frame = request.target.header.frame_id or self._base_frame
        if target_frame != self._base_frame:
            response.success = False
            response.message = (
                f"target.header.frame_id='{target_frame}' must equal "
                f"base_frame='{self._base_frame}' (TF transform not implemented)"
            )
            return response

        T_goal = _pose_to_matrix(request.target.pose)
        strategy = request.strategy or "joint_quintic"
        kwargs = {}
        if request.duration > 0.0:
            kwargs["duration"] = float(request.duration)

        try:
            ts, qs = self.planner.plan_pose_move(
                self._q_measured.copy(), T_goal, strategy=strategy, **kwargs,
            )
        except Exception as e:
            response.success = False
            response.message = f"Planning failed: {e}"
            return response

        self._goal_commands_gripper = False
        self._sync_held_gripper_from_measured()
        response = self._start_trajectory_and_wait(
            ts, qs, response, label=f"go_to_pose strategy={strategy}"
        )
        if not response.success:
            return response

        reached, pos_err_m, rot_err_rad = self._wait_for_pose_convergence(T_goal)
        if not reached:
            response.message = (
                f"Pose not reached after trajectory: "
                f"pos_err={pos_err_m:.3f}m rot_err={rot_err_rad:.3f}rad"
            )
            self.get_logger().warn(response.message)
            response.success = not self._fail_on_goal_tolerance
            return response

        response.message = (
            f"{response.message}; final pose err "
            f"pos={pos_err_m:.3f}m rot={rot_err_rad:.3f}rad"
        )
        return response

    def _execute_joint_goal(self, request: GoToJoints.Request, response: GoToJoints.Response):
        if self._q_measured is None:
            response.success = False
            response.message = "No joint_states received yet"
            return response

        names = list(request.joint_names)
        positions = list(request.positions)
        if len(names) != len(positions):
            response.success = False
            response.message = (
                f"joint_names ({len(names)}) and positions "
                f"({len(positions)}) length mismatch"
            )
            return response
        if not names:
            response.success = False
            response.message = "joint_names is empty"
            return response

        unknown = [n for n in names if n not in self.joint_names]
        if unknown:
            response.success = False
            response.message = (
                f"Unknown joint names {unknown}; known: {self.joint_names}"
            )
            return response

        # Any joint not listed keeps its measured value. This lets callers
        # move a subset (e.g. leave the gripper alone).
        q_goal = self._q_measured.copy()
        requested = dict(zip(names, positions))
        for i, name in enumerate(self.joint_names):
            if name in requested:
                q_goal[i] = float(requested[name])

        kwargs = {}
        if request.duration > 0.0:
            kwargs["duration"] = float(request.duration)

        try:
            ts, qs = self.planner.plan_joint_move(
                self._q_measured.copy(), q_goal, **kwargs,
            )
        except Exception as e:
            response.success = False
            response.message = f"Planning failed: {e}"
            return response

        self._goal_commands_gripper = "gripper" in requested
        if not self._goal_commands_gripper:
            self._sync_held_gripper_from_measured()
        return self._start_trajectory_and_wait(
            ts, qs, response, label=f"go_to_joints ({len(names)} joints)"
        )

    def _sync_held_gripper_from_measured(self):
        if self._q_held is None or self._q_measured is None:
            return
        self._q_held[self.gripper_index] = self._q_measured[self.gripper_index]

    def _start_trajectory_and_wait(self, ts, qs, response, label: str):
        """Shared trajectory handoff: hand off to timer, wait for completion."""
        # Drop any pending servo target so when the trajectory finishes the
        # arm holds at its final point instead of snapping back.
        self._servo_T = None
        self._servo_stamp_ns = None
        self._goal_done.clear()
        self.traj_executor.start(ts, qs)
        self._goal_active = True
        self.get_logger().info(
            f"{label}: duration={ts[-1]:.2f}s steps={len(ts)}"
        )

        total_timeout = float(ts[-1]) + 2.0
        finished = self._goal_done.wait(timeout=total_timeout)

        if not finished:
            self.traj_executor.cancel()
            self._goal_active = False
            response.success = False
            response.message = (
                f"Trajectory deadline exceeded ({total_timeout:.2f}s)"
            )
            return response

        response.success = True
        response.message = f"Trajectory complete ({ts[-1]:.2f}s, {len(ts)} steps)"
        return response

    def _publish(self, q_cmd: np.ndarray, include_gripper: bool = True):
        msg = Float64MultiArray()
        if include_gripper:
            msg.data = [float(v) for v in q_cmd]
        else:
            msg.data = [
                float(v)
                for i, v in enumerate(q_cmd)
                if i != self.gripper_index
            ]
        self.cmd_pub.publish(msg)

    def _current_ee_pose_matrix(self) -> Optional[np.ndarray]:
        if self._q_measured is None:
            return None
        return self.solver.fk(self._q_measured.copy())

    def _pose_error(self, T_goal: np.ndarray) -> tuple[float, float]:
        T_now = self._current_ee_pose_matrix()
        if T_now is None:
            return float("inf"), float("inf")

        pos_err_m = float(np.linalg.norm(T_now[:3, 3] - T_goal[:3, 3]))
        rot_delta = T_now[:3, :3].T @ T_goal[:3, :3]
        rot_err_rad = float(np.linalg.norm(Rotation.from_matrix(rot_delta).as_rotvec()))
        return pos_err_m, rot_err_rad

    def _wait_for_pose_convergence(self, T_goal: np.ndarray) -> tuple[bool, float, float]:
        deadline = time.time() + max(0.0, self._goal_wait_timeout_s)
        best_pos_err_m = float("inf")
        best_rot_err_rad = float("inf")

        while time.time() <= deadline:
            pos_err_m, rot_err_rad = self._pose_error(T_goal)
            best_pos_err_m = min(best_pos_err_m, pos_err_m)
            best_rot_err_rad = min(best_rot_err_rad, rot_err_rad)
            if (
                pos_err_m <= self._goal_position_tolerance_m
                and rot_err_rad <= self._goal_orientation_tolerance_rad
            ):
                return True, pos_err_m, rot_err_rad
            time.sleep(0.02)

        return False, best_pos_err_m, best_rot_err_rad


def main(args=None):
    rclpy.init(args=args)
    node = CartesianMotionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
