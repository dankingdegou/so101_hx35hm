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

"""Thin motion-planning layer on top of a robokin solver."""

import numpy as np
from robokin.transformations import ease_quintic


class MotionPlanner:
    """Thin planning layer around a robokin kinematics solver.

    Responsibilities:
    - Cartesian offline planning via solver.generate_segment()
    - Joint-space quintic planning via solver.solve_goal() + local quintic sampling

    Returns trajectories as (ts, qs):
      ts: shape [N]
      qs: shape [N, dof]
    """

    def __init__(self, solver, dt: float | None = None):
        self.solver = solver
        self.dt = float(dt if dt is not None else solver.cfg.dt)
        self._wrist_roll_index = (
            self.solver.joint_names.index("wrist_roll")
            if "wrist_roll" in self.solver.joint_names
            else None
        )

    def plan_pose_move(
        self,
        q_start: np.ndarray,
        T_goal: np.ndarray,
        strategy: str = "cartesian",
        **kwargs,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Unified entry point.

        strategy:
          - "cartesian"     -> solver.generate_segment(...)
          - "joint_quintic" -> solver.solve_goal(...) + quintic samples
        """
        if strategy == "cartesian":
            return self.plan_cartesian_segment(q_start, T_goal, **kwargs)
        if strategy == "joint_quintic":
            return self.plan_joint_quintic(q_start, T_goal, **kwargs)
        raise ValueError(
            f"Unknown strategy '{strategy}'. Expected 'cartesian' or 'joint_quintic'."
        )

    def plan_cartesian_segment(
        self,
        q_start: np.ndarray,
        T_goal: np.ndarray,
        n_steps: int | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Cartesian-interpolated trajectory using the solver's built-in segment generator.

        Returns (ts, qs) where ts is [N+1] and qs is [N+1, dof],
        starting at t=0 with q_start.
        """
        q_start = np.asarray(q_start, dtype=float)
        qs = self.solver.generate_segment(q_start, T_goal, n_steps=n_steps)
        # generate_segment returns steps 1..N, prepend start for t=0
        qs = np.vstack([q_start[np.newaxis], np.asarray(qs)])
        ts = self.dt * np.arange(len(qs))
        return ts, qs

    def plan_joint_quintic(
        self,
        q_start: np.ndarray,
        T_goal: np.ndarray,
        duration: float | None = None,
        n_iters: int = 100,
        max_joint_speed_rad_s: float | None = None,
        min_duration: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Solve IK once, then build a quintic joint-space trajectory.

        Falls back to solver.cfg for speed/duration defaults when not provided.

        Returns (ts, qs).
        """
        q_start = np.asarray(q_start, dtype=float)
        cfg = self.solver.cfg

        max_joint_speed_rad_s = (
            max_joint_speed_rad_s
            if max_joint_speed_rad_s is not None
            else getattr(cfg, "joint_max_speed_rad_s", 0.8)
        )
        min_duration = (
            min_duration
            if min_duration is not None
            else getattr(cfg, "joint_min_duration", 0.6)
        )

        q_goal = self._solve_goal_with_seed_search(
            q_start, T_goal, n_iters=n_iters,
        )

        if duration is None:
            duration = self._estimate_duration(
                q_start, q_goal, max_joint_speed_rad_s, min_duration,
            )

        return self._build_joint_quintic(q_start, q_goal, duration)

    def _solve_goal_with_seed_search(
        self,
        q_start: np.ndarray,
        T_goal: np.ndarray,
        n_iters: int = 100,
    ) -> np.ndarray:
        """Try a few wrist-roll-biased IK seeds and keep the least extreme result.

        The target end-effector pose stays exactly the same. We only vary the seed
        given to the IK solver so it can land on a different valid branch when the
        mechanism has that freedom. This is especially helpful for HX-35HM where
        wrist_roll near its hard limits tends to produce ugly or unsafe motions.
        """
        q_start = np.asarray(q_start, dtype=float)

        # Default single-seed behavior when the robot has no wrist_roll joint.
        if self._wrist_roll_index is None:
            return np.asarray(
                self.solver.solve_goal(q_start, T_goal, n_iters=n_iters),
                dtype=float,
            )

        wrist_idx = self._wrist_roll_index
        seed_targets = [
            float(q_start[wrist_idx]),
            0.0,
            -0.78539816339,
            0.78539816339,
            -1.57079632679,
            1.57079632679,
        ]

        seen = set()
        candidates: list[tuple[tuple[float, float], np.ndarray]] = []
        for wrist_seed in seed_targets:
            key = round(wrist_seed, 6)
            if key in seen:
                continue
            seen.add(key)

            q_seed = q_start.copy()
            q_seed[wrist_idx] = wrist_seed
            q_goal = np.asarray(
                self.solver.solve_goal(q_seed, T_goal, n_iters=n_iters),
                dtype=float,
            )
            # Prefer solutions that keep wrist_roll away from its extremes.
            # Secondary tie-breaker: stay close to the current arm state.
            score = (
                float(abs(q_goal[wrist_idx])),
                float(np.linalg.norm(q_goal - q_start)),
            )
            candidates.append((score, q_goal))

        if not candidates:
            return np.asarray(
                self.solver.solve_goal(q_start, T_goal, n_iters=n_iters),
                dtype=float,
            )

        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def plan_joint_move(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        duration: float | None = None,
        max_joint_speed_rad_s: float | None = None,
        min_duration: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Build a quintic trajectory directly between two joint vectors.

        This is used by /go_to_joints, where the caller already provides the
        desired joint target and no IK solve is needed.
        """
        q_start = np.asarray(q_start, dtype=float)
        q_goal = np.asarray(q_goal, dtype=float)
        if q_start.shape != q_goal.shape:
            raise ValueError(
                f"q_start shape {q_start.shape} does not match q_goal shape {q_goal.shape}"
            )

        cfg = self.solver.cfg
        max_joint_speed_rad_s = (
            max_joint_speed_rad_s
            if max_joint_speed_rad_s is not None
            else getattr(cfg, "joint_max_speed_rad_s", 0.8)
        )
        min_duration = (
            min_duration
            if min_duration is not None
            else getattr(cfg, "joint_min_duration", 0.6)
        )

        if duration is None:
            duration = self._estimate_duration(
                q_start, q_goal, max_joint_speed_rad_s, min_duration,
            )

        return self._build_joint_quintic(q_start, q_goal, duration)

    def _build_joint_quintic(
        self,
        q0: np.ndarray,
        q1: np.ndarray,
        duration: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Minimum-jerk quintic in joint space (C2, zero vel/accel at endpoints).

        Returns (ts [N], qs [N, dof]).
        """
        duration = max(duration, self.dt)
        ts = np.arange(0.0, duration + 0.5 * self.dt, self.dt)
        dq = q1 - q0
        qs = np.array([q0 + dq * ease_quintic(t / duration) for t in ts])
        return ts, qs

    @staticmethod
    def _estimate_duration(
        q0: np.ndarray,
        q1: np.ndarray,
        max_joint_speed_rad_s: float = 0.8,
        min_duration: float = 0.6,
    ) -> float:
        """Duration so the fastest joint stays under max_joint_speed_rad_s (peak)."""
        if max_joint_speed_rad_s <= 0.0:
            raise ValueError("max_joint_speed_rad_s must be > 0")
        dq_max = float(np.max(np.abs(q1 - q0)))
        # quintic peak factor = 15/8
        return max(min_duration, (15.0 / 8.0) * dq_max / max_joint_speed_rad_s)
