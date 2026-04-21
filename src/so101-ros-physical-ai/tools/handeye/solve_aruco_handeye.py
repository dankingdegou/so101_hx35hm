#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


def pose_to_matrix(position_xyz: list[float], quaternion_xyzw: list[float]) -> np.ndarray:
    t = np.asarray(position_xyz, dtype=np.float64).reshape(3)
    q = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    q_norm = np.linalg.norm(q)
    if q_norm < 1e-12:
        raise ValueError("Invalid quaternion (norm too small).")
    q = q / q_norm
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def pack_params(Rx: np.ndarray, tx: np.ndarray, Rz: np.ndarray, tz: np.ndarray) -> np.ndarray:
    return np.concatenate(
        [
            R.from_matrix(Rx).as_rotvec(),
            tx.reshape(3),
            R.from_matrix(Rz).as_rotvec(),
            tz.reshape(3),
        ]
    )


def unpack_params(p: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    Rx = R.from_rotvec(p[0:3]).as_matrix()
    tx = p[3:6].reshape(3)
    Rz = R.from_rotvec(p[6:9]).as_matrix()
    tz = p[9:12].reshape(3)
    return Rx, tx, Rz, tz


def make_transform(Rm: np.ndarray, tm: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rm
    T[:3, 3] = tm
    return T


def residuals(
    p: np.ndarray,
    bTg_list: list[np.ndarray],
    cTm_list: list[np.ndarray],
    rot_weight: float,
) -> np.ndarray:
    Rx, tx, Rz, tz = unpack_params(p)
    X = make_transform(Rx, tx)  # gTm
    Z = make_transform(Rz, tz)  # bTc

    res = []
    for bTg, cTm in zip(bTg_list, cTm_list):
        lhs = bTg @ X
        rhs = Z @ cTm
        dT = np.linalg.inv(rhs) @ lhs
        dR = dT[:3, :3]
        dt = dT[:3, 3]
        rotvec = R.from_matrix(dR).as_rotvec()
        res.extend((rot_weight * rotvec).tolist())
        res.extend(dt.tolist())
    return np.asarray(res, dtype=np.float64)


def matrix_to_xyz_rpy_deg(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xyz = T[:3, 3].copy()
    rpy = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
    return xyz, rpy


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Solve eye-to-hand from ArUco-on-gripper samples using bTg * gTm = bTc * cTm."
    )
    parser.add_argument("--samples", default="aruco_handeye_samples.json")
    parser.add_argument("--out", default="aruco_handeye_result.json")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--camera-frame", default="cam_overhead")
    parser.add_argument("--rot-weight", type=float, default=0.10, help="Meter-equivalent weight for rotation residual.")
    args = parser.parse_args()

    samples_path = Path(args.samples).expanduser().resolve()
    if not samples_path.exists():
        raise FileNotFoundError(f"Samples file not found: {samples_path}")

    data = json.loads(samples_path.read_text(encoding="utf-8"))
    samples = data.get("samples", [])
    if len(samples) < 6:
        raise RuntimeError(f"Need at least 6 samples, got {len(samples)}.")

    bTg_list: list[np.ndarray] = []
    cTm_list: list[np.ndarray] = []
    for s in samples:
        tool = s["tool_in_base"]
        marker = s["marker_in_camera"]
        bTg = pose_to_matrix(tool["position_xyz"], tool["quaternion_xyzw"])
        cTm = pose_to_matrix(marker["position_xyz"], marker["quaternion_xyzw"])
        bTg_list.append(bTg)
        cTm_list.append(cTm)

    # Initialize with X=I, Z from first sample.
    X0 = np.eye(4, dtype=np.float64)
    Z0 = bTg_list[0] @ np.linalg.inv(cTm_list[0])
    p0 = pack_params(X0[:3, :3], X0[:3, 3], Z0[:3, :3], Z0[:3, 3])

    opt = least_squares(
        residuals,
        p0,
        args=(bTg_list, cTm_list, args.rot_weight),
        method="trf",
        loss="huber",
        f_scale=0.01,
        max_nfev=5000,
    )

    Rx, tx, Rz, tz = unpack_params(opt.x)
    X = make_transform(Rx, tx)  # gTm
    Z = make_transform(Rz, tz)  # bTc

    # Residual report
    rot_err_deg = []
    trans_err_m = []
    z_stack = []
    for bTg, cTm in zip(bTg_list, cTm_list):
        lhs = bTg @ X
        rhs = Z @ cTm
        dT = np.linalg.inv(rhs) @ lhs
        rv = R.from_matrix(dT[:3, :3]).as_rotvec()
        rot_err_deg.append(float(np.linalg.norm(rv) * 180.0 / math.pi))
        trans_err_m.append(float(np.linalg.norm(dT[:3, 3])))
        z_i = bTg @ X @ np.linalg.inv(cTm)
        z_stack.append(z_i[:3, 3])
    z_stack_np = np.asarray(z_stack, dtype=np.float64)

    xyz, rpy_deg = matrix_to_xyz_rpy_deg(Z)
    quat_xyzw = R.from_matrix(Z[:3, :3]).as_quat()

    result = {
        "meta": {
            "tool": "solve_aruco_handeye.py",
            "samples_file": str(samples_path),
            "samples_count": len(samples),
            "status": int(opt.status),
            "message": str(opt.message),
            "cost": float(opt.cost),
            "success": bool(opt.success),
        },
        "base_to_camera": {
            "parent_frame": args.base_frame,
            "child_frame": args.camera_frame,
            "translation_xyz_m": xyz.tolist(),
            "quaternion_xyzw": quat_xyzw.tolist(),
            "rpy_xyz_deg": rpy_deg.tolist(),
            "matrix_4x4": Z.tolist(),
        },
        "gripper_to_marker": {
            "matrix_4x4": X.tolist(),
        },
        "fit_error": {
            "translation_rmse_m": float(np.sqrt(np.mean(np.square(trans_err_m)))),
            "translation_mean_m": float(np.mean(trans_err_m)),
            "translation_max_m": float(np.max(trans_err_m)),
            "rotation_mean_deg": float(np.mean(rot_err_deg)),
            "rotation_max_deg": float(np.max(rot_err_deg)),
            "z_repeatability_std_m_xyz": z_stack_np.std(axis=0).tolist(),
        },
    }

    out_path = Path(args.out).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

    print("=== Hand-Eye Solve Result ===")
    print(f"samples: {len(samples)}")
    print(f"success: {opt.success}  status={opt.status}  cost={opt.cost:.6f}")
    print(f"base->camera xyz (m): {xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}")
    print(f"base->camera rpy (deg): {rpy_deg[0]:.3f}, {rpy_deg[1]:.3f}, {rpy_deg[2]:.3f}")
    print(
        "base->camera quat (xyzw): "
        f"{quat_xyzw[0]:.6f}, {quat_xyzw[1]:.6f}, {quat_xyzw[2]:.6f}, {quat_xyzw[3]:.6f}"
    )
    print(
        "fit err: "
        f"trans_rmse={result['fit_error']['translation_rmse_m']:.6f} m, "
        f"rot_mean={result['fit_error']['rotation_mean_deg']:.3f} deg"
    )
    print(f"saved: {out_path}")


if __name__ == "__main__":
    main()
