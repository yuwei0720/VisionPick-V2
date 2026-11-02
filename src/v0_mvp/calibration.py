from __future__ import annotations
import csv
from pathlib import Path
import numpy as np
from .geometry import as_transform


class CalibrationError(RuntimeError):
    pass


def solve_rigid_transform(camera_points_m: np.ndarray, robot_points_m: np.ndarray):
    A = np.asarray(camera_points_m, dtype=float)
    B = np.asarray(robot_points_m, dtype=float)
    if A.shape != B.shape or A.ndim != 2 or A.shape[1] != 3:
        raise CalibrationError(f"點集需同為 Nx3，收到 {A.shape} / {B.shape}")
    if not np.isfinite(A).all() or not np.isfinite(B).all():
        raise CalibrationError("校正點必須都是有限數值")
    if len(A) < 3:
        raise CalibrationError("至少需要 3 組對應點")
    centered = A - A.mean(axis=0)
    if np.linalg.matrix_rank(centered) < 2 or np.linalg.matrix_rank(B - B.mean(axis=0)) < 2:
        raise CalibrationError("校正點退化：至少需要非共線的 3D 對應點")
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = cb - R @ ca
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    as_transform(T)
    pred = (R @ A.T).T + t
    err_m = np.linalg.norm(pred - B, axis=1)
    return T, float(np.sqrt(np.mean(err_m ** 2)) * 1000.0), err_m * 1000.0


def load_point_csv(path: str | Path):
    cam, rob = [], []
    with Path(path).open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        required = {"camera_x_m", "camera_y_m", "camera_z_m", "robot_x_mm", "robot_y_mm", "robot_z_mm"}
        if not required.issubset(set(reader.fieldnames or [])):
            raise CalibrationError(f"CSV 欄位必須包含: {sorted(required)}")
        for row in reader:
            cam.append([float(row["camera_x_m"]), float(row["camera_y_m"]), float(row["camera_z_m"])])
            rob.append([float(row["robot_x_mm"]) / 1000.0, float(row["robot_y_mm"]) / 1000.0, float(row["robot_z_mm"]) / 1000.0])
    return np.asarray(cam, dtype=float), np.asarray(rob, dtype=float)
