from __future__ import annotations
import math
import cv2
import numpy as np
from .types import DepthEstimate


class GeometryError(RuntimeError):
    pass


def obb_center(corners: np.ndarray) -> tuple[float, float]:
    pts = np.asarray(corners, dtype=float).reshape(4, 2)
    c = pts.mean(axis=0)
    return float(c[0]), float(c[1])


def _longest_edge_vector(corners: np.ndarray) -> np.ndarray:
    pts = np.asarray(corners, dtype=float).reshape(4, 2)
    edges = np.roll(pts, -1, axis=0) - pts
    lengths = np.linalg.norm(edges, axis=1)
    if not np.all(np.isfinite(lengths)) or float(lengths.max()) < 1e-6:
        raise GeometryError("退化的 OBB corners")
    v = edges[int(np.argmax(lengths))]
    return v


def long_axis_angle_deg(corners: np.ndarray) -> float:
    v = _longest_edge_vector(corners)
    angle = math.degrees(math.atan2(float(v[1]), float(v[0])))
    # OBB 長軸無方向性，規範到 [-90, 90)。
    return ((angle + 90.0) % 180.0) - 90.0


def scale_polygon(corners: np.ndarray, scale: float) -> np.ndarray:
    pts = np.asarray(corners, dtype=float).reshape(4, 2)
    if not 0 < scale <= 1:
        raise ValueError("scale 必須在 (0, 1]")
    c = pts.mean(axis=0)
    return c + (pts - c) * float(scale)


def _valid_depth_values(depth_m: np.ndarray, mask: np.ndarray, min_m: float, max_m: float) -> np.ndarray:
    vals = depth_m[mask > 0].astype(float)
    return vals[np.isfinite(vals) & (vals >= min_m) & (vals <= max_m)]


def robust_depth_from_obb(
    depth_m: np.ndarray,
    corners: np.ndarray,
    *,
    min_m: float,
    max_m: float,
    inner_scale: float = 0.55,
    min_valid_pixels: int = 20,
    method: str = "median",
    percentile: float = 35.0,
    fallback_roi_size: int = 11,
) -> DepthEstimate:
    depth = np.asarray(depth_m, dtype=float)
    if depth.ndim != 2:
        raise GeometryError(f"depth 必須為 HxW，收到 {depth.shape}")
    h, w = depth.shape
    inner = scale_polygon(corners, inner_scale)
    poly = np.rint(inner).astype(np.int32)
    poly[:, 0] = np.clip(poly[:, 0], 0, w - 1)
    poly[:, 1] = np.clip(poly[:, 1], 0, h - 1)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillConvexPoly(mask, poly, 1)
    vals = _valid_depth_values(depth, mask, min_m, max_m)

    used_method = f"obb-{method}"
    if vals.size < int(min_valid_pixels):
        u, v = obb_center(corners)
        r = max(1, int(fallback_roi_size) // 2)
        x0, x1 = max(0, int(round(u)) - r), min(w, int(round(u)) + r + 1)
        y0, y1 = max(0, int(round(v)) - r), min(h, int(round(v)) + r + 1)
        roi = depth[y0:y1, x0:x1]
        vals = roi[np.isfinite(roi) & (roi >= min_m) & (roi <= max_m)].astype(float)
        used_method = f"center-roi-{method}"

    if vals.size < int(min_valid_pixels):
        raise GeometryError(f"有效 depth pixel 不足: {vals.size} < {min_valid_pixels}")

    method = str(method).lower()
    if method == "median":
        z = float(np.median(vals))
    elif method == "percentile":
        z = float(np.percentile(vals, float(percentile)))
    else:
        raise ValueError("depth.method 只支援 median 或 percentile")
    med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - med)))
    return DepthEstimate(depth_m=z, valid_pixels=int(vals.size), mad_m=mad, method=used_method)


def as_transform(T) -> np.ndarray:
    arr = np.asarray(T, dtype=float)
    if arr.shape != (4, 4):
        raise GeometryError(f"Rigid transform 必須為 4x4，收到 {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise GeometryError("Rigid transform 含 NaN/Inf")
    if not np.allclose(arr[3], [0, 0, 0, 1], atol=1e-8):
        raise GeometryError("Rigid transform 最後一列必須為 [0,0,0,1]")
    R = arr[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-3) or np.linalg.det(R) < 0.99:
        raise GeometryError("T 的 rotation block 不是有效 proper rotation")
    return arr


def transform_point(T, point_xyz_m) -> np.ndarray:
    T = as_transform(T)
    p = np.asarray(point_xyz_m, dtype=float).reshape(3)
    return (T @ np.r_[p, 1.0])[:3]


def choose_equivalent_yaw_deg(yaw_deg: float, reference_deg: float) -> float:
    # OBB 軸向 yaw 與 yaw+180 等價；挑離 reference 最近者。
    candidates = [yaw_deg + 180.0 * k for k in range(-2, 3)]
    return float(min(candidates, key=lambda x: abs(x - reference_deg)))


def camera_axis_to_robot_yaw_deg(
    point_a_cam_m,
    point_b_cam_m,
    T_robot_from_camera,
    *,
    gripper_yaw_offset_deg: float = 0.0,
    reference_yaw_deg: float = 0.0,
) -> float:
    a = transform_point(T_robot_from_camera, point_a_cam_m)
    b = transform_point(T_robot_from_camera, point_b_cam_m)
    d = b[:2] - a[:2]
    if np.linalg.norm(d) < 1e-9:
        raise GeometryError("OBB 長軸轉到 Robot XY 後退化")
    yaw = math.degrees(math.atan2(float(d[1]), float(d[0]))) + float(gripper_yaw_offset_deg)
    return choose_equivalent_yaw_deg(yaw, float(reference_yaw_deg))
