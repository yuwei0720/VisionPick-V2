"""Pure geometry and robust-depth helpers for YOLO OBB localization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


@dataclass(frozen=True)
class DepthEstimate:
    depth_m: float | None
    valid_ratio: float
    valid_pixels: int
    roi_pixels: int
    mad_m: float | None
    status: str

    @property
    def ok(self) -> bool:
        return self.depth_m is not None and self.status == "ok"


def obb_center_px(corners: np.ndarray) -> tuple[int, int]:
    points = _validated_corners(corners)
    center = points.mean(axis=0)
    return int(round(float(center[0]))), int(round(float(center[1])))


def shrink_obb(corners: np.ndarray, scale: float) -> np.ndarray:
    points = _validated_corners(corners)
    if not 0.0 < scale <= 1.0:
        raise ValueError("roi_scale must be in (0, 1]")
    center = points.mean(axis=0)
    return center + (points - center) * scale


def estimate_robust_depth(
    depth_image_m: np.ndarray,
    corners: np.ndarray,
    *,
    min_m: float,
    max_m: float,
    roi_scale: float,
    min_valid_pixels: int,
    min_valid_ratio: float,
    mad_multiplier: float,
) -> DepthEstimate:
    """Estimate object depth from a shrunken OBB polygon."""
    depth = np.asarray(depth_image_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth_image_m must be a 2D array")
    if not 0.0 < min_m < max_m:
        raise ValueError("depth range must satisfy 0 < min_m < max_m")
    if min_valid_pixels < 1:
        raise ValueError("min_valid_pixels must be >= 1")
    if not 0.0 <= min_valid_ratio <= 1.0:
        raise ValueError("min_valid_ratio must be in [0, 1]")
    if mad_multiplier <= 0.0:
        raise ValueError("mad_multiplier must be > 0")

    polygon = shrink_obb(corners, roi_scale)
    mask = _convex_polygon_mask(depth.shape, polygon)
    roi_pixels = int(np.count_nonzero(mask))
    if roi_pixels == 0:
        return DepthEstimate(None, 0.0, 0, 0, None, "empty_roi")

    values = depth[mask.astype(bool)]
    valid = np.isfinite(values) & (values >= min_m) & (values <= max_m)
    values = values[valid]
    raw_valid_ratio = float(values.size / roi_pixels)

    if values.size < min_valid_pixels:
        return DepthEstimate(
            None,
            raw_valid_ratio,
            int(values.size),
            roi_pixels,
            None,
            "insufficient_valid_pixels",
        )
    if raw_valid_ratio < min_valid_ratio:
        return DepthEstimate(
            None,
            raw_valid_ratio,
            int(values.size),
            roi_pixels,
            None,
            "insufficient_valid_ratio",
        )

    median = float(np.median(values))
    absolute_deviation = np.abs(values - median)
    mad = float(np.median(absolute_deviation))
    if mad > np.finfo(np.float32).eps:
        robust_sigma = 1.4826 * mad
        values = values[absolute_deviation <= mad_multiplier * robust_sigma]
    else:
        values = values[np.isclose(absolute_deviation, 0.0, atol=1e-6)]

    filtered_ratio = float(values.size / roi_pixels)
    if values.size < min_valid_pixels:
        return DepthEstimate(
            None,
            filtered_ratio,
            int(values.size),
            roi_pixels,
            mad,
            "insufficient_inlier_pixels",
        )
    if filtered_ratio < min_valid_ratio:
        return DepthEstimate(
            None,
            filtered_ratio,
            int(values.size),
            roi_pixels,
            mad,
            "insufficient_inlier_ratio",
        )

    return DepthEstimate(
        float(np.median(values)),
        filtered_ratio,
        int(values.size),
        roi_pixels,
        mad,
        "ok",
    )


def obb_long_axis_angle_deg(corners: np.ndarray) -> float:
    """Return the OBB long-axis image angle in [-90, 90) degrees."""
    points = _validated_corners(corners)
    edges = np.roll(points, -1, axis=0) - points
    lengths_sq = np.einsum("ij,ij->i", edges, edges)
    edge = edges[int(np.argmax(lengths_sq))]
    if float(np.dot(edge, edge)) <= np.finfo(np.float32).eps:
        raise ValueError("OBB has zero-length edges")
    angle = float(np.degrees(np.arctan2(edge[1], edge[0])))
    return (angle + 90.0) % 180.0 - 90.0


def deproject_pixel_to_camera(
    pixel: tuple[int, int],
    depth_m: float,
    intrinsics: Any,
    deproject_fn: Callable[[Any, list[float], float], list[float]],
) -> tuple[float, float, float]:
    """Deproject an aligned-depth pixel using an injectable RealSense API."""
    if not np.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError("depth_m must be finite and > 0")
    point = np.asarray(
        deproject_fn(
            intrinsics,
            [float(pixel[0]), float(pixel[1])],
            float(depth_m),
        ),
        dtype=float,
    )
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("deprojection must return three finite coordinates")
    return float(point[0]), float(point[1]), float(point[2])


def _validated_corners(corners: np.ndarray) -> np.ndarray:
    points = np.asarray(corners, dtype=np.float32)
    if points.shape != (4, 2):
        raise ValueError("corners must have shape (4, 2)")
    if not np.all(np.isfinite(points)):
        raise ValueError("corners must contain finite values")
    return points


def _convex_polygon_mask(
    image_shape: tuple[int, int],
    polygon: np.ndarray,
) -> np.ndarray:
    """Rasterize an ordered convex polygon using pixel-center half planes."""
    height, width = image_shape
    mask = np.zeros((height, width), dtype=bool)
    if height == 0 or width == 0:
        return mask

    min_x = max(0, int(np.floor(float(np.min(polygon[:, 0])))))
    max_x = min(width - 1, int(np.ceil(float(np.max(polygon[:, 0])))))
    min_y = max(0, int(np.floor(float(np.min(polygon[:, 1])))))
    max_y = min(height - 1, int(np.ceil(float(np.max(polygon[:, 1])))))
    if min_x > max_x or min_y > max_y:
        return mask

    yy, xx = np.mgrid[min_y : max_y + 1, min_x : max_x + 1]
    points = np.stack((xx, yy), axis=-1).astype(np.float32)
    edges = np.roll(polygon, -1, axis=0) - polygon
    relative = points[..., None, :] - polygon[None, None, :, :]
    cross = (
        edges[None, None, :, 0] * relative[..., 1]
        - edges[None, None, :, 1] * relative[..., 0]
    )
    epsilon = 1e-5
    inside = np.all(cross >= -epsilon, axis=-1) | np.all(
        cross <= epsilon,
        axis=-1,
    )
    mask[min_y : max_y + 1, min_x : max_x + 1] = inside
    return mask
