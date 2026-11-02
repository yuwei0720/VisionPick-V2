from __future__ import annotations
from dataclasses import dataclass
from typing import Any
import numpy as np


@dataclass(frozen=True)
class FrameBundle:
    color_bgr: np.ndarray
    depth_m: np.ndarray
    intrinsics: Any


@dataclass(frozen=True)
class OBBDetection:
    class_id: int
    class_name: str
    confidence: float
    corners_px: np.ndarray
    center_uv: tuple[float, float]
    long_axis_angle_deg: float


@dataclass(frozen=True)
class DepthEstimate:
    depth_m: float
    valid_pixels: int
    mad_m: float
    method: str


@dataclass(frozen=True)
class GraspPlan:
    pregrasp_pose_mm_deg: tuple[float, float, float, float, float, float]
    grasp_pose_mm_deg: tuple[float, float, float, float, float, float]
    lift_pose_mm_deg: tuple[float, float, float, float, float, float]

    def as_dict(self) -> dict:
        return {
            "pregrasp_pose_mm_deg": [round(float(v), 3) for v in self.pregrasp_pose_mm_deg],
            "grasp_pose_mm_deg": [round(float(v), 3) for v in self.grasp_pose_mm_deg],
            "lift_pose_mm_deg": [round(float(v), 3) for v in self.lift_pose_mm_deg],
        }
