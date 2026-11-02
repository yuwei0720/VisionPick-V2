from __future__ import annotations
import math
import numpy as np
from .types import GraspPlan


class GraspError(RuntimeError):
    pass


def _check_workspace(pose, workspace):
    labels = ("x", "y", "z")
    for i, key in enumerate(labels):
        lo, hi = map(float, workspace[key])
        value = float(pose[i])
        if not (lo <= value <= hi):
            raise GraspError(f"{key.upper()}={value:.2f} mm 超出 workspace [{lo}, {hi}]")


def build_top_down_plan(point_robot_m, yaw_deg: float, cfg: dict, *, canonical_target=None, table_z_mm=None) -> GraspPlan:
    offsets = cfg.get("target_z_offsets_mm", {})
    if not isinstance(offsets, dict):
        raise GraspError("target_z_offsets_mm must be a dictionary")
    def finite_mm(value, name):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise GraspError(f"{name} must be finite millimeters")
        return float(value)
    for name, value in offsets.items():
        if not isinstance(name, str):
            raise GraspError("target_z_offsets_mm keys must be canonical target names")
        finite_mm(value, f"target_z_offsets_mm.{name}")
    xyz = np.asarray(point_robot_m, dtype=float).reshape(3) * 1000.0
    xy_offset = np.asarray(cfg.get("tcp_xy_offset_mm", [0, 0]), dtype=float).reshape(2)
    xyz[:2] += xy_offset
    xyz[2] += (finite_mm(cfg.get("grasp_z_offset_mm", 0.0), "grasp_z_offset_mm")
               + offsets.get(canonical_target, 0.0))
    # Clamp TCP Z only; no unmeasured finger geometry is inferred here.
    floors = [cfg.get("min_grasp_z_mm"), cfg.get("table_z_mm"), table_z_mm]
    for floor in floors:
        if floor is not None:
            xyz[2] = max(xyz[2], finite_mm(floor, "grasp_z_floor"))
    rx = float(cfg.get("rx_deg", -175.0))
    ry = float(cfg.get("ry_deg", -5.0))
    approach = float(cfg.get("approach_clearance_mm", 80.0))
    lift = float(cfg.get("lift_clearance_mm", 100.0))
    grasp = (xyz[0], xyz[1], xyz[2], rx, ry, float(yaw_deg))
    pre = (xyz[0], xyz[1], xyz[2] + approach, rx, ry, float(yaw_deg))
    up = (xyz[0], xyz[1], xyz[2] + lift, rx, ry, float(yaw_deg))
    workspace = cfg["workspace_mm"]
    for pose in (pre, grasp, up):
        _check_workspace(pose, workspace)
    return GraspPlan(pre, grasp, up)
