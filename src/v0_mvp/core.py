from __future__ import annotations
import numpy as np
from .geometry import robust_depth_from_obb, transform_point, camera_axis_to_robot_yaw_deg
from .grasp import build_top_down_plan
import logging

logger = logging.getLogger(__name__)

def solve_detection_to_grasp(det, depth_m, deproject_fn, T_robot_from_camera, cfg):
    dcfg = cfg["depth"]
    depth = robust_depth_from_obb(
        depth_m,
        det.corners_px,
        min_m=float(dcfg["min_m"]),
        max_m=float(dcfg["max_m"]),
        inner_scale=float(dcfg.get("inner_obb_scale", 0.55)),
        min_valid_pixels=int(dcfg.get("min_valid_pixels", 20)),
        method=str(dcfg.get("method", "median")),
        percentile=float(dcfg.get("percentile", 35.0)),
        fallback_roi_size=int(dcfg.get("fallback_roi_size", 11)),
    )
    u, v = det.center_uv
    point_cam = np.asarray(deproject_fn(u, v, depth.depth_m), dtype=float)
    point_robot = transform_point(T_robot_from_camera, point_cam)

    logger.debug("depth_m=%s", depth.depth_m)
    logger.debug("point_cam_m=%s", point_cam)
    logger.debug("point_robot_m=%s", point_robot)

    ocfg = cfg["orientation"]
    half = float(ocfg.get("axis_sample_half_length_px", 20.0))
    theta = np.deg2rad(det.long_axis_angle_deg)
    du, dv = float(np.cos(theta) * half), float(np.sin(theta) * half)
    a_cam = np.asarray(deproject_fn(u - du, v - dv, depth.depth_m), dtype=float)
    b_cam = np.asarray(deproject_fn(u + du, v + dv, depth.depth_m), dtype=float)
    yaw = camera_axis_to_robot_yaw_deg(
        a_cam, b_cam, T_robot_from_camera,
        gripper_yaw_offset_deg=float(ocfg.get("gripper_yaw_offset_deg", 0.0)),
        reference_yaw_deg=float(ocfg.get("reference_yaw_deg", 0.0)),
    )
    plan = build_top_down_plan(point_robot, yaw, cfg["grasp"], canonical_target=det.class_name)
    return depth, point_cam, point_robot, yaw, plan
