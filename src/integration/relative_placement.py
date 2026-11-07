"""S3 static RGB-D geometry preview. This module never authorizes motion."""
from __future__ import annotations

import hashlib
import json
import math
import time

import cv2
import numpy as np
from object_catalog import graspable_classes

from .scene_grounding import SceneGroundingError, normalize_rotation_deg
from v0_mvp.geometry import as_transform, camera_axis_to_robot_yaw_deg, robust_depth_from_obb
from v0_mvp.grasp import build_top_down_plan


def _reject(reason, **details):
    raise SceneGroundingError(reason, reason, details=details)


def _number(value, name, *, minimum=None):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or (minimum is not None and value < minimum)):
        _reject("invalid_relative_place_settings", field=name)
    return float(value)


def _bounds(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        _reject("invalid_relative_place_settings", field=name)
    lo, hi = (_number(v, name) for v in value)
    if lo >= hi:
        _reject("invalid_relative_place_settings", field=name)
    return lo, hi


def _calibration_id(transform):
    return hashlib.sha256(json.dumps(transform.tolist()).encode()).hexdigest()


ORIENTATION_POLICIES = {
    'relative': ('align_reference', 'keep_target', 'fixed_deg'),
    'self_relative': ('keep_target', 'fixed_deg'),
    'swap': ('match_initial', 'keep_target', 'fixed_deg'),
    'named': ('profile_pose', 'keep_target'),
}


def resolve_place_orientation(config, scenario, target_yaw, *, reference_yaw=None,
                              initial_yaw=None, profile_yaw=None, override=None):
    """Resolve tool RZ before rotating footprints or evaluating clearance."""
    orientation = config.get('orientation', {})
    allowed = ORIENTATION_POLICIES[scenario]
    policy = orientation.get('place_orientation_policy_' + scenario, allowed[0])
    if policy not in allowed:
        _reject('invalid_place_orientation_policy', scenario=scenario, policy=policy)
    fixed = orientation.get('fixed_place_yaw_deg', -45.0)
    if override is not None:
        if not isinstance(override, dict) or set(override) - {'policy','rz_deg'}:
            _reject('invalid_place_orientation_override')
        selected = override.get('policy', 'default')
        if selected != 'default':
            if selected not in set(allowed) | {'fixed_deg'}:
                _reject('invalid_place_orientation_override')
            policy = selected
        if selected == 'fixed_deg':
            fixed = override.get('rz_deg')
    value = {'keep_target':target_yaw, 'align_reference':reference_yaw,
             'match_initial':initial_yaw, 'profile_pose':profile_yaw, 'fixed_deg':fixed}[policy]
    if type(value) not in (int,float) or not math.isfinite(value) or not -180 <= value <= 180:
        _reject('invalid_place_orientation_angle', policy=policy)
    return policy, float(value)


def footprint_yaw_deg(polygon, config):
    """Derive a snapshot tool yaw from its measured base-frame long axis."""
    from v0_mvp.geometry import choose_equivalent_yaw_deg
    polygon = np.asarray(polygon, dtype=float)
    edges = np.roll(polygon, -1, axis=0) - polygon
    edge = edges[np.argmax(np.linalg.norm(edges, axis=1))]
    orientation = config.get('orientation', {})
    return choose_equivalent_yaw_deg(math.degrees(math.atan2(edge[1],edge[0]))
        + orientation.get('gripper_yaw_offset_deg',0), orientation.get('reference_yaw_deg',0))


def project_robot_points_mm(points, config, intrinsics):
    """Base millimetres -> inverse extrinsic -> pinhole pixels (unrotated RGB)."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('invalid_preview_points')
    transform = np.linalg.inv(as_transform(config['calibration']['T_robot_from_camera']))
    camera = np.column_stack((points / 1000., np.ones(len(points)))) @ transform.T
    if np.any(camera[:,2] <= 1e-9):
        raise ValueError('preview_points_behind_camera')
    def intrinsic(name, alias=None):
        value = intrinsics.get(name, intrinsics.get(alias)) if isinstance(intrinsics,dict) else getattr(intrinsics,name,getattr(intrinsics,alias,None) if alias else None)
        if type(value) not in (int,float) or not math.isfinite(value):
            raise ValueError('invalid_preview_intrinsics')
        return value
    fx,fy,cx,cy = intrinsic('fx'),intrinsic('fy'),intrinsic('cx','ppx'),intrinsic('cy','ppy')
    if fx <= 0 or fy <= 0: raise ValueError('invalid_preview_intrinsics')
    return np.column_stack((camera[:,0]*fx/camera[:,2]+cx,camera[:,1]*fy/camera[:,2]+cy)).tolist()


def placement_preview_geometry(geometry, config, intrinsics):
    """Normalize a successful geometry or last failed candidate for the canvas."""
    poly = geometry.get('destination_footprint_robot_xy_mm', geometry.get('candidate_footprint_robot_xy_mm'))
    center = geometry.get('destination_center_robot_xy_mm', geometry.get('candidate_center_robot_xy_mm'))
    envelope = geometry.get('clearance_envelope_robot_xy_mm')
    if poly is None or center is None or envelope is None:
        return {}
    z = float(geometry.get('landing_surface_z_mm', geometry.get('table_z_mm',0)))
    poly = np.asarray(poly,dtype=float)
    center = np.asarray(center,dtype=float)
    edges = np.roll(poly,-1,axis=0)-poly
    axis = edges[np.argmax(np.linalg.norm(edges,axis=1))] / 2
    axis3 = np.r_[axis,0.]
    center3 = np.r_[center,z]
    polygon3 = np.column_stack((poly,np.full(4,z)))
    envelope3 = np.column_stack((envelope,np.full(4,z)))
    result = {'polygon_mm':polygon3.tolist(), 'center_mm':center3.tolist(),
        'envelope_mm':envelope3.tolist(), 'long_axis_vector_mm':axis3.tolist(),
        'rz_deg':geometry.get('place_yaw_deg'), 'orientation_policy':geometry.get('orientation_policy')}
    result.update({key:geometry[key] for key in
        ('center_distance_mm','projected_gap_mm','nearest_edge_gap_mm') if key in geometry})
    try:
        result.update(polygon_px=project_robot_points_mm(polygon3,config,intrinsics),
            center_px=project_robot_points_mm([center3],config,intrinsics)[0],
            envelope_px=project_robot_points_mm(envelope3,config,intrinsics),
            direction_px=project_robot_points_mm([center3,center3+axis3],config,intrinsics))
    except ValueError as exc:
        result['projection_reason'] = str(exc)
    return result


def _unique(detections, name, selected_index=None):
    if selected_index is not None:
        if (type(selected_index) is not int or not 0 <= selected_index < len(detections)
                or detections[selected_index].class_name != name):
            _reject("selected_instance_mismatch", object=name)
        return selected_index
    matches = [i for i, det in enumerate(detections) if det.class_name == name]
    if len(matches) != 1:
        _reject("object_not_detected" if not matches else "ambiguous_object_instances", object=name)
    return matches[0]


def _footprint(det, frame, deproject, transform, config, surface_tolerance_mm):
    corners = np.asarray(det.corners_px, dtype=float)
    h, w = frame.depth_m.shape
    if (corners.shape != (4, 2) or not np.isfinite(corners).all()
            or not np.isfinite(det.center_uv).all()
            or np.any(corners < 0) or np.any(corners[:, 0] >= w) or np.any(corners[:, 1] >= h)
            or not 0 <= det.center_uv[0] < w or not 0 <= det.center_uv[1] < h
            or not cv2.isContourConvex(corners.astype(np.float32))
            or abs(cv2.contourArea(corners.astype(np.float32))) < 1
            or not math.isfinite(det.confidence) or det.confidence < config["model"]["confidence"]):
        _reject("unreliable_detection_geometry", object=det.class_name)
    depth_cfg = config["depth"]
    depth_image = getattr(frame.intrinsics, "object_depths_m", {}).get(det.class_id, frame.depth_m)
    try:
        depth = robust_depth_from_obb(
            depth_image, corners, min_m=depth_cfg["min_m"], max_m=depth_cfg["max_m"],
            inner_scale=depth_cfg.get("inner_obb_scale", .55),
            min_valid_pixels=depth_cfg.get("min_valid_pixels", 20),
            method=depth_cfg.get("method", "median"),
            percentile=depth_cfg.get("percentile", 35),
            fallback_roi_size=depth_cfg.get("fallback_roi_size", 11),
        )
    except (ValueError, RuntimeError) as exc:
        _reject("unreliable_object_depth", object=det.class_name, detail=str(exc))
    if depth.mad_m * 1000 > surface_tolerance_mm:
        _reject("unreliable_object_depth", object=det.class_name)
    points = np.asarray([deproject(u, v, depth.depth_m) for u, v in corners], dtype=float)
    if points.shape != (4, 3) or not np.isfinite(points).all():
        _reject("unreliable_object_depth", object=det.class_name)
    robot_points = (points @ transform[:3, :3].T + transform[:3, 3]) * 1000
    footprint = robot_points[:, :2]
    if abs(cv2.contourArea(footprint.astype(np.float32))) < 1e-3:
        _reject("degenerate_table_footprint", object=det.class_name)
    return footprint, depth.depth_m


def _table_point(u, v, z_mm, deproject, transform):
    origin = transform[:3, 3] * 1000
    ray = transform[:3, :3] @ np.asarray(deproject(u, v, 1.0), dtype=float)
    if ray.shape != (3,) or not np.isfinite(ray).all() or abs(ray[2]) < 1e-8:
        _reject("table_projection_unavailable")
    distance = (z_mm - origin[2]) / ray[2]
    if distance <= 0:
        _reject("table_projection_unavailable")
    return (origin + distance * ray)[:2]


def _polygon_axes(polygon):
    edges = np.roll(polygon, -1, axis=0) - polygon
    lengths = np.linalg.norm(edges, axis=1)
    edges = edges[lengths > 1e-9] / lengths[lengths > 1e-9, None]
    return np.column_stack((-edges[:, 1], edges[:, 0]))


def convex_polygons_intersect(first, second):
    """SAT on convex footprints; touching counts as occupied clearance."""
    first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    axes = np.vstack((_polygon_axes(first), _polygon_axes(second)))
    a, b = first @ axes.T, second @ axes.T
    return bool(np.all((a.max(axis=0) >= b.min(axis=0) - 1e-9)
                       & (b.max(axis=0) >= a.min(axis=0) - 1e-9)))


def polygon_distance_mm(first, second):
    """Minimum Euclidean distance, including intersecting/contained polygons."""
    first, second = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    if convex_polygons_intersect(first, second):
        return 0.0
    distances = []
    for points, polygon in ((first, second), (second, first)):
        edges = np.roll(polygon, -1, axis=0) - polygon
        relative = points[:, None, :] - polygon[None, :, :]
        lengths2 = np.sum(edges * edges, axis=1)
        fraction = np.clip(np.sum(relative * edges, axis=2) / lengths2, 0., 1.)
        distances.append(np.linalg.norm(relative - fraction[:, :, None] * edges, axis=2).min())
    return float(min(distances))


def _center_advance_for_gap(reference, centered_target, direction, gap):
    """Find the outward ray's unique positive-distance root (mm tolerance <1e-7).

    Centered convex OBBs overlap at alpha=0. SAT gives the exit from that
    overlap; distance is nondecreasing along the remaining ray. Projection
    separation provides only an upper bracket, never the final placement.
    """
    axes = np.vstack((_polygon_axes(reference), _polygon_axes(centered_target)))
    along = axes @ direction
    axes = axes[np.abs(along) > 1e-10] * np.sign(along[np.abs(along) > 1e-10, None])
    along = axes @ direction
    low = max(0., float(np.min(((reference @ axes.T).max(axis=0)
                               - (centered_target @ axes.T).min(axis=0)) / along)))
    if gap == 0:
        return low
    high = max(low, float(np.max(reference @ direction)
                          - np.min(centered_target @ direction) + gap)) + 1e-7
    for _ in range(60):
        mid = (low + high) / 2
        if polygon_distance_mm(reference, centered_target + mid * direction) < gap:
            low = mid
        else:
            high = mid
        if high - low < 1e-7:
            break
    return high


def clearance_envelope_obb(polygon, padding_mm, margin_mm, tcp_xy_offset):
    """Expand in the landing OBB's own axes, enclosing both +/- TCP offsets.

    Projecting the actual robot-frame offset into each local axis retains a
    symmetric conservative compensation without an axis-aligned XY rectangle.
    A measured nonrectangular footprint is enclosed in the same long-axis OBB.
    """
    polygon = np.asarray(polygon, dtype=float)
    tcp = np.asarray(tcp_xy_offset, dtype=float)
    if tcp.shape != (2,) or not np.isfinite(tcp).all():
        _reject('invalid_relative_place_settings', field='tcp_xy_offset_mm')
    edges = np.roll(polygon, -1, axis=0) - polygon
    axis = edges[np.argmax(np.linalg.norm(edges, axis=1))]
    axis = axis / np.linalg.norm(axis)
    axes = np.array([axis, [-axis[1], axis[0]]])
    local = polygon @ axes.T
    expansion = padding_mm + margin_mm + np.abs(axes @ tcp)
    low, high = local.min(axis=0) - expansion, local.max(axis=0) + expansion
    return np.array([[low[0], low[1]], [high[0], low[1]],
                     [high[0], high[1]], [low[0], high[1]]]) @ axes


def _surface_clear(envelope, frame, deproject, project, transform, z_mm, tolerance_mm, depth_cfg, *, allow_lower=False,
                   exclude_footprint=None, exclude_top_z_mm=None, exclude_image_polygon=None, mask_diagnostics=None):
    """Check visible depth; a top placement may have lower empty surroundings.

    Planar landing still requires a flat measured surface. The reference top ROI
    is measured separately; top placement rejects higher intrusions and depth
    holes, without requiring the whole gripper envelope to be a support surface.
    """
    robot = np.column_stack((envelope / 1000, np.full(4, z_mm / 1000)))
    camera = (robot - transform[:3, 3]) @ transform[:3, :3]
    if np.any(camera[:, 2] <= 0):
        _reject("destination_not_visible", stage="surface_visibility", camera_z_m=camera[:, 2].tolist())
    uv = np.asarray([project(point) for point in camera], dtype=float)
    h, w = frame.depth_m.shape
    if (uv.shape != (4, 2) or not np.isfinite(uv).all() or np.any(uv < 0)
            or np.any(uv[:, 0] >= w) or np.any(uv[:, 1] >= h)):
        _reject("destination_not_visible", stage="surface_visibility", projected_uv=uv.tolist(), image_size_px=[w, h])
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.rint(uv).astype(np.int32), 1)
    rows, columns = np.nonzero(mask)
    if len(rows) < depth_cfg.get("min_valid_pixels", 20):
        _reject("destination_depth_unavailable", stage="surface_depth", checked_pixels=int(len(rows)),
                minimum_pixels=depth_cfg.get("min_valid_pixels", 20))
    values = frame.depth_m[rows, columns]
    valid = np.isfinite(values) & (values >= depth_cfg["min_m"]) & (values <= depth_cfg["max_m"])
    invalid_count = int(np.count_nonzero(~valid))
    max_invalid = int(len(rows) * 0.02)
    if invalid_count > max_invalid:
        _reject("destination_depth_unavailable", stage="surface_depth", checked_pixels=int(len(rows)),
                invalid_pixels=invalid_count, allowed_invalid_pixels=max_invalid)
    valid_rows, valid_columns = rows[valid], columns[valid]
    if len(valid_rows) < depth_cfg.get("min_valid_pixels", 20):
        _reject("destination_depth_unavailable", stage="surface_depth", checked_pixels=int(len(rows)),
                valid_pixels=int(len(valid_rows)), minimum_pixels=depth_cfg.get("min_valid_pixels", 20))
    points = np.asarray([deproject(float(u), float(v), float(z))
                         for u, v, z in zip(valid_columns, valid_rows, values[valid])], dtype=float)
    if points.shape != (len(valid_rows), 3) or not np.isfinite(points).all():
        _reject("destination_depth_unavailable")
    robot_points = (points @ transform[:3, :3].T + transform[:3, 3]) * 1000
    heights = robot_points[:, 2]
    if exclude_footprint is not None:
        polygon = np.asarray(exclude_footprint, dtype=np.float32)
        if polygon.shape != (4, 2) or not np.isfinite(polygon).all() or not math.isfinite(exclude_top_z_mm):
            _reject('invalid_self_exclusion_mask')
        # Remove only measured points inside the selected target's original
        # footprint AND at its measured height. No depth holes are filled; all
        # other objects and unclassified intrusions remain in the surface check.
        own = np.abs(heights-exclude_top_z_mm) <= tolerance_mm
        if exclude_image_polygon is not None:
            image_polygon = np.asarray(exclude_image_polygon, dtype=float)
            if image_polygon.shape != (4,2) or not np.isfinite(image_polygon).all():
                _reject('invalid_self_exclusion_mask')
            original_mask = np.zeros((h,w),dtype=np.uint8)
            # Use the original observation's raster footprint. Testing world XY
            # against a floating OBB alone leaves one-pixel raster edge ghosts.
            cv2.fillConvexPoly(original_mask,image_polygon.astype(np.int32),1)
            own &= original_mask[valid_rows,valid_columns].astype(bool)
        else:
            indices = np.flatnonzero(own)
            own[indices] = [cv2.pointPolygonTest(polygon, (float(robot_points[i,0]),float(robot_points[i,1])), False) >= 0 for i in indices]
        if mask_diagnostics is not None:
            mask_diagnostics['excluded_self_pixels'] = int(own.sum())
        heights = heights[~own]
        if len(heights) < depth_cfg.get('min_valid_pixels', 20):
            _reject('destination_depth_unavailable', stage='surface_after_self_exclusion', checked_pixels=int(len(heights)))
    deviations = heights - z_mm
    violations = deviations > tolerance_mm if allow_lower else np.abs(deviations) > tolerance_mm
    if np.any(violations):
        _reject("destination_surface_not_clear", stage="surface_height", table_z_mm=float(z_mm),
                observed_height_range_mm=[float(heights.min()), float(heights.max())],
                maximum_height_error_mm=float(np.max(deviations if allow_lower else np.abs(deviations))), tolerance_mm=float(tolerance_mm),
                violating_pixels=int(np.count_nonzero(violations)))
    return int(len(heights)), float(np.median(heights))


def plan_relative_place(task, detections, frame, *, deproject, project, config, surface,
                        gap_mm, captured_at_unix_ms, now_unix_ms=None,
                        selected_target_index=None, selected_reference_index=None, exact_gap=False,
                         initial_reference=None, self_displacement_mm=None, orientation_override=None,
                          preview_scene=None, reserved_regions=None, locked_destination=None):
    """Produce a read-only planar or modelled support candidate, preserving or aligning orientation.

    surface is an explicitly measured/modelled horizontal robot-frame table.
    OBB footprints and an expanded rectangular gripper envelope are conservative
    approximations, not a 3D collision model or an IK/path validation result.
    """
    if task.get("needs_clarification"):
        _reject("semantic_clarification_required")
    relation = task.get('spatial_relation')
    is_self = task.get('destination_type') == 'self_relative'
    if is_self:
        if (relation not in {'left_of','right_of','in_front_of','behind'} or task.get('reference_object') is not None
                or gap_mm is not None or selected_reference_index is not None or initial_reference is not None):
            _reject('invalid_self_relative_contract')
        displacement = _number(self_displacement_mm, 'displacement_mm', minimum=0)
        if displacement <= 0:
            _reject('invalid_self_relative_contract')
    elif self_displacement_mm is not None:
        _reject('displacement_not_applicable')
    top = relation == 'on_top_of'
    is_near = relation == 'near'
    is_initial = relation == 'at_initial'
    reservations = []
    for region in reserved_regions or []:
        polygon = np.asarray(region.get('polygon_mm'), dtype=float)
        if (not is_near or polygon.shape != (4, 2) or not np.isfinite(polygon).all()
                or not cv2.isContourConvex(polygon.astype(np.float32))):
            _reject('invalid_swap_reservation')
        reservations.append((region, polygon))
    if task.get("intent") != "place" or relation not in {"left_of", "right_of", 'in_front_of', 'behind', 'above', 'below', 'on_top_of', 'near', 'at_initial'}:
        _reject("relative_relation_not_supported")
    target, reference = task.get("target_object"), task.get("reference_object")
    if target not in graspable_classes() or (not is_self and reference in {None, 'unknown'}):
        _reject("relative_objects_not_supported")
    if target == reference and not is_initial and (selected_target_index is None or selected_reference_index is None):
        _reject("relative_objects_not_supported")
    if gap_mm is None and not (top or is_near or is_initial or is_self):
        _reject("missing_edge_gap", clarification_question="物品邊緣至少相隔幾公分？10秒未回覆採用5公分。",
                timeout_ms=10000, default_gap_mm=50)
    gap = None if top else (_number(gap_mm, "gap_mm", minimum=0) if gap_mm is not None else None)
    z_mm = _number(surface["z_mm"], "surface.z_mm")
    base_table_z_mm = z_mm
    margin = _number(surface["error_margin_mm"], "error_margin_mm", minimum=0)
    padding = _number(surface["gripper_padding_mm"], "gripper_padding_mm", minimum=0)
    tolerance = _number(surface["surface_tolerance_mm"], "surface_tolerance_mm", minimum=0)
    release = _number(surface["release_tcp_z_offset_mm"], "release_tcp_z_offset_mm", minimum=0)
    age_limit = _number(surface["max_age_ms"], "max_age_ms", minimum=1)
    if surface.get("static_scene") is not True:
        _reject("static_scene_required")
    now = time.time() * 1000 if now_unix_ms is None else _number(now_unix_ms, "now_unix_ms")
    captured = _number(captured_at_unix_ms, "captured_at_unix_ms")
    if not 0 <= now - captured <= age_limit:
        _reject("scene_snapshot_stale")
    table = {axis: _bounds(surface["bounds_mm"][axis], "surface." + axis) for axis in ("x", "y")}
    workspace = {axis: _bounds(config["grasp"]["workspace_mm"][axis], "workspace." + axis)
                 for axis in ("x", "y", "z")}
    if frame.depth_m.ndim != 2 or frame.color_bgr.shape[:2] != frame.depth_m.shape:
        _reject("invalid_rgbd_frame")
    transform = as_transform(config["calibration"]["T_robot_from_camera"])
    ti = _unique(detections, target, selected_target_index)
    ri = ti if is_self else None if is_initial else _unique(detections, reference, selected_reference_index)
    if ti == ri and not is_self:
        _reject("target_reference_same_instance")
    measurements = [_footprint(det, frame, deproject, transform, config, tolerance) for det in detections]
    if preview_scene is not None:
        measurements = preview_scene['measurements']
    target_poly, depth = measurements[ti]
    if is_initial:
        if not isinstance(initial_reference, dict) or initial_reference.get('class_name') != reference:
            _reject('initial_reference_snapshot_required')
        reference_poly = np.asarray(initial_reference.get('footprint_robot_xy_mm'), dtype=float)
        ref_point = np.asarray(initial_reference.get('xyz_mm'), dtype=float) / 1000.0
        if (reference_poly.shape != (4, 2) or ref_point.shape != (3,)
                or not np.isfinite(reference_poly).all() or not np.isfinite(ref_point).all()
                or not cv2.isContourConvex(reference_poly.astype(np.float32))):
            _reject('initial_reference_snapshot_invalid')
    else:
        reference_poly, ref_depth = measurements[ri]
        ref_det = detections[ri]
        ref_point = transform[:3, :3] @ np.asarray(deproject(*ref_det.center_uv, ref_depth)) + transform[:3, 3]
        if preview_scene is not None:
            ref_point = np.asarray(preview_scene['points_mm'][ri])/1000.
    ref_top_z_mm = round(float(ref_point[2] * 1000.0), 3)
    det = detections[ti]
    u, v = det.center_uv
    point = (transform[:3, :3] @ np.asarray(deproject(u, v, depth)) + transform[:3, 3])
    if preview_scene is not None:
        point = np.asarray(preview_scene['points_mm'][ti])/1000.
    target_z_mm = float(point[2] * 1000.0)
    for index, (polygon, obj_depth) in enumerate(measurements):
        if index == ti:
            continue
        obj_z_mm = float((transform[:3, :3] @ np.asarray(deproject(*detections[index].center_uv, obj_depth)) + transform[:3, 3])[2] * 1000.0)
        if (obj_z_mm > target_z_mm + tolerance
                and cv2.intersectConvexConvex(polygon.astype(np.float32), target_poly.astype(np.float32))[0] > 0):
            _reject("target_occluded_by_stacked_object", object=target, occluding_object=detections[index].class_name)

    if top:
        z_mm = ref_top_z_mm
        for index, (polygon, obj_depth) in enumerate(measurements):
            if index in {ti, ri}:
                continue
            obj_z_mm = float((transform[:3, :3] @ np.asarray(deproject(*detections[index].center_uv, obj_depth)) + transform[:3, 3])[2] * 1000.0)
            if (obj_z_mm >= ref_top_z_mm - tolerance
                    and cv2.intersectConvexConvex(polygon.astype(np.float32), reference_poly.astype(np.float32))[0] > 0):
                _reject("target_occluded_by_stacked_object", object=reference, occluding_object=detections[index].class_name)

    support = surface.get('support_surfaces', {}).get(reference) if isinstance(surface.get('support_surfaces'), dict) else None
    object_model = surface.get('object_models', {}).get(target) if isinstance(surface.get('object_models'), dict) else None

    h, w = frame.depth_m.shape
    rotation = normalize_rotation_deg(config.get("scene", {}).get("workspace_view_rotation_deg", 90))
    du, dv = {0: (1, 0), 90: (0, -1), 180: (-1, 0), 270: (0, 1)}[rotation]
    right = (_table_point(w / 2 + du, h / 2 + dv, z_mm, deproject, transform)
             - _table_point(w / 2, h / 2, z_mm, deproject, transform))
    if np.linalg.norm(right) < 1e-6:
        _reject("table_projection_unavailable")
    right /= np.linalg.norm(right)
    up = (_table_point(w / 2 + dv, h / 2 - du, z_mm, deproject, transform)
          - _table_point(w / 2, h / 2, z_mm, deproject, transform))
    if np.linalg.norm(up) < 1e-6:
        _reject('table_projection_unavailable')
    up /= np.linalg.norm(up)

    direction_map = {
        'right_of': right, 'right': right,
        'left_of': -right, 'left': -right,
        'above': up, 'top': up,
        'below': -up, 'bottom': -up,
        # Near/far are defined in the displayed workbench plane, not robot Z.
        'in_front_of': -up, 'behind': up,
        'on_top_of': right,
        'at_initial': right,
    }

    if is_near:
        near_cfg = config.get("relative_placement", {}).get("near", {})
        min_offset_mm = float(near_cfg.get("min_offset_mm", 30.0))
        max_offset_mm = float(near_cfg.get("max_offset_mm", 80.0))
        step_mm = float(near_cfg.get("step_mm", 10.0))
        cand_dir_names = list(near_cfg.get("candidate_directions", ["left", "right", "above", "below"]))
        if not cand_dir_names:
            _reject("no_valid_near_placement_candidate", detail="candidate_directions_empty")
        if min_offset_mm > max_offset_mm or step_mm <= 0:
            _reject("invalid_relative_place_settings", field="near_offset_range")
        if gap is not None:
            cand_gaps = [float(gap)]
            if not exact_gap:
                cand_gaps += [float(g) for g in np.arange(min_offset_mm, max_offset_mm + 1e-6, step_mm) if abs(g - gap) > 1e-4]
        else:
            cand_gaps = [float(g) for g in np.arange(min_offset_mm, max_offset_mm + 1e-6, step_mm)]
        if not cand_gaps:
            cand_gaps = [min_offset_mm]
        cand_specs = []
        for g_val in cand_gaps:
            for d_name in cand_dir_names:
                if d_name in direction_map:
                    cand_specs.append((direction_map[d_name], g_val))
        # Swap-only fallback: preserve the ordinary four-direction near priority,
        # then try diagonals and (when defaults are used) farther gaps. The
        # existing placement checks are applied to every candidate unchanged.
        # Explicit engineering direction/distance settings remain authoritative.
        if reservations:
            fallback_gaps = list(cand_gaps)
            if gap is None and 'max_offset_mm' not in near_cfg:
                fallback_max = float(near_cfg.get('swap_fallback_max_offset_mm', 120.0))
                fallback_gaps += [float(g) for g in np.arange(max_offset_mm + step_mm,
                    fallback_max + 1e-6, step_mm)]
            if 'candidate_directions' not in near_cfg:
                for g_val in fallback_gaps:
                    for first, second in ((right, up), (right, -up), (-right, up), (-right, -up)):
                        direction = first + second
                        direction /= np.linalg.norm(direction)
                        cand_specs.append((direction, g_val))
            for g_val in fallback_gaps[len(cand_gaps):]:
                for d_name in cand_dir_names:
                    if d_name in direction_map:
                        cand_specs.append((direction_map[d_name], g_val))
        if not cand_specs:
            _reject("no_valid_near_placement_candidate")
    else:
        direction = direction_map[relation]
        cand_specs = [(direction, 0.0 if top or is_initial or is_self else gap)]

    angle = math.radians(det.long_axis_angle_deg)
    if not math.isfinite(angle):
        _reject("unreliable_detection_geometry", object=target)
    half = config["orientation"].get("axis_sample_half_length_px", 20)
    yaw = camera_axis_to_robot_yaw_deg(
        deproject(u - math.cos(angle) * half, v - math.sin(angle) * half, depth),
        deproject(u + math.cos(angle) * half, v + math.sin(angle) * half, depth), transform,
        gripper_yaw_offset_deg=config["orientation"].get("gripper_yaw_offset_deg", 0),
        reference_yaw_deg=config["orientation"].get("reference_yaw_deg", 0))
    ref_angle = float('nan') if is_initial else math.radians(ref_det.long_axis_angle_deg)
    if math.isfinite(ref_angle):
        ru, rv = ref_det.center_uv
        ref_yaw = camera_axis_to_robot_yaw_deg(
            deproject(ru - math.cos(ref_angle) * half, rv - math.sin(ref_angle) * half, ref_depth),
            deproject(ru + math.cos(ref_angle) * half, rv + math.sin(ref_angle) * half, ref_depth), transform,
            gripper_yaw_offset_deg=config["orientation"].get("gripper_yaw_offset_deg", 0),
            reference_yaw_deg=config["orientation"].get("reference_yaw_deg", 0))
    else:
        ref_yaw = yaw
    if preview_scene is not None:
        yaw = footprint_yaw_deg(target_poly,config)
        if not is_initial:
            ref_yaw = footprint_yaw_deg(reference_poly,config)
    target_center = target_poly.mean(axis=0)
    reference_center = ref_point[:2]*1000.0 if is_initial else reference_poly.mean(axis=0)
    base_offsets = target_poly - target_center
    tcp_offset = np.asarray(config["grasp"].get("tcp_xy_offset_mm", [0, 0]), dtype=float)
    if tcp_offset.shape != (2,) or not np.isfinite(tcp_offset).all():
        _reject("invalid_relative_place_settings", field="tcp_xy_offset_mm")
    scenario = 'self_relative' if is_self else 'swap' if is_initial else 'relative'
    initial_yaw = ((initial_reference.get('yaw_deg') if initial_reference.get('yaw_deg') is not None else footprint_yaw_deg(reference_poly,config))
                   if is_initial else None)
    # Old standalone S3 fixtures retain their explicit surface policy. V2 config
    # carries all four policies; an explicit policy never falls back by 90 degrees.
    policy_config = config
    key = 'place_orientation_policy_' + scenario
    if key not in config.get('orientation',{}) and scenario == 'relative':
        policy_config = {**config,'orientation':{**config.get('orientation',{}),key:
            'align_reference' if surface.get('align_to_reference_yaw',False) else 'keep_target'}}
    orientation_policy, selected_yaw = resolve_place_orientation(policy_config,scenario,float(yaw),
        reference_yaw=float(ref_yaw),initial_yaw=initial_yaw,override=orientation_override)
    if locked_destination is not None:
        selected_yaw = locked_destination['place_yaw_deg']
        if top:
            z_mm = locked_destination['landing_surface_z_mm']
        cand_specs = cand_specs[:1]
    candidate_rotations = [selected_yaw - yaw]

    last_error = None
    chosen_plan = None
    for cand_dir, cand_gap in cand_specs:
        for idx_rot, rot_deg_delta in enumerate(candidate_rotations):
            rad = math.radians(rot_deg_delta)
            c_r, s_r = math.cos(rad), math.sin(rad)
            rot_mat = np.array([[c_r, -s_r], [s_r, c_r]], dtype=float)
            offsets = base_offsets @ rot_mat.T
            place_yaw = selected_yaw
            offset = displacement if is_self else 0.0 if top or is_initial else _center_advance_for_gap(
                reference_poly - reference_center, offsets, cand_dir, cand_gap + margin)
            center = reference_center + cand_dir * offset
            if locked_destination is not None:
                center = np.asarray(locked_destination['destination_center_robot_xy_mm'], dtype=float)
            destination = offsets + center
            envelope = clearance_envelope_obb(destination, padding, margin, tcp_offset)
            lower, upper = envelope.min(axis=0), envelope.max(axis=0)  # Diagnostics only.
            gap_metrics = {'center_distance_mm': float(np.linalg.norm(center-reference_center)),
                'projected_gap_mm': float(np.min(destination @ cand_dir)-np.max(reference_poly @ cand_dir)),
                'nearest_edge_gap_mm': polygon_distance_mm(reference_poly, destination)}
            diagnostic = {"candidate_center_robot_xy_mm": center.tolist(),
                          "candidate_footprint_robot_xy_mm": destination.tolist(),
                          "clearance_envelope_robot_xy_mm": envelope.tolist(), "table_z_mm": float(z_mm),
                          "place_yaw_deg": float(place_yaw), "orientation_policy":orientation_policy,
                          "requested_gap_mm": cand_gap, "spatial_relation": relation, **gap_metrics}
            try:
                for region, reserved in reservations:
                    occupied = envelope if region.get('check_envelope') else destination
                    if convex_polygons_intersect(occupied, reserved):
                        _reject('swap_temporary_area_reserved', stage='swap_reservation',
                                reserved_region=region['label'], **diagnostic)
                # Surface XY bounds and arm workspace XY bounds constrain the
                # destination *object centre*, not the padded gripper envelope.
                # Depth/height, other-object clearance and TCP pose checks below
                # remain unchanged, including their use of the full envelope.
                for axis, name in enumerate(("x", "y")):
                    coordinate = float(center[axis])
                    if not table[name][0] <= coordinate <= table[name][1]:
                        _reject("destination_outside_surface", stage="surface_bounds", axis=name,
                                allowed_mm=list(table[name]), center_mm=coordinate,
                                minimum_correction_mm=max(table[name][0] - coordinate, coordinate - table[name][1], 0), **diagnostic)
                    if not workspace[name][0] <= coordinate <= workspace[name][1]:
                        _reject("destination_outside_workspace", stage="workspace_bounds", axis=name,
                                allowed_mm=list(workspace[name]), center_mm=coordinate,
                                minimum_correction_mm=max(workspace[name][0] - coordinate, coordinate - workspace[name][1], 0), **diagnostic)
                for index, (polygon, obj_depth) in enumerate(measurements):
                    if index != ti and (not top or index != ri) and convex_polygons_intersect(envelope, polygon):
                        if top:
                            obj_z_mm = float((transform[:3, :3] @ np.asarray(deproject(*detections[index].center_uv, obj_depth)) + transform[:3, 3])[2] * 1000.0)
                            if (obj_z_mm < z_mm - tolerance
                                    and cv2.intersectConvexConvex(polygon.astype(np.float32), reference_poly.astype(np.float32))[0] > 0):
                                continue
                        _reject("gripper_area_occupied", stage="gripper_clearance", object=detections[index].class_name, **diagnostic)

                mask_diagnostics = {}
                checked_pixels, local_z_mm = _surface_clear(
                    envelope, frame, deproject, project, transform, z_mm, tolerance, config["depth"], allow_lower=top,
                    **({'exclude_footprint':target_poly,'exclude_top_z_mm':target_z_mm,
                        'exclude_image_polygon':det.corners_px,'mask_diagnostics':mask_diagnostics} if is_self or locked_destination is not None else {}))
                landing_z_mm = float(local_z_mm) if (surface.get("use_local_surface_z") and not top) else float(z_mm)
                grasp = build_top_down_plan(point, yaw, config["grasp"],
                    canonical_target=det.class_name, table_z_mm=base_table_z_mm)
                delta_z_grasp = max(0.0, float(grasp.grasp_pose_mm_deg[2]) - base_table_z_mm) if surface.get("use_local_surface_z") else 0.0
                effective_release = max(release, delta_z_grasp + margin) if (surface.get("use_local_surface_z") and not top) else release
                xy = center + tcp_offset
                pose = [float(xy[0]), float(xy[1]), float(landing_z_mm + effective_release), config["grasp"].get("rx_deg", -175),
                        config["grasp"].get("ry_deg", -5), float(place_yaw)]
                pre, retreat = list(pose), list(pose)
                approach_mm = _number(config["grasp"]["approach_clearance_mm"], "approach_clearance_mm", minimum=1)
                lift_mm = _number(config["grasp"]["lift_clearance_mm"], "lift_clearance_mm", minimum=1)
                pre[2] += approach_mm
                retreat[2] += lift_mm
                if surface.get("use_safe_transfer_z"):
                    pre[2] = float(min(
                        workspace["z"][1],
                        max(float(pre[2]), float(grasp.lift_pose_mm_deg[2]), float(ref_top_z_mm + approach_mm)),
                    ))
                for candidate_pose in (pose, pre, retreat):
                    if (not np.isfinite(candidate_pose).all()
                            or any(not workspace[a][0] <= candidate_pose[i] <= workspace[a][1] for i, a in enumerate(("x", "y", "z")))):
                        violations = [{"axis": a, "actual_mm": float(candidate_pose[i]), "allowed_mm": list(workspace[a])}
                                      for i, a in enumerate(("x", "y", "z")) if not workspace[a][0] <= candidate_pose[i] <= workspace[a][1]]
                        _reject("destination_outside_workspace", stage="tcp_pose_bounds", failed_pose_mm_deg=candidate_pose,
                                violations=violations, **diagnostic)

                chosen_plan = {
                    "cand_dir": cand_dir, "cand_gap": cand_gap, "landing_z_mm": landing_z_mm,
                    "local_z_mm": local_z_mm, "place_yaw": place_yaw, "center": center,
                    "destination": destination, "envelope": envelope, "grasp": grasp,
                    "pre": pre, "pose": pose, "retreat": retreat, "checked_pixels": checked_pixels,
                    'excluded_self_pixels':mask_diagnostics.get('excluded_self_pixels',0),
                    'gap_metrics':gap_metrics,
                }
                last_error = None
                break
            except SceneGroundingError as exc:
                exc.details.update({k: v for k, v in diagnostic.items() if k not in exc.details})
                last_error = exc
                if not is_near and idx_rot + 1 == len(candidate_rotations):
                    raise
        if chosen_plan is not None:
            break

    if chosen_plan is None:
        if last_error is not None:
            raise last_error
        _reject("no_valid_near_placement_candidate")

    cand_dir = chosen_plan["cand_dir"]
    cand_gap = chosen_plan["cand_gap"]
    landing_z_mm = chosen_plan["landing_z_mm"]
    local_z_mm = chosen_plan["local_z_mm"]
    place_yaw = chosen_plan["place_yaw"]
    center = chosen_plan["center"]
    destination = chosen_plan["destination"]
    envelope = chosen_plan["envelope"]
    grasp = chosen_plan["grasp"]
    pre = chosen_plan["pre"]
    pose = chosen_plan["pose"]
    retreat = chosen_plan["retreat"]
    checked_pixels = chosen_plan["checked_pixels"]

    return {
        "schema_version": 1, "status": "geometry_preview", "authorizes_motion": False,
        "requires_operator_confirmation": True, "target_object": target, "reference_object": reference,
        "spatial_relation": task["spatial_relation"], "target_index": ti, "reference_index": None if is_self else ri,
        'destination_type':'self_relative' if is_self else 'relative',
        'anchor_policy':'step_pre_pick' if is_self else 'plan_initial' if is_initial else 'current_reference',
        'coordinate_frame':'display_workbench_plane',
        'display_rotation_deg':rotation,
        'anchor_center_robot_xy_mm':target_center.tolist() if is_self else reference_center.tolist(),
        'requested_displacement_mm':displacement if is_self else None,
        'planned_displacement_mm':float(np.linalg.norm(center-target_center)) if is_self else None,
        'excluded_self_pixels':chosen_plan['excluded_self_pixels'],
        'swap_reserved_regions':reserved_regions or [],
        "calibration_id": _calibration_id(transform), "captured_at_unix_ms": captured,
        "snapshot_age_ms": now - captured, "max_scene_age_ms": age_limit,
        "gap_policy": 'self_center_displacement' if is_self else 'initial_pose_center' if is_initial else 'support_surface' if top else "minimum_edge_gap",
        "requested_gap_mm": None if top or is_self else cand_gap,
        "planned_gap_mm": None if top or is_self else cand_gap + margin,
        'gap_metric': 'minimum_euclidean_polygon_distance', **chosen_plan['gap_metrics'],
        "right_axis_robot_xy": right.tolist(), "reference_center_robot_xy_mm": reference_center.tolist(),
        'direction_axis_robot_xy': cand_dir.tolist(), 'landing_surface_z_mm': landing_z_mm,
        'expected_top_z_mm': (locked_destination['expected_top_z_mm'] if locked_destination is not None
                             else ref_top_z_mm + target_z_mm - base_table_z_mm) if top else None,
        "local_surface_z_mm": local_z_mm, "grasp_yaw_deg": float(yaw), "reference_yaw_deg": float(ref_yaw),
        "place_yaw_deg": float(place_yaw), "orientation_policy": orientation_policy,
        'support_model_checked': False, 'support_model': support, 'object_model': object_model,
        "destination_center_robot_xy_mm": center.tolist(), "destination_footprint_robot_xy_mm": destination.tolist(),
        "clearance_envelope_robot_xy_mm": envelope.tolist(), "source_grasp_plan": grasp.as_dict(),
        "candidate_tcp_poses_mm_deg": {"pre_place": pre, "place": pose, "retreat": retreat},
        "checked_surface_pixels": checked_pixels, "ik_validated": False, "path_validated": False,
        "limitations": ["Static horizontal table and OBB footprint approximation; target orientation preserved or aligned to reference.",
                        "Visible RGB-D surface check does not validate 3D gripper/arm collision or support stability.",
                        "Release TCP height and clearance margins require independent hardware measurements."] +
                       (["Selected target points are excluded at their measured height; the table under the lifted target is assumed to match the measured table model, not newly observed.",
                         "OBB/height masking is not instance segmentation; an unknown same-height intrusion inside the target footprint cannot be distinguished."] if is_self else []),
    }


def evaluate_relative_place(plan, detections, frame, *, deproject, config, position_tolerance_mm,
                            captured_at_unix_ms, now_unix_ms=None):
    """Evaluate observed object geometry, not the robot's command completion."""
    tolerance = _number(position_tolerance_mm, "position_tolerance_mm", minimum=0)
    captured = _number(captured_at_unix_ms, "captured_at_unix_ms")
    now = time.time() * 1000 if now_unix_ms is None else _number(now_unix_ms, "now_unix_ms")
    if captured <= plan["captured_at_unix_ms"]:
        _reject("completion_observation_not_new")
    if not 0 <= now - captured <= plan["max_scene_age_ms"]:
        _reject("scene_snapshot_stale")
    transform = as_transform(config["calibration"]["T_robot_from_camera"])
    if _calibration_id(transform) != plan["calibration_id"]:
        _reject("calibration_changed")
    target = _unique(detections, plan["target_object"])
    target_poly = _footprint(detections[target], frame, deproject, transform, config, tolerance)[0]
    if plan.get('destination_type') == 'self_relative':
        observed_center = target_poly.mean(axis=0)
        delta = observed_center - np.asarray(plan['anchor_center_robot_xy_mm'])
        error = float(np.linalg.norm(observed_center-np.asarray(plan['destination_center_robot_xy_mm'])))
        return {'status':'geometry_result','passed':error <= tolerance,'destination_error_mm':error,
                'observed_displacement_mm':float(np.linalg.norm(delta)),
                'requested_displacement_mm':plan['requested_displacement_mm'],
                'anchor_policy':'step_pre_pick','position_tolerance_mm':tolerance,'authorizes_motion':False}
    reference = _unique(detections, plan["reference_object"])
    reference_poly = _footprint(detections[reference], frame, deproject, transform, config, tolerance)[0]
    direction = np.asarray(plan.get('direction_axis_robot_xy', plan["right_axis_robot_xy"]))
    if 'direction_axis_robot_xy' not in plan and plan["spatial_relation"] == "left_of":
        direction = -direction
    projected_gap = float(np.min(target_poly @ direction) - np.max(reference_poly @ direction))
    nearest_gap = polygon_distance_mm(target_poly, reference_poly)
    gap = nearest_gap if plan.get('gap_metric') == 'minimum_euclidean_polygon_distance' else projected_gap
    error = float(np.linalg.norm(target_poly.mean(axis=0) - plan["destination_center_robot_xy_mm"]))
    reference_error = float(np.linalg.norm(reference_poly.mean(axis=0) - plan["reference_center_robot_xy_mm"]))
    top = plan['spatial_relation'] == 'on_top_of'
    vertical_error = reference_height_error = 0
    if top:
        def observed_height(index):
            depth = _footprint(detections[index], frame, deproject, transform, config, tolerance)[1]
            return float((transform[:3, :3] @ np.asarray(deproject(*detections[index].center_uv, depth))
                          + transform[:3, 3])[2] * 1000)
        obj_model = plan.get('object_model')
        if isinstance(obj_model, dict) and 'height_mm' in obj_model:
            vertical_error = abs(observed_height(target) - obj_model['height_mm'] - plan['landing_surface_z_mm'])
        else:
            vertical_error = max(0.0, plan['landing_surface_z_mm'] - observed_height(target))
        reference_height_error = abs(observed_height(reference) - plan['landing_surface_z_mm'])
    return {"status": "geometry_result", "passed": (top or gap >= plan["requested_gap_mm"] - tolerance)
            and error <= tolerance and reference_error <= tolerance
            and vertical_error <= tolerance and reference_height_error <= tolerance,
            "observed_gap_mm": None if top else gap, 'vertical_error_mm': vertical_error,
            'projected_gap_mm': projected_gap, 'nearest_edge_gap_mm': nearest_gap,
            'center_distance_mm': float(np.linalg.norm(target_poly.mean(axis=0)-reference_poly.mean(axis=0))),
            'reference_height_error_mm': reference_height_error,
            "destination_error_mm": error, "reference_motion_mm": reference_error,
            "position_tolerance_mm": tolerance, "authorizes_motion": False}


def match_result_target(expected, instances, *, stationary_radius_mm=20.0):
    """Exclude each unselected same-class object once, then match the moved target."""
    candidates = [i for i in instances if i['class_name'] == expected['target_object']]
    if any(i.get('xyz_mm') is None for i in candidates):
        _reject('result_target_depth_unavailable')
    candidates = list(candidates)
    for center in expected['unselected_centers_mm']:
        if not candidates:
            break
        other = min(candidates, key=lambda i: np.linalg.norm(np.asarray(i['xyz_mm'][:2]) - center))
        if np.linalg.norm(np.asarray(other['xyz_mm'][:2]) - center) <= stationary_radius_mm:
            candidates.remove(other)
    if not candidates:
        return None
    return min(candidates, key=lambda i: np.linalg.norm(
        np.asarray(i['xyz_mm'][:2]) - expected['destination_center_robot_xy_mm']))


def evaluate_local_result(expected, instances, *, position_tolerance_mm=25.0,
                          height_tolerance_mm=20.0, named_min_displacement_mm=25.0):
    """V2's permissive local object check; this never verifies robot arrival."""
    target = match_result_target(expected, instances)
    named = expected['destination_type'] == 'named'
    if target is None and not named:
        _reject('result_target_not_observed')
    result = {'status':'geometry_result', 'authorizes_motion':False,
              'target_instance_id':target['instance_id'] if target else None,
              'position_tolerance_mm':position_tolerance_mm,
              'height_tolerance_mm':height_tolerance_mm,
              'named_min_displacement_mm':named_min_displacement_mm}
    if target is None:
        return {**result, 'passed':True, 'basis':'named_target_not_observed'}
    xy = np.asarray(target['xyz_mm'][:2])
    displacement = float(np.linalg.norm(xy - expected['source_center_robot_xy_mm']))
    error = float(np.linalg.norm(xy - expected['destination_center_robot_xy_mm']))
    height_error = 0.0
    if expected['spatial_relation'] == 'on_top_of':
        height_error = abs(target['xyz_mm'][2] - expected['expected_top_z_mm'])
    # Sub-micrometre rounding avoids float32 projection flipping an exact boundary.
    passed = (round(displacement, 5) >= named_min_displacement_mm if named else
              round(error, 5) <= position_tolerance_mm and round(height_error, 5) <= height_tolerance_mm)
    return {**result, 'passed':bool(passed), 'destination_error_mm':error,
            'observed_displacement_mm':displacement, 'vertical_error_mm':height_error,
            'basis':'named_displacement' if named else 'center_and_height' if
            expected['spatial_relation'] == 'on_top_of' else 'center'}
