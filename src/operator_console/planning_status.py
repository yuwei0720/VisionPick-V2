"""Read-only collision-validation readiness for the operator console.

This module intentionally does not contact ROS/MoveIt or hardware.  It turns the
0.9 planning configuration and manifests into a small, truthful status object
that can be shown in the local UI and stored in diagnostics.
"""
from __future__ import annotations

from pathlib import Path
import hashlib
import yaml


def _read_yaml(path: Path) -> dict:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except FileNotFoundError:
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def _sha256(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def _resolve_under_root(root: Path, value: object) -> Path | None:
    if not isinstance(value, str) or not value.strip():
        return None
    candidate = (root / value).resolve() if not Path(value).is_absolute() else Path(value).resolve()
    # Planning status is metadata-only; never allow a manifest path to escape the project root.
    try:
        candidate.relative_to(root)
    except ValueError:
        return None
    return candidate


def planning_readiness(root: str | Path, *, config_path: str = "config/planning_v09.yaml") -> dict:
    root = Path(root).resolve()
    cfg_path = _resolve_under_root(root, config_path)
    cfg = _read_yaml(cfg_path) if cfg_path else {}
    planning = cfg.get("planning") if isinstance(cfg.get("planning"), dict) else {}

    robot_path = _resolve_under_root(root, planning.get("robot_model_manifest"))
    scene_path = _resolve_under_root(root, planning.get("scene_manifest"))
    robot = _read_yaml(robot_path) if robot_path else {}
    scene = _read_yaml(scene_path) if scene_path else {}

    enabled = planning.get("enabled") is True
    robot_verified = robot.get("verified") is True
    scene_verified = scene.get("verified") is True
    paths_present = bool(cfg_path and cfg_path.exists() and robot_path and robot_path.exists() and scene_path and scene_path.exists())

    blockers: list[dict] = []
    if not cfg_path or not cfg_path.exists():
        blockers.append({"component": "planning_config", "reason": "planning_v09 configuration is missing"})
    if not robot_path or not robot_path.exists():
        blockers.append({"component": "robot_model", "reason": "robot model manifest is missing or outside the project root"})
    elif not robot_verified:
        reasons = robot.get("blocking_reasons") if isinstance(robot.get("blocking_reasons"), list) else []
        blockers.extend({"component": "robot_model", "reason": str(reason)} for reason in reasons)
        if not reasons:
            blockers.append({"component": "robot_model", "reason": "robot model is not hardware-verified"})
    if not scene_path or not scene_path.exists():
        blockers.append({"component": "scene", "reason": "planning scene manifest is missing or outside the project root"})
    elif not scene_verified:
        reasons = scene.get("blocking_reasons") if isinstance(scene.get("blocking_reasons"), list) else []
        blockers.extend({"component": "scene", "reason": str(reason)} for reason in reasons)
        if not reasons:
            blockers.append({"component": "scene", "reason": "planning scene is not hardware-verified"})

    # READY is deliberately strict.  A configured endpoint alone is never evidence of safety.
    if enabled and robot_verified and scene_verified and paths_present:
        status = "READY"
        summary = "目標位置檢查可用；整段移動路徑尚未驗證。"
    elif blockers:
        status = "BLOCKED"
        summary = "碰撞檢查尚未完成設備驗證，目前不能據此允許手臂移動。"
    else:
        status = "DISABLED"
        summary = "碰撞驗證目前停用。"

    hardware = robot.get("hardware") if isinstance(robot.get("hardware"), dict) else {}
    sensor_facts = scene.get("sensor_facts") if isinstance(scene.get("sensor_facts"), dict) else {}
    realsense = sensor_facts.get("realsense") if isinstance(sensor_facts.get("realsense"), dict) else {}
    gripper = hardware.get("end_effector") if isinstance(hardware.get("end_effector"), dict) else {}
    gripper_name = gripper.get("model") or gripper.get("name") or hardware.get("gripper") or hardware.get("end_effector_model")

    return {
        "schema_version": 1,
        "feature": "static_collision_goal_validation",
        "status": status,
        "summary": summary,
        "enabled": enabled,
        "mode": planning.get("mode", "unknown"),
        "endpoint": planning.get("endpoint"),
        "required_validation_level": planning.get("required_validation_level"),
        "assurance_scope": "goal_only_level_0_1",
        "guard_enforcement": False,
        "trajectory_execution": False,
        "robot_model": {
            "verified": robot_verified,
            "version": robot.get("robot_model_version"),
            "status": robot.get("status"),
            "manufacturer": hardware.get("manufacturer"),
            "model": hardware.get("model") or hardware.get("exact_model"),
            "end_effector": gripper_name,
            "manifest": str(robot_path.relative_to(root)) if robot_path and robot_path.exists() else None,
            "manifest_sha256": _sha256(robot_path) if robot_path else None,
        },
        "scene": {
            "verified": scene_verified,
            "version": scene.get("scene_version"),
            "status": scene.get("status"),
            "planning_frame": scene.get("planning_frame"),
            "manifest": str(scene_path.relative_to(root)) if scene_path and scene_path.exists() else None,
            "manifest_sha256": _sha256(scene_path) if scene_path else None,
            "realsense_mounting": realsense.get("mounting"),
            "realsense_exact_model": realsense.get("exact_model"),
            "realsense_pose_verified": bool(realsense.get("pose_in_robot_base_xyz_mm") and realsense.get("pose_in_robot_base_rpy_deg")),
        },
        "blocking_reasons": blockers,
        "safety_note": (
            "這裡只顯示目標位置的檢查狀態；"
            "整段路徑、實機碰撞與急停能力仍需另行驗證。"
        ),
    }
