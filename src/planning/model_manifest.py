from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
import json
import math
import yaml

from .errors import PlanningError, PlanningStatus


def sha256_file(path: str | Path) -> str:
    h = sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_hash(data: dict) -> str:
    payload = json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return sha256(payload).hexdigest()


def _finite_vector(value, n: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, f"{label} must contain exactly {n} values")
    out = []
    for item in value:
        if isinstance(item, bool):
            raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, f"{label} must be numeric")
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, f"{label} must be numeric") from exc
        if not math.isfinite(number):
            raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, f"{label} must be finite")
        out.append(number)
    return tuple(out)


@dataclass(frozen=True)
class VerifiedManifest:
    path: Path
    data: dict
    version: str
    manifest_hash: str


def _load_yaml(path: str | Path) -> tuple[Path, dict]:
    p = Path(path).resolve()
    if not p.exists():
        raise PlanningError(PlanningStatus.ROBOT_MODEL_UNVERIFIED, f"manifest not found: {p}")
    data = yaml.safe_load(p.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"manifest must be a mapping: {p}")
    return p, data


def load_verified_robot_model_manifest(path: str | Path) -> VerifiedManifest:
    p, data = _load_yaml(path)
    if data.get("kind") != "visionpick_robot_model":
        raise PlanningError(PlanningStatus.ROBOT_MODEL_UNVERIFIED, "unexpected robot model manifest kind")
    if data.get("verified") is not True or data.get("status") != "verified":
        raise PlanningError(
            PlanningStatus.ROBOT_MODEL_UNVERIFIED,
            "robot collision model is not hardware-verified; MoveIt validation must remain disabled",
            details={"blocking_reasons": list(data.get("blocking_reasons") or [])},
        )
    hw = data.get("hardware") or {}
    required_hw = ("manufacturer", "family", "exact_model", "gripper", "gripper_adapter", "camera_mount", "robot_mounting")
    missing = [k for k in required_hw if not str(hw.get(k) or "").strip()]
    moveit = data.get("moveit") or {}
    for key in ("planning_group", "tip_link", "base_frame"):
        if not str(moveit.get(key) or "").strip():
            missing.append(f"moveit.{key}")
    joint_names = [str(x).strip() for x in (moveit.get("joint_names") or [])]
    if len(joint_names) != 6 or len(set(joint_names)) != 6 or any(not x for x in joint_names):
        missing.append("moveit.joint_names[6]")
    if missing:
        raise PlanningError(PlanningStatus.ROBOT_MODEL_UNVERIFIED, "robot model manifest is incomplete", details={"missing": missing})
    root = p.parent
    artifacts = data.get("artifacts") or {}
    if not artifacts:
        raise PlanningError(PlanningStatus.ROBOT_MODEL_UNVERIFIED, "verified robot model must version at least one selected description artifact")
    for key, item in artifacts.items():
        if not isinstance(item, dict) or not item.get("path") or not item.get("sha256"):
            raise PlanningError(PlanningStatus.ROBOT_MODEL_UNVERIFIED, f"artifact {key} lacks path/sha256")
        artifact = (root / item["path"]).resolve()
        if not artifact.exists():
            raise PlanningError(PlanningStatus.ROBOT_MODEL_UNVERIFIED, f"artifact missing: {artifact}")
        actual = sha256_file(artifact)
        if actual != item["sha256"]:
            raise PlanningError(PlanningStatus.ROBOT_MODEL_MISMATCH, f"artifact hash mismatch: {key}")
    return VerifiedManifest(p, data, str(data["robot_model_version"]), canonical_hash(data))


def load_verified_scene_manifest(path: str | Path) -> VerifiedManifest:
    p, data = _load_yaml(path)
    if data.get("kind") != "visionpick_planning_scene":
        raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, "unexpected scene manifest kind")
    if data.get("verified") is not True or data.get("status") != "verified":
        raise PlanningError(
            PlanningStatus.SCENE_UNVERIFIED,
            "planning scene is not hardware-verified; MoveIt validation must remain disabled",
            details={"blocking_reasons": list(data.get("blocking_reasons") or [])},
        )
    frame = str(data.get("planning_frame") or "").strip()
    if not frame:
        raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, "planning_frame is required")
    objects = data.get("objects") or []
    if not objects:
        raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, "verified planning scene must contain at least one measured collision object")
    object_ids: set[str] = set()
    for obj in objects:
        if not isinstance(obj, dict) or obj.get("shape") != "box":
            raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, "0.9 scene objects must be verified boxes")
        object_id = str(obj.get("id") or "").strip()
        if not object_id or object_id in object_ids:
            raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, "scene object ids must be non-empty and unique")
        object_ids.add(object_id)
        dims = _finite_vector(obj.get("dimensions_mm"), 3, f"scene object {object_id} dimensions_mm")
        if any(v <= 0 for v in dims):
            raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, f"scene object {object_id} dimensions must be > 0")
        _finite_vector(obj.get("pose_xyz_mm"), 3, f"scene object {object_id} pose_xyz_mm")
        _finite_vector(obj.get("pose_rpy_deg") or (0, 0, 0), 3, f"scene object {object_id} pose_rpy_deg")
        if obj.get("frame_id") is not None and not str(obj.get("frame_id") or "").strip():
            raise PlanningError(PlanningStatus.SCENE_UNVERIFIED, f"scene object {object_id} frame_id cannot be blank")
    return VerifiedManifest(p, data, str(data["scene_version"]), canonical_hash(data))
