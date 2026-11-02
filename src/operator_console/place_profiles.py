"""Versioned fixed place-zone profiles and deterministic place planning.

Version 0.6 deliberately implements only Level-1 placement: named, fixed robot-
frame destinations.  Profiles never infer free space, obstacles, container
interiors, or support surfaces.  A profile must be validated against the active
workspace and explicitly activated by an operator before it can be used.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import math
import re
import threading

import yaml

from .events import atomic_json


CATALOG_SCHEMA_VERSION = 1
PROFILE_SCHEMA_VERSION = 3
PLACE_PLANNER_MODES = {"joint", "cartesian", "hybrid"}
SUPPORTED_RELATIONS = {"inside", "on_top_of"}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _clean_aliases(raw) -> list[str]:
    if raw is None:
        return []
    if isinstance(raw, str):
        items = re.split(r"[,，、;\n]+", raw)
    elif isinstance(raw, list):
        items = raw
    else:
        raise ValueError("aliases must be a list of strings or comma-separated string")
    out: list[str] = []
    seen: set[str] = set()
    for item in items:
        if not isinstance(item, str):
            raise ValueError("each alias must be a string")
        text = item.strip()
        if not text:
            continue
        if len(text) > 80:
            raise ValueError("alias must contain at most 80 characters")
        key = text.lower()
        if key not in seen:
            seen.add(key)
            out.append(text)
    return out














def _digest(value) -> str:
    return hashlib.sha256(
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    ).hexdigest()


def _slug(value: str) -> str:
    raw = value.strip()
    text = re.sub(r"[^A-Za-z0-9_-]+", "-", raw).strip("-").lower()
    if text:
        return text[:32]
    return "place-" + hashlib.sha256(raw.encode("utf-8")).hexdigest()[:10]


def _finite_number(value, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _pose(value, *, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{name} must contain six values [x,y,z,rx,ry,rz]")
    out = [_finite_number(item, name=f"{name}[{index}]") for index, item in enumerate(value)]
    if any(abs(angle) > 360 for angle in out[3:]):
        raise ValueError(f"{name} orientation values must stay within ±360 degrees")
    return out



def _joint_angles(value, *, name: str, required: bool = True):
    if value is None and not required:
        return None
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{name} must contain six joint angles")
    out = [_finite_number(item, name=f"{name}[{index}]") for index, item in enumerate(value)]
    if any(abs(angle) > 360 for angle in out):
        raise ValueError(f"{name} contains an invalid angle")
    return out

def _workspace_check(pose: list[float], workspace: dict, *, label: str) -> None:
    for index, axis in enumerate(("x", "y", "z")):
        bounds = workspace.get(axis)
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValueError(f"workspace.{axis} must contain [min,max]")
        lo, hi = map(float, bounds)
        value = float(pose[index])
        if not (math.isfinite(lo) and math.isfinite(hi) and lo <= value <= hi):
            raise ValueError(f"{label}: {axis.upper()}={value:.2f} mm outside workspace [{lo}, {hi}]")


@dataclass(frozen=True)
class PlacePlan:
    profile_id: str
    label: str
    reference_object: str
    spatial_relation: str
    pre_place_pose_mm_deg: tuple[float, float, float, float, float, float]
    place_pose_mm_deg: tuple[float, float, float, float, float, float]
    retreat_pose_mm_deg: tuple[float, float, float, float, float, float]
    transfer_mode: int = 0
    place_mode: int = 1
    retreat_mode: int = 1
    descent_strategy: str = "operator_selected"
    planner_mode: str = "hybrid"
    safe_transit_joint_angles_deg: tuple[float, float, float, float, float, float] | None = None
    place_joint_angles_deg: tuple[float, float, float, float, float, float] | None = None
    retreat_joint_angles_deg: tuple[float, float, float, float, float, float] | None = None
    pre_place_joint_angles_deg: tuple[float, float, float, float, float, float] | None = None
    preferred_ik_enabled: bool = False
    preferred_ik_reference_angles_deg: tuple[float, float, float, float, float, float] | None = None

    def as_dict(self) -> dict:
        def rounded(pose):
            return [round(float(value), 3) for value in pose]

        return {
            "schema_version": 1,
            "profile_id": self.profile_id,
            "label": self.label,
            "reference_object": self.reference_object,
            "spatial_relation": self.spatial_relation,
            "pre_place_pose_mm_deg": rounded(self.pre_place_pose_mm_deg),
            "place_pose_mm_deg": rounded(self.place_pose_mm_deg),
            "retreat_pose_mm_deg": rounded(self.retreat_pose_mm_deg),
            "transfer_mode": self.transfer_mode,
            "place_mode": self.place_mode,
            "retreat_mode": self.retreat_mode,
            "descent_strategy": self.descent_strategy,
            "planner_mode": "hybrid",
            "stored_planner_mode": self.planner_mode,
            "control_policy": "joint_transfer_vertical_tcp",
            "safe_transit_joint_angles_deg": [round(float(value), 3) for value in self.safe_transit_joint_angles_deg] if self.safe_transit_joint_angles_deg else None,
            "place_joint_angles_deg": [round(float(value), 3) for value in self.place_joint_angles_deg] if self.place_joint_angles_deg else None,
            "retreat_joint_angles_deg": [round(float(value), 3) for value in self.retreat_joint_angles_deg] if self.retreat_joint_angles_deg else None,
            "pre_place_joint_angles_deg": list(self.pre_place_joint_angles_deg) if self.pre_place_joint_angles_deg else None,
            "preferred_ik_enabled": bool(self.preferred_ik_enabled),
            "preferred_ik_reference_angles_deg": [round(float(value), 3) for value in self.preferred_ik_reference_angles_deg] if self.preferred_ik_reference_angles_deg else None,
            "transfer_strategy": "joint_preferred_with_cartesian_fallback",
            "preferred_ik": {
                "enabled": bool(self.preferred_ik_enabled),
                "reference_angles_deg": [round(float(value), 3) for value in self.preferred_ik_reference_angles_deg] if self.preferred_ik_reference_angles_deg else None,
                "meaning": "soft_posture_preference_not_collision_avoidance",
            },
            "limitations": [
                "Fixed robot-frame destination; no visual tracking of the destination.",
                "No general collision planner; the operator remains responsible for transfer-path clearance.",
                "Controller/workspace checks remain active; failed IK may use bounded Cartesian transfer with fresh arrival feedback.",
            ],
        }


class PlaceProfileManager:
    """Maintain immutable candidate versions and explicitly active place zones."""

    def __init__(self, root: str | Path, config_path: str | Path, *, asset_registry=None, state_root: str | Path | None = None):
        self.root = Path(root).resolve()
        self.config_path = Path(config_path).resolve()
        self.state_root = Path(state_root or self.root / "operator_data").resolve()
        self.path = self.state_root / "place_profiles.json"
        self.asset_registry = asset_registry
        self._lock = threading.RLock()
        self.state_root.mkdir(parents=True, exist_ok=True)
        if not self.path.exists():
            atomic_json(self.path, self._empty())

    @staticmethod
    def _empty() -> dict:
        return {'schema_version': CATALOG_SCHEMA_VERSION, 'updated_at': None,
                'default_profile_id': None, 'profiles': []}

    def _load(self) -> dict:
        if self.path.is_symlink():
            raise ValueError("Symlink place-profile catalog is not supported")
        if not self.path.is_file():
            return self._empty()
        data = json.loads(self.path.read_text(encoding="utf-8"))
        if data.get("schema_version") != CATALOG_SCHEMA_VERSION or not isinstance(data.get("profiles"), list):
            raise ValueError("Unsupported place-profile catalog")
        for key in ('active_layout', 'layouts', 'slot_bindings'):
            data.pop(key, None)
        data.setdefault("default_profile_id", None)
        return data

    def _save(self, data: dict) -> None:
        for key in ('active_layout', 'layouts', 'slot_bindings'):
            data.pop(key, None)
        data["updated_at"] = _utc_now()
        atomic_json(self.path, data)

    def _config(self) -> dict:
        data = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise ValueError("config root must be an object")
        return data

    def _world_entities(self) -> set[str]:
        path = self.root / "world_entities.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        return set((data.get("entities") or {}).keys())

    def _dependencies(self) -> dict:
        if self.asset_registry is None:
            return {"calibration_asset_id": None, "motion_safety_asset_id": None}
        snapshot = self.asset_registry.current_snapshot()
        by_kind = {item.get("kind"): item for item in snapshot.get("assets", [])}
        return {
            "calibration_asset_id": (by_kind.get("calibration_profile") or {}).get("asset_id"),
            "motion_safety_asset_id": (by_kind.get("motion_safety_profile") or {}).get("asset_id"),
        }

    def _validate_payload(self, payload: dict) -> dict:
        allowed = {
            "label", "display_name", "aliases",
            "reference_object", "spatial_relation", "allowed_objects",
            "place_pose_mm_deg", "approach_clearance_mm", "retreat_clearance_mm",
            "notes", "base_profile_id", "placement_source", "safe_transit_joint_angles_deg",
            "planner_mode", "place_joint_angles_deg", "retreat_joint_angles_deg",
            "pre_place_joint_angles_deg",
            "preferred_ik_enabled", "preferred_ik_reference_angles_deg",
        }
        if not isinstance(payload, dict) or set(payload) - allowed:
            raise ValueError("Unknown place-profile field")
        label = payload.get("label") or payload.get("display_name")
        if not isinstance(label, str) or not label.strip() or len(label.strip()) > 80:
            raise ValueError("label must contain 1–80 characters")
        reference = payload.get("reference_object")
        if reference not in self._world_entities():
            raise ValueError("reference_object must exist in world_entities.json")
        relation = payload.get("spatial_relation")
        if relation not in SUPPORTED_RELATIONS:
            raise ValueError("spatial_relation must be inside or on_top_of")
        # Legacy clients may still send allowed_objects. Destinations are now
        # object-independent: accept but ignore the old field, including on
        # existing profiles. An empty stored list denotes unrestricted use.
        objects = []
        place = _pose(payload.get("place_pose_mm_deg"), name="place_pose_mm_deg")
        approach = _finite_number(payload.get("approach_clearance_mm", 60), name="approach_clearance_mm")
        retreat = _finite_number(payload.get("retreat_clearance_mm", 80), name="retreat_clearance_mm")
        if not 20 <= approach <= 150 or not 20 <= retreat <= 180:
            raise ValueError("place clearances must stay within approach 20–150 mm and retreat 20–180 mm")
        notes = payload.get("notes", "")
        if not isinstance(notes, str) or len(notes) > 2000:
            raise ValueError("notes must be at most 2000 characters")
        base_id = payload.get("base_profile_id") or None
        if base_id is not None and (not isinstance(base_id, str) or len(base_id) > 120):
            raise ValueError("Invalid base_profile_id")
        placement_source = payload.get("placement_source") or {"type": "manual_entry"}
        if not isinstance(placement_source, dict):
            raise ValueError("placement_source must be an object")
        source_type = placement_source.get("type", "manual_entry")
        if source_type not in {"manual_entry", "robot_pose_read", "robot_teaching", "visual_manual_selection"}:
            raise ValueError("Unsupported placement_source.type")
        # Keep provenance compact and JSON-only; never trust source metadata as a safety gate.
        placement_source = json.loads(json.dumps(placement_source, ensure_ascii=False, allow_nan=False))
        planner_mode = str(payload.get("planner_mode") or "hybrid").strip().lower()
        if planner_mode not in PLACE_PLANNER_MODES:
            raise ValueError("planner_mode must be joint, cartesian, or hybrid")
        safe_transit = _joint_angles(payload.get("safe_transit_joint_angles_deg"), name="safe_transit_joint_angles_deg", required=False)
        place_joint = _joint_angles(payload.get("place_joint_angles_deg"), name="place_joint_angles_deg", required=planner_mode == "joint")
        retreat_joint = _joint_angles(payload.get("retreat_joint_angles_deg"), name="retreat_joint_angles_deg", required=planner_mode == "joint")
        pre_joint = _joint_angles(payload.get("pre_place_joint_angles_deg"), name="pre_place_joint_angles_deg", required=False)
        if planner_mode == "joint" and (place_joint is None or retreat_joint is None):
            raise ValueError("joint planner requires place_joint_angles_deg and retreat_joint_angles_deg")
        preferred_ik_enabled = payload.get("preferred_ik_enabled", False)
        if not isinstance(preferred_ik_enabled, bool):
            raise ValueError("preferred_ik_enabled must be boolean")
        preferred_ik_reference = _joint_angles(
            payload.get("preferred_ik_reference_angles_deg"),
            name="preferred_ik_reference_angles_deg", required=preferred_ik_enabled,
        )

        pre = list(place)
        pre[2] += approach
        retreat_pose = list(place)
        retreat_pose[2] += retreat
        workspace = (self._config().get("grasp") or {}).get("workspace_mm") or {}
        for pose, name in ((place, "place"), (pre, "pre_place"), (retreat_pose, "retreat")):
            _workspace_check(pose, workspace, label=name)

        return {
            "label": label.strip(),
            "profile_key": _slug(label),
            "reference_object": reference,
            "spatial_relation": relation,
            "allowed_objects": objects,
            "place_pose_mm_deg": place,
            "approach_clearance_mm": approach,
            "retreat_clearance_mm": retreat,
            "pre_place_pose_mm_deg": pre,
            "retreat_pose_mm_deg": retreat_pose,
            "notes": notes.strip(),
            "base_profile_id": base_id,
            "placement_source": placement_source,
            "planner_mode": planner_mode,
            "safe_transit_joint_angles_deg": safe_transit,
            "place_joint_angles_deg": place_joint,
            "retreat_joint_angles_deg": retreat_joint,
            "pre_place_joint_angles_deg": pre_joint,
            "preferred_ik_enabled": preferred_ik_enabled,
            "preferred_ik_reference_angles_deg": preferred_ik_reference,
        }

    def create_candidate(self, payload: dict) -> dict:
        clean = self._validate_payload(payload)
        aliases = _clean_aliases(payload.get("aliases")) if isinstance(payload, dict) else []
        display_name = str(payload.get("display_name") or clean["label"]).strip() if isinstance(payload, dict) else clean["label"]
        with self._lock:
            catalog = self._load()
            if clean.get('base_profile_id') and clean['placement_source'].get('type') in {'robot_pose_read', 'visual_manual_selection'}:
                base = next((p for p in catalog['profiles'] if p['profile_id'] == clean['base_profile_id']), None)
                if base is None:
                    raise ValueError('teaching_base_profile_not_found')
                clean['profile_key'] = base['profile_key']
            dependencies = self._dependencies()
            identity = {**clean, **dependencies}
            digest = _digest(identity)
            existing = next((item for item in catalog["profiles"] if item.get("profile_digest") == digest), None)
            if existing:
                if aliases and not existing.get("aliases"):
                    existing["aliases"] = aliases
                return {"profile": self._effective(existing, dependencies), "already_existed": True}
            versions = [int(item.get("version", 0)) for item in catalog["profiles"] if item.get("profile_key") == clean["profile_key"]]
            version = max(versions, default=0) + 1
            profile_id = f"{clean['profile_key']}_v{version}_{digest[:8]}"
            profile = {
                "schema_version": PROFILE_SCHEMA_VERSION,
                "profile_id": profile_id,
                "version": version,
                "status": "candidate",
                "created_at": _utc_now(),
                "activated_at": None,
                "disabled_at": None,
                "clear_path_acknowledged": False,
                "profile_digest": digest,
                "display_name": display_name,
                "aliases": aliases,
                **clean,
                **dependencies,
                "place_policy": {
                    "fixed_robot_frame": True,
                    "visual_destination_tracking": False,
                    "collision_avoidance": "guarded_no_general_planner",
                    "place_planner_v1": True,
                    "preferred_ik_v1": bool(clean["preferred_ik_enabled"]),
                    "planner_mode": clean["planner_mode"],
                    "descent_strategy": "operator_selected",
                    "max_coordinate_speed": 100,
                    "confirmation_mode": "operator_selected",
                },
            }
            catalog["profiles"].append(profile)
            self._save(catalog)
            return {"profile": self._effective(profile, dependencies), "already_existed": False}

    def _effective(self, profile: dict, dependencies: dict) -> dict:
        item = json.loads(json.dumps(profile, ensure_ascii=False))
        display_name = str(item.get("display_name") or item.get("label") or item.get("profile_id") or "").strip()
        item["display_name"] = display_name
        item["label"] = display_name
        item["aliases"] = _clean_aliases(item.get("aliases"))
        effective = item.get("status")
        reasons = []
        if effective == "active":
            if item.get("calibration_asset_id") != dependencies.get("calibration_asset_id"):
                effective = "needs_revalidation"
                reasons.append("calibration_asset_changed")
            if item.get("motion_safety_asset_id") != dependencies.get("motion_safety_asset_id"):
                effective = "needs_revalidation"
                reasons.append("motion_safety_asset_changed")
            if not item.get("clear_path_acknowledged"):
                effective = "needs_revalidation"
                reasons.append("clear_path_not_acknowledged")
        item["effective_status"] = effective
        item["revalidation_reasons"] = reasons
        return item



    def status(self) -> dict:
        with self._lock:
            catalog = self._load()
            dependencies = self._dependencies()
            profiles = [self._effective(row, dependencies) for row in catalog['profiles']]
            profiles.sort(key=lambda row: (row.get('profile_key', ''), -int(row.get('version', 0))))
            grasp = self._config().get('grasp') or {}
            return {'schema_version': CATALOG_SCHEMA_VERSION, 'updated_at': catalog.get('updated_at'),
                    'dependencies': dependencies, 'profiles': profiles,
                    'active_profiles': [r for r in profiles if r['effective_status'] == 'active'],
                    'default_profile_id': catalog.get('default_profile_id'),
                    'default_ready': any(r['profile_id'] == catalog.get('default_profile_id') and r['effective_status'] == 'active' for r in profiles),
                    'default_orientation_deg': [grasp.get('rx_deg', -175.), grasp.get('ry_deg', -5.), -45.]}





    def update_metadata(
        self,
        profile_id: str,
        *,
        display_name: str | None = None,
        label: str | None = None,
        aliases: list[str] | str | None = None,
        notes: str | None = None,
    ) -> dict:
        with self._lock:
            catalog = self._load()
            target = next((item for item in catalog["profiles"] if item.get("profile_id") == profile_id), None)
            if not target:
                raise KeyError("Place profile not found")
            new_name = display_name if display_name is not None else label
            if new_name is not None:
                if not isinstance(new_name, str) or not new_name.strip() or len(new_name.strip()) > 80:
                    raise ValueError("display_name must contain 1–80 characters")
                clean_name = new_name.strip()
                target["display_name"] = clean_name
                target["label"] = clean_name
            if aliases is not None:
                target["aliases"] = _clean_aliases(aliases)
            if notes is not None:
                if not isinstance(notes, str) or len(notes) > 2000:
                    raise ValueError("notes must be at most 2000 characters")
                target["notes"] = notes.strip()
            self._save(catalog)
            return self._effective(target, self._dependencies())



    def activate(self, profile_id: str, *, confirm: str, clear_path_acknowledged: bool) -> dict:
        if confirm != 'ACTIVATE_PLACE_PROFILE' or clear_path_acknowledged is not True:
            raise ValueError('Explicit activation and clear-path acknowledgement are required')
        with self._lock:
            catalog = self._load()
            target = next((r for r in catalog['profiles'] if r['profile_id'] == profile_id), None)
            if target is None:
                raise KeyError('Place profile not found')
            self._validate_payload({key: target.get(key) for key in (
                'label', 'reference_object', 'spatial_relation', 'allowed_objects', 'place_pose_mm_deg',
                'approach_clearance_mm', 'retreat_clearance_mm', 'notes', 'base_profile_id', 'placement_source',
                'safe_transit_joint_angles_deg', 'planner_mode', 'place_joint_angles_deg', 'retreat_joint_angles_deg',
                'pre_place_joint_angles_deg', 'preferred_ik_enabled', 'preferred_ik_reference_angles_deg')})
            deps = self._dependencies()
            now = _utc_now()
            for row in catalog['profiles']:
                if row is not target and row.get('profile_key') == target.get('profile_key') and row.get('status') == 'active':
                    if catalog.get('default_profile_id') == row['profile_id']:
                        catalog['default_profile_id'] = profile_id
                    row.update(status='retired', disabled_at=now)
            target.update(status='active', activated_at=now, disabled_at=None,
                          clear_path_acknowledged=True, **deps)
            self._save(catalog)
            return self._effective(target, deps)

    def sync_motion_fingerprint(self, previous_dependencies: dict, eligible_ids: list[str]) -> list[str]:
        """Rebind only previously valid stations after a controlled tuning save.

        Calibration changes and pre-existing invalid stations cannot be repaired
        by this operation. Geometry is rechecked without changing taught values.
        The caller owns the configuration transaction and rolls back on failure.
        """
        with self._lock:
            current = self._dependencies()
            if current.get('calibration_asset_id') != previous_dependencies.get('calibration_asset_id'):
                raise ValueError('calibration_changed_requires_station_revalidation')
            catalog = self._load()
            updated = []
            for row in catalog['profiles']:
                if row['profile_id'] not in eligible_ids:
                    continue
                if self._effective(row, previous_dependencies)['effective_status'] != 'active':
                    raise ValueError('station_changed_during_tuning')
                self._validate_payload({key: row.get(key) for key in (
                    'label','reference_object','spatial_relation','allowed_objects',
                    'place_pose_mm_deg','approach_clearance_mm','retreat_clearance_mm',
                    'notes','base_profile_id','placement_source','safe_transit_joint_angles_deg',
                    'planner_mode','place_joint_angles_deg','retreat_joint_angles_deg',
                    'pre_place_joint_angles_deg','preferred_ik_enabled','preferred_ik_reference_angles_deg')})
                row['motion_safety_asset_id'] = current['motion_safety_asset_id']
                updated.append(row['profile_id'])
            if updated:
                self._save(catalog)
            return updated

    def disable(self, profile_id: str, *, confirm: str) -> dict:
        if confirm != 'DISABLE_PLACE_PROFILE':
            raise ValueError('Explicit disable confirmation is required')
        with self._lock:
            catalog = self._load()
            target = next((r for r in catalog['profiles'] if r['profile_id'] == profile_id), None)
            if target is None:
                raise KeyError('Place profile not found')
            target.update(status='disabled', disabled_at=_utc_now())
            if catalog.get('default_profile_id') == profile_id:
                catalog['default_profile_id'] = None
            self._save(catalog)
            return self._effective(target, self._dependencies())

    def delete(self, profile_id: str, *, confirm: str, force_unbind: bool = False) -> dict:
        if confirm != 'DELETE_PLACE_PROFILE':
            raise ValueError('Explicit delete confirmation is required')
        with self._lock:
            catalog = self._load()
            if not any(r['profile_id'] == profile_id for r in catalog['profiles']):
                raise KeyError('Place profile not found')
            catalog['profiles'] = [r for r in catalog['profiles'] if r['profile_id'] != profile_id]
            if catalog.get('default_profile_id') == profile_id:
                catalog['default_profile_id'] = None
            self._save(catalog)
            return {'deleted_profile_id': profile_id, 'place_profiles': self.status()}

    def get(self, profile_id: str, *, require_active: bool = False) -> dict:
        status = self.status()
        profile = next((item for item in status["profiles"] if item.get("profile_id") == profile_id), None)
        if not profile:
            raise KeyError("Place profile not found")
        if require_active and profile.get("effective_status") != "active":
            raise ValueError("place_profile_not_active_or_requires_revalidation")
        return profile





    def set_default(self, profile_id: str) -> dict:
        if not isinstance(profile_id, str) or not profile_id:
            raise ValueError("default_profile_id_required")
        with self._lock:
            catalog = self._load()
            row = next((p for p in catalog["profiles"] if p["profile_id"] == profile_id), None)
            if row is None or self._effective(row, self._dependencies())["effective_status"] != "active":
                raise ValueError("default_profile_must_be_active")
            catalog["default_profile_id"] = profile_id
            self._save(catalog)
        return self.status()

    def get_default(self) -> dict:
        with self._lock:
            profile_id = self._load().get("default_profile_id")
        if not profile_id:
            raise ValueError("default_place_profile_not_configured")
        return self.get(profile_id, require_active=True)

    @staticmethod
    def build_plan(profile: dict) -> PlacePlan:
        if profile.get("effective_status") != "active":
            raise ValueError("place_profile_not_active_or_requires_revalidation")
        return PlacePlan(
            profile_id=profile["profile_id"],
            label=profile.get("display_name") or profile["label"],
            reference_object=profile["reference_object"],
            spatial_relation=profile["spatial_relation"],
            pre_place_pose_mm_deg=tuple(float(v) for v in profile["pre_place_pose_mm_deg"]),
            place_pose_mm_deg=tuple(float(v) for v in profile["place_pose_mm_deg"]),
            retreat_pose_mm_deg=tuple(float(v) for v in profile["retreat_pose_mm_deg"]),
            planner_mode=str(profile.get("planner_mode") or "hybrid"),
            safe_transit_joint_angles_deg=tuple(float(v) for v in profile["safe_transit_joint_angles_deg"]) if profile.get("safe_transit_joint_angles_deg") else None,
            place_joint_angles_deg=tuple(float(v) for v in profile["place_joint_angles_deg"]) if profile.get("place_joint_angles_deg") else None,
            retreat_joint_angles_deg=tuple(float(v) for v in profile["retreat_joint_angles_deg"]) if profile.get("retreat_joint_angles_deg") else None,
            pre_place_joint_angles_deg=tuple(float(v) for v in profile["pre_place_joint_angles_deg"]) if profile.get("pre_place_joint_angles_deg") else None,
            preferred_ik_enabled=bool(profile.get("preferred_ik_enabled", False)),
            preferred_ik_reference_angles_deg=tuple(float(v) for v in profile["preferred_ik_reference_angles_deg"]) if profile.get("preferred_ik_reference_angles_deg") else None,
        )
