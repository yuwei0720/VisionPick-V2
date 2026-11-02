"""Versioned, read-only project asset provenance for operator runs.

The registry fingerprints files and deterministic configuration subsets that affect
semantic interpretation, perception, calibration, and motion planning.  It does
not mutate those assets or activate candidates; it only records what was active.
"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import math
import threading

import yaml

from .events import atomic_json, sha256


REGISTRY_SCHEMA_VERSION = 1
SNAPSHOT_SCHEMA_VERSION = 1


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _json_safe(value):
    """Return a deterministic JSON-safe representation, including invalid floats.

    Invalid configuration values must remain fingerprintable so provenance
    recording never masks the original validation failure.
    """
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in sorted(value.items(), key=lambda row: str(row[0]))}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"__nonfinite__": "NaN" if math.isnan(value) else ("Infinity" if value > 0 else "-Infinity")}
    return value


def _json_digest(value) -> str:
    raw = json.dumps(_json_safe(value), sort_keys=True, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _relative(root: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


class AssetRegistry:
    """Fingerprint active project assets and keep a small append-only catalog."""

    FILE_ASSETS = (
        ("semantic_tools", "tools.json"),
        ("world_entities", "world_entities.json"),
        ("model_policy", "model_policy.json"),
        ("task_schema", "task_schema_v2.json"),
        ("runtime_config", "config/v0.yaml"),
    )

    def __init__(self, root: str | Path, state_root: str | Path | None = None, *, config_path=None):
        self.root = Path(root).resolve()
        self.state_root = Path(state_root or self.root / "operator_data").resolve()
        self.config_path = Path(config_path or self.root / "config/v0.yaml").resolve()
        self.path = self.state_root / "asset_registry.json"
        self._lock = threading.RLock()

    def _file_asset(self, kind: str, relative_path: str) -> dict:
        path = self.root / relative_path
        if not path.is_file() or path.is_symlink():
            return {
                "kind": kind,
                "status": "missing",
                "path": relative_path,
                "asset_id": f"{kind}:missing",
                "sha256": None,
                "bytes": None,
            }
        digest = sha256(path)
        return {
            "kind": kind,
            "status": "available",
            "path": _relative(self.root, path),
            "asset_id": f"{kind}:{digest[:12]}",
            "sha256": digest,
            "bytes": path.stat().st_size,
        }

    def _tree_asset(self, kind: str, files: list[Path]) -> dict:
        rows = []
        for path in sorted({item.resolve() for item in files if item.is_file() and not item.is_symlink()}):
            try:
                relative = path.relative_to(self.root).as_posix()
            except ValueError:
                continue
            rows.append({"path": relative, "sha256": sha256(path), "bytes": path.stat().st_size})
        if not rows:
            return {
                "kind": kind, "status": "missing", "path": None,
                "asset_id": f"{kind}:missing", "sha256": None, "bytes": None,
                "file_count": 0,
            }
        digest = _json_digest(rows)
        return {
            "kind": kind,
            "status": "available",
            "path": "aggregate",
            "asset_id": f"{kind}:{digest[:12]}",
            "sha256": digest,
            "bytes": sum(row["bytes"] for row in rows),
            "file_count": len(rows),
            "files": rows,
        }

    def _code_assets(self) -> list[dict]:
        runtime_files = [self.root / name for name in ("ontology.py", "run_v0.py")]
        runtime_files.extend((self.root / "src").rglob("*.py") if (self.root / "src").exists() else [])
        static_root = self.root / "src/operator_console/static"
        ui_files = []
        if static_root.exists():
            for suffix in ("*.html", "*.js", "*.css"):
                ui_files.extend(static_root.rglob(suffix))
        return [
            self._tree_asset("runtime_code", runtime_files),
            self._tree_asset("operator_ui", ui_files),
        ]

    def _configuration_assets(self, config: dict) -> list[dict]:
        calibration = deepcopy(config.get("calibration") or {})
        calibration_payload = {
            "calibration": calibration,
            "camera_device_id": (config.get("camera") or {}).get("device_id"),
        }
        calibration_digest = _json_digest(calibration_payload)

        table_path = self.root / 'operator_data' / 'table_surface.json'
        table = json.loads(table_path.read_text(encoding='utf-8')) if table_path.is_file() else {}
        motion_payload = {
            'table_surface': {key: table.get(key) for key in ('z_mm', 'confirmed', 'calibration_digest')},
            "controller_policy": "v2-fixed-wait-vertical-abc-v1",
            "workspace_mm": (config.get("grasp") or {}).get("workspace_mm"),
            "approach_clearance_mm": (config.get("grasp") or {}).get("approach_clearance_mm"),
            "lift_clearance_mm": (config.get("grasp") or {}).get("lift_clearance_mm"),
            "grasp_z_offset_mm": (config.get("grasp") or {}).get("grasp_z_offset_mm"),
            "target_z_offsets_mm": (config.get("grasp") or {}).get("target_z_offsets_mm", {}),
            "min_grasp_z_mm": (config.get("grasp") or {}).get("min_grasp_z_mm"),
            "table_z_mm": (config.get("grasp") or {}).get("table_z_mm"),
            "tcp_xy_offset_mm": (config.get("grasp") or {}).get("tcp_xy_offset_mm"),
            "rx_deg": (config.get("grasp") or {}).get("rx_deg"),
            "ry_deg": (config.get("grasp") or {}).get("ry_deg"),
            "orientation": config.get("orientation"),
            "gripper_wait_s": ((config.get("robot") or {}).get("gripper") or {}).get("wait_s"),
            "gripper": deepcopy((config.get("robot") or {}).get("gripper") or {}),
            "robot_verification": {
                key: (config.get("robot") or {}).get(key)
                for key in (
                    "pose_verify_position_tolerance_mm",
                    "pose_verify_orientation_tolerance_deg",
                    "pose_verify_stable_polls",
                    "start_angles_deg",
                    "start_angle_speed",
                    "coordinate_speed",
                    "place_joint_speed",
                    "motion_timeout_s",
                    "motion_poll_interval_s",
                    "coarse_step_mm",
                    "vertical_motion_mode",
                    "vertical_segment_step_mm",
                    "approach_coordinate_mode",
                    "lift_coordinate_mode",
                    "fallback_wait_s",
                )
            },
        }
        motion_digest = _json_digest(motion_payload)
        return [
            {
                "kind": "calibration_profile",
                "status": "available",
                "path": _relative(self.root, self.config_path) + "#calibration",
                "asset_id": f"calibration_profile:{calibration_digest[:12]}",
                "sha256": calibration_digest,
                "bytes": len(json.dumps(calibration_payload, ensure_ascii=False).encode("utf-8")),
                "summary": {
                    "ready_for_motion": calibration.get("ready_for_motion"),
                    "rmse_mm": calibration.get("rmse_mm"),
                    "max_rmse_mm": calibration.get("max_rmse_mm"),
                },
            },
            {
                "kind": "motion_safety_profile",
                "status": "available",
                "path": _relative(self.root, self.config_path) + "#grasp+orientation+robot-verification+table",
                "asset_id": f"motion_safety_profile:{motion_digest[:12]}",
                "sha256": motion_digest,
                "bytes": len(json.dumps(motion_payload, ensure_ascii=False).encode("utf-8")),
            },
        ]

    def _model_asset(self, config: dict) -> dict:
        value = (config.get("model") or {}).get("weights")
        if not value:
            return {
                "kind": "detector_weights",
                "status": "missing",
                "path": None,
                "asset_id": "detector_weights:missing",
                "sha256": None,
                "bytes": None,
            }
        path = Path(value)
        if not path.is_absolute():
            path = self.root / path
        relative = _relative(self.root, path)
        if not path.is_file() or path.is_symlink():
            return {
                "kind": "detector_weights",
                "status": "missing",
                "path": relative,
                "asset_id": "detector_weights:missing",
                "sha256": None,
                "bytes": None,
            }
        digest = sha256(path)
        return {
            "kind": "detector_weights",
            "status": "available",
            "path": relative,
            "asset_id": f"detector_weights:{digest[:12]}",
            "sha256": digest,
            "bytes": path.stat().st_size,
        }

    def current_snapshot(self) -> dict:
        config_path = self.config_path
        try:
            config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError):
            config = {}
        assets = [self._file_asset(kind, _relative(self.root, self.config_path) if kind == "runtime_config" else path)
                  for kind, path in self.FILE_ASSETS]
        if (self.root / "operator_data/table_surface.json").is_file():
            assets.append(self._file_asset("table_surface", "operator_data/table_surface.json"))
        captures_root = self.root / "operator_data/place_captures"
        if captures_root.is_dir():
            captures = [p for p in captures_root.rglob("*") if p.is_file()]
            if captures:
                assets.append(self._tree_asset("place_captures", captures))
        place_catalog = self.root / "operator_data/place_profiles.json"
        if place_catalog.is_file() and not place_catalog.is_symlink():
            assets.append(self._file_asset("place_profiles", "operator_data/place_profiles.json"))
        else:
            empty_place = {"schema_version": 1, "updated_at": None, "profiles": []}
            digest = _json_digest(empty_place)
            assets.append({
                "kind": "place_profiles", "status": "available",
                "path": "operator_data/place_profiles.json (implicit empty)",
                "asset_id": f"place_profiles:{digest[:12]}", "sha256": digest,
                "bytes": len(json.dumps(empty_place, ensure_ascii=False).encode("utf-8")),
                "profile_count": 0,
            })
        assets.append(self._model_asset(config))
        assets.extend(self._configuration_assets(config))
        assets.extend(self._code_assets())
        assets = sorted(assets, key=lambda item: item["kind"])
        identity = {
            item["kind"]: {
                "asset_id": item["asset_id"],
                "sha256": item.get("sha256"),
                "status": item["status"],
            }
            for item in assets
        }
        snapshot_digest = _json_digest(identity)
        return {
            "schema_version": SNAPSHOT_SCHEMA_VERSION,
            "snapshot_id": f"assets:{snapshot_digest[:16]}",
            "generated_at": _utc_now(),
            "root": str(self.root),
            "assets": assets,
            "complete": all(item["status"] == "available" for item in assets),
        }

    def _empty_catalog(self) -> dict:
        return {
            "schema_version": REGISTRY_SCHEMA_VERSION,
            "updated_at": None,
            "active_snapshot_id": None,
            "snapshots": [],
        }

    def _load_catalog(self) -> dict:
        if not self.path.is_file() or self.path.is_symlink():
            return self._empty_catalog()
        data = json.loads(self.path.read_text(encoding="utf-8"))
        if data.get("schema_version") != REGISTRY_SCHEMA_VERSION or not isinstance(data.get("snapshots"), list):
            raise ValueError("Unsupported asset registry schema")
        return data

    def register_current(self) -> dict:
        snapshot = self.current_snapshot()
        with self._lock:
            catalog = self._load_catalog()
            if not any(item.get("snapshot_id") == snapshot["snapshot_id"] for item in catalog["snapshots"]):
                catalog["snapshots"].append(snapshot)
                # Bound growth while preserving enough history for handoff/replay.
                catalog["snapshots"] = catalog["snapshots"][-100:]
            catalog["active_snapshot_id"] = snapshot["snapshot_id"]
            catalog["updated_at"] = _utc_now()
            self.state_root.mkdir(parents=True, exist_ok=True)
            atomic_json(self.path, catalog)
        return snapshot

    def catalog(self) -> dict:
        snapshot = self.register_current()
        with self._lock:
            catalog = self._load_catalog()
        return {
            **catalog,
            "active": snapshot,
            "registry_path": _relative(self.root, self.path),
        }
