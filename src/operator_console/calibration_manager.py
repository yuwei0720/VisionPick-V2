"""Non-motion calibration health analysis and candidate generation.

This module intentionally never activates a calibration or sets
``ready_for_motion``.  It analyzes existing point datasets and emits immutable
candidate files for later, explicit hardware validation.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import math

import numpy as np
import yaml

from .events import atomic_json, sha256
from v0_mvp.calibration import CalibrationError, load_point_csv, solve_rigid_transform


CANDIDATE_SCHEMA_VERSION = 1


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _canonical_json_value(value):
    if isinstance(value, dict):
        return {str(key): _canonical_json_value(item) for key, item in sorted(value.items(), key=lambda row: str(row[0]))}
    if isinstance(value, (list, tuple)):
        return [_canonical_json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"__nonfinite__": "NaN" if math.isnan(value) else ("Infinity" if value > 0 else "-Infinity")}
    return value


def _digest_json(value) -> str:
    raw = json.dumps(_canonical_json_value(value), sort_keys=True, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _percentile(values: np.ndarray, q: float) -> float | None:
    return float(np.percentile(values, q)) if values.size else None


def _rotation_delta_deg(a: np.ndarray, b: np.ndarray) -> float:
    delta = a @ b.T
    value = float(np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(np.arccos(value)))


def _matrix_health(matrix) -> dict:
    result = {
        "shape_valid": False,
        "finite": False,
        "homogeneous_row_valid": False,
        "rotation_orthogonality_error": None,
        "rotation_determinant": None,
        "status": "BLOCKED",
        "issues": [],
    }
    try:
        T = np.asarray(matrix, dtype=float)
    except (TypeError, ValueError):
        result["issues"].append("matrix_not_numeric")
        return result
    if T.shape != (4, 4):
        result["issues"].append(f"matrix_shape_{T.shape}")
        return result
    result["shape_valid"] = True
    result["finite"] = bool(np.isfinite(T).all())
    if not result["finite"]:
        result["issues"].append("matrix_nonfinite")
        return result
    result["homogeneous_row_valid"] = bool(np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8))
    if not result["homogeneous_row_valid"]:
        result["issues"].append("invalid_homogeneous_row")
    R = T[:3, :3]
    orth = float(np.linalg.norm(R.T @ R - np.eye(3), ord="fro"))
    det = float(np.linalg.det(R))
    result["rotation_orthogonality_error"] = orth
    result["rotation_determinant"] = det
    if orth > 1e-5:
        result["issues"].append("rotation_not_orthonormal")
    if abs(det - 1.0) > 1e-5:
        result["issues"].append("rotation_determinant_not_one")
    result["status"] = "PASS" if not result["issues"] else "BLOCKED"
    return result


class CalibrationManager:
    def __init__(self, root: str | Path, config_path: str | Path):
        self.root = Path(root).resolve()
        self.config_path = Path(config_path).resolve()
        self.data_root = self.root / "calibration"
        self.candidate_root = self.root / "operator_data" / "calibration_candidates"

    def _config(self) -> dict:
        data = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise ValueError("config root must be an object")
        return data

    def _dataset_path(self, name: str) -> Path:
        if (not isinstance(name, str) or not name.strip() or len(name) > 240
                or "\x00" in name or Path(name).name != name or Path(name).suffix.lower() != ".csv"):
            raise ValueError("Invalid calibration dataset name")
        path = self.data_root / name
        if path.is_symlink() or not path.is_file() or path.resolve().parent != self.data_root.resolve():
            raise FileNotFoundError("Calibration dataset not found")
        return path

    def _candidate_files(self) -> list[dict]:
        if not self.candidate_root.exists():
            return []
        out = []
        for path in self.candidate_root.glob("*.json"):
            if path.is_symlink():
                continue
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                patch_path = path.with_suffix(".yaml")
                yaml_snippet = ""
                if patch_path.is_file() and not patch_path.is_symlink():
                    yaml_snippet = patch_path.read_text(encoding="utf-8")
                elif data.get("matrix"):
                    yaml_snippet = yaml.safe_dump({
                        "calibration": {
                            "ready_for_motion": False,
                            "rmse_mm": data.get("metrics", {}).get("fit_rmse_mm"),
                            "T_robot_from_camera": data.get("matrix"),
                        },
                        "candidate_id": data.get("candidate_id"),
                    }, allow_unicode=True, sort_keys=False)
                out.append({
                    "candidate_id": data.get("candidate_id"),
                    "created_at": data.get("created_at"),
                    "dataset": data.get("source_dataset", {}).get("name"),
                    "status": data.get("status"),
                    "fit_rmse_mm": data.get("metrics", {}).get("fit_rmse_mm"),
                    "loo_rmse_mm": data.get("metrics", {}).get("leave_one_out", {}).get("rmse_mm"),
                    "path": path.relative_to(self.root).as_posix(),
                    "patch_path": patch_path.relative_to(self.root).as_posix() if patch_path.is_file() else None,
                    "yaml_snippet": yaml_snippet,
                })
            except (OSError, ValueError, TypeError):
                continue
        out.sort(key=lambda item: item.get("created_at") or "", reverse=True)
        return out[:100]

    def status(self) -> dict:
        cfg = self._config()
        calibration = cfg.get("calibration") or {}
        health = _matrix_health(calibration.get("T_robot_from_camera"))
        rmse = calibration.get("rmse_mm")
        maximum = calibration.get("max_rmse_mm")
        rmse_finite = isinstance(rmse, (int, float)) and math.isfinite(float(rmse)) and float(rmse) >= 0
        max_finite = isinstance(maximum, (int, float)) and math.isfinite(float(maximum)) and float(maximum) > 0
        issues = list(health["issues"])
        if not rmse_finite:
            issues.append("rmse_nonfinite_or_missing")
        if not max_finite:
            issues.append("max_rmse_nonfinite_or_missing")
        elif rmse_finite and float(rmse) > float(maximum):
            issues.append("rmse_above_limit")
        if calibration.get("ready_for_motion") is not True:
            issues.append("not_ready_for_motion")
        blocking = {"rmse_nonfinite_or_missing", "max_rmse_nonfinite_or_missing", "rmse_above_limit"}
        status = "BLOCKED" if health["status"] == "BLOCKED" or blocking.intersection(issues) else ("WARNING" if issues else "PASS")
        datasets = []
        if self.data_root.exists():
            for path in sorted(self.data_root.glob("*.csv")):
                if path.is_symlink():
                    continue
                rows = 0
                try:
                    with path.open("r", encoding="utf-8-sig") as stream:
                        rows = max(0, sum(1 for _ in stream) - 1)
                except OSError:
                    pass
                datasets.append({"name": path.name, "bytes": path.stat().st_size, "rows": rows, "sha256": sha256(path)})
        active_payload = {
            "T_robot_from_camera": calibration.get("T_robot_from_camera"),
            "rmse_mm": rmse,
            "max_rmse_mm": maximum,
            "ready_for_motion": calibration.get("ready_for_motion"),
        }
        digest = _digest_json(active_payload)
        return {
            "schema_version": 1,
            "status": status,
            "active_calibration_id": f"calibration:{digest[:12]}",
            "ready_for_motion": calibration.get("ready_for_motion") is True,
            "rmse_mm": float(rmse) if rmse_finite else None,
            "max_rmse_mm": float(maximum) if max_finite else None,
            "matrix_health": health,
            "issues": issues,
            "datasets": datasets,
            "candidates": self._candidate_files(),
            "policy": {
                "candidate_only": True,
                "automatic_activation": False,
                "motion_commands": False,
                "note": "UI analysis never sets ready_for_motion or overwrites config/v0.yaml.",
            },
        }

    def analyze(self, dataset_name: str) -> dict:
        path = self._dataset_path(dataset_name)
        cfg = self._config()
        A, B = load_point_csv(path)
        if A.shape[0] == 0 or not np.isfinite(A).all() or not np.isfinite(B).all():
            raise CalibrationError("Calibration dataset contains no finite points")
        T, rmse, errors = solve_rigid_transform(A, B)
        pred = (T[:3, :3] @ A.T).T + T[:3, 3]
        delta_mm = (pred - B) * 1000.0
        abs_axis = np.abs(delta_mm)
        loo_errors = []
        loo_failures = []
        if len(A) >= 4:
            for index in range(len(A)):
                mask = np.arange(len(A)) != index
                try:
                    T_i, _, _ = solve_rigid_transform(A[mask], B[mask])
                    predicted = T_i[:3, :3] @ A[index] + T_i[:3, 3]
                    loo_errors.append(float(np.linalg.norm(predicted - B[index]) * 1000.0))
                except CalibrationError as exc:
                    loo_failures.append({"index": index, "reason": str(exc)})
        loo = np.asarray(loo_errors, dtype=float)
        compare = None
        try:
            active = np.asarray((cfg.get("calibration") or {}).get("T_robot_from_camera"), dtype=float)
            if active.shape == (4, 4) and np.isfinite(active).all():
                compare = {
                    "translation_delta_mm": float(np.linalg.norm(T[:3, 3] - active[:3, 3]) * 1000.0),
                    "rotation_delta_deg": _rotation_delta_deg(T[:3, :3], active[:3, :3]),
                }
        except (TypeError, ValueError):
            compare = None
        cam_span = (A.max(axis=0) - A.min(axis=0)) * 1000.0
        robot_span = (B.max(axis=0) - B.min(axis=0)) * 1000.0
        raw_max_rmse = (cfg.get("calibration") or {}).get("max_rmse_mm")
        limit_valid = isinstance(raw_max_rmse, (int, float)) and math.isfinite(float(raw_max_rmse)) and float(raw_max_rmse) > 0
        max_rmse = float(raw_max_rmse) if limit_valid else None
        advisories = []
        if not limit_valid:
            advisories.append("active_max_rmse_limit_invalid")
        if len(A) < 6:
            advisories.append("fewer_than_6_points")
        if float(np.min(cam_span)) < 30.0:
            advisories.append("camera_coverage_narrow_on_at_least_one_axis")
        if len(loo) != len(A):
            advisories.append("leave_one_out_incomplete")
        if loo.size and max_rmse is not None and float(np.max(loo)) > max_rmse:
            advisories.append("leave_one_out_error_above_active_limit")
        validation_status = "PASS" if max_rmse is not None and rmse <= max_rmse and len(A) >= 3 else "BLOCKED"
        if validation_status == "PASS" and advisories:
            validation_status = "WARNING"
        dataset_digest = sha256(path)
        candidate_identity = hashlib.sha256((dataset_digest + json.dumps(T.tolist(), separators=(",", ":"))).encode("utf-8")).hexdigest()
        return {
            "schema_version": 1,
            "candidate_id": f"cal-candidate:{candidate_identity[:12]}",
            "status": validation_status,
            "source_dataset": {"name": path.name, "path": path.relative_to(self.root).as_posix(), "sha256": dataset_digest},
            "point_count": int(len(A)),
            "matrix": T.tolist(),
            "matrix_health": _matrix_health(T),
            "metrics": {
                "fit_rmse_mm": float(rmse),
                "max_error_mm": float(np.max(errors)),
                "mean_error_mm": float(np.mean(errors)),
                "p95_error_mm": _percentile(errors, 95),
                "per_axis_mean_abs_mm": {axis: float(value) for axis, value in zip("xyz", abs_axis.mean(axis=0))},
                "per_axis_max_abs_mm": {axis: float(value) for axis, value in zip("xyz", abs_axis.max(axis=0))},
                "leave_one_out": {
                    "available": bool(loo.size),
                    "valid_folds": int(loo.size),
                    "failed_folds": loo_failures,
                    "rmse_mm": float(np.sqrt(np.mean(loo ** 2))) if loo.size else None,
                    "max_error_mm": float(np.max(loo)) if loo.size else None,
                    "p95_error_mm": _percentile(loo, 95),
                },
            },
            "coverage": {
                "camera_span_mm": {axis: float(value) for axis, value in zip("xyz", cam_span)},
                "robot_span_mm": {axis: float(value) for axis, value in zip("xyz", robot_span)},
                "camera_center_m": {axis: float(value) for axis, value in zip("xyz", A.mean(axis=0))},
                "robot_center_mm": {axis: float(value * 1000.0) for axis, value in zip("xyz", B.mean(axis=0))},
            },
            "active_comparison": compare,
            "residuals": [
                {
                    "index": int(i),
                    "error_mm": float(errors[i]),
                    "delta_xyz_mm": [float(v) for v in delta_mm[i]],
                }
                for i in range(len(errors))
            ],
            "advisories": advisories,
            "thresholds": {"active_max_rmse_mm": max_rmse},
            "evidence_scope": "fit_and_leave_one_out_only; no independent holdout dataset was supplied",
            "ready_for_motion": False,
        }

    def create_candidate(self, dataset_name: str, notes: str = "") -> dict:
        if not isinstance(notes, str) or len(notes) > 2000:
            raise ValueError("notes must be a string up to 2000 characters")
        analysis = self.analyze(dataset_name)
        if analysis["status"] == "BLOCKED":
            raise ValueError("Blocked calibration analysis cannot become a candidate")
        record = {
            "schema_version": CANDIDATE_SCHEMA_VERSION,
            "candidate_id": analysis["candidate_id"],
            "created_at": _utc_now(),
            "status": "candidate_needs_hardware_validation",
            "notes": notes.strip(),
            "source_dataset": analysis["source_dataset"],
            "matrix": analysis["matrix"],
            "metrics": analysis["metrics"],
            "coverage": analysis["coverage"],
            "active_comparison": analysis["active_comparison"],
            "advisories": analysis["advisories"],
            "ready_for_motion": False,
            "activation_policy": "manual review + independent hardware validation required; this file does not modify config/v0.yaml",
        }
        self.candidate_root.mkdir(parents=True, exist_ok=True)
        filename = record["candidate_id"].replace(":", "_") + ".json"
        path = self.candidate_root / filename
        patch_path = path.with_suffix(".yaml")
        if path.is_symlink() or patch_path.is_symlink():
            raise ValueError("Calibration candidate symlink rejected")
        patch = {
            "calibration": {
                "ready_for_motion": False,
                "rmse_mm": analysis["metrics"]["fit_rmse_mm"],
                "T_robot_from_camera": analysis["matrix"],
            },
            "candidate_id": record["candidate_id"],
            "warning": "Candidate only. Do not set ready_for_motion until independent hardware validation is complete.",
        }
        if path.exists():
            # Content-addressed candidates are immutable and candidate creation is idempotent.
            existing = json.loads(path.read_text(encoding="utf-8"))
            if not patch_path.exists():
                temp = patch_path.with_suffix(".yaml.tmp")
                temp.write_text(yaml.safe_dump(patch, allow_unicode=True, sort_keys=False), encoding="utf-8")
                temp.replace(patch_path)
            return {**existing, "path": path.relative_to(self.root).as_posix(), "patch_path": patch_path.relative_to(self.root).as_posix(), "already_existed": True}
        temp = patch_path.with_suffix(".yaml.tmp")
        temp.write_text(yaml.safe_dump(patch, allow_unicode=True, sort_keys=False), encoding="utf-8")
        atomic_json(path, record)
        temp.replace(patch_path)
        return {**record, "path": path.relative_to(self.root).as_posix(), "patch_path": patch_path.relative_to(self.root).as_posix(), "already_existed": False}
