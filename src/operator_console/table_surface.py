"""Operator-confirmed horizontal table height, bound to the active calibration."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from .events import atomic_json, utc_now


def calibration_digest(cfg):
    value = {"matrix": cfg["calibration"]["T_robot_from_camera"],
             "camera_device_id": cfg.get("camera", {}).get("device_id")}
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


class TableSurfaceManager:
    def __init__(self, root):
        self.path = Path(root) / "operator_data" / "table_surface.json"

    def status(self, cfg):
        if not self.path.is_file():
            return {"status": "missing", "ready": False, "reason": "table_surface_not_configured"}
        try:
            item = json.loads(self.path.read_text(encoding="utf-8"))
            z = item["z_mm"]
            if type(z) not in (int, float) or not math.isfinite(z) or abs(z) > 1000 or item.get("confirmed") is not True:
                raise ValueError("invalid table measurement")
            if item.get("calibration_digest") != calibration_digest(cfg):
                return {**item, "status": "stale", "ready": False, "reason": "table_surface_calibration_changed"}
            return {**item, "status": "ready", "ready": True}
        except (OSError, ValueError, KeyError, TypeError):
            return {"status": "invalid", "ready": False, "reason": "table_surface_invalid"}

    def require(self, cfg):
        item = self.status(cfg)
        if not item["ready"]:
            raise ValueError(item["reason"])
        return item

    def save(self, cfg, *, z_mm, confirmed, method, note="", provenance=None):
        if confirmed is not True:
            raise ValueError("Confirm that the selected/measured surface is the empty horizontal table")
        if type(z_mm) not in (int, float) or not math.isfinite(z_mm) or abs(z_mm) > 1000:
            raise ValueError("Table Z must be finite and within -1000..1000 mm")
        if method not in {"manual_measurement", "camera_selection"}:
            raise ValueError("Unsupported table measurement method")
        if not isinstance(note, str) or len(note) > 500:
            raise ValueError("Table note must be a string of at most 500 characters")
        item = {"schema_version": 1, "z_mm": float(z_mm), "confirmed": True, "horizontal": True,
                "method": method, "note": note, "updated_at": utc_now(),
                "calibration_digest": calibration_digest(cfg), "provenance": provenance or {},
                "hardware_motion_validated": False}
        self.path.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(self.path, item)
        return self.status(cfg)
