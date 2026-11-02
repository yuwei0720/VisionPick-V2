"""Manual RealSense place-point capture and deterministic pixel/depth conversion.

This module deliberately does not infer a destination or orientation. The
operator chooses a point/ROI in a frozen RGB frame. Median valid depth is
computed inside that ROI, deprojected with the saved RealSense intrinsics, and
transformed by the active camera->robot calibration. The UI may apply an
explicit Z clearance; RX/RY/RZ remain operator-provided.
"""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import json
import math
import secrets

import cv2
import numpy as np
import yaml

from v0_mvp.camera import RealSenseCamera, _import_rs
from v0_mvp.geometry import as_transform, transform_point


def _utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class VisualPlaceCaptureManager:
    def __init__(self, root: str | Path, config_path: str | Path, *, state_root: str | Path | None = None):
        self.root = Path(root).resolve()
        self.config_path = Path(config_path).resolve()
        self.state_root = Path(state_root or self.root / "operator_data" / "place_captures").resolve()
        self.state_root.mkdir(parents=True, exist_ok=True)

    def _config(self):
        data = yaml.safe_load(self.config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(data, dict):
            raise ValueError("config root must be an object")
        return data

    def persist(self, frame, cfg, *, source='current_rgbd_frame'):
        """Freeze aligned RGB/depth/intrinsics; no task-model or robot dependency."""
        color = np.asarray(frame.color_bgr)
        depth = np.asarray(frame.depth_m, dtype=np.float32)
        if color.ndim != 3 or depth.shape != color.shape[:2]:
            raise ValueError('aligned_rgbd_frame_required')
        capture_id = 'pcap_' + secrets.token_hex(8)
        path = self._path(capture_id)
        path.mkdir(parents=True, exist_ok=False)
        try:
            if not cv2.imwrite(str(path / 'frame.png'), color):
                raise OSError('Could not persist RGB frame')
            np.save(path / 'depth_m.npy', depth, allow_pickle=False)
            meta = {'schema_version': 1, 'capture_id': capture_id, 'captured_at': _utc_now(),
                    'width': color.shape[1], 'height': color.shape[0],
                    'intrinsics': self._intrinsics_dict(frame.intrinsics), 'source': source,
                    'camera_device_id': cfg.get('camera', {}).get('device_id'),
                    'calibration_matrix': cfg.get('calibration', {}).get('T_robot_from_camera'),
                    'calibration_rmse_mm': cfg.get('calibration', {}).get('rmse_mm'),
                    'workspace_mm': cfg.get('grasp', {}).get('workspace_mm'),
                    'depth_policy': cfg.get('depth', {})}
            (path / 'metadata.json').write_text(json.dumps(meta, ensure_ascii=False, allow_nan=False), encoding='utf-8')
            return {**meta, 'frame_url': f'/api/v2/places/captures/{capture_id}/frame'}
        except Exception:
            for item in path.iterdir():
                item.unlink()
            path.rmdir()
            raise

    def _path(self, capture_id: str) -> Path:
        import re
        if not isinstance(capture_id, str) or not re.fullmatch(r'pcap_[0-9a-f]{16}', capture_id):
            raise ValueError('invalid_capture_id')
        path = self.state_root / capture_id
        if path.is_symlink() or self.state_root.is_symlink():
            raise ValueError('capture_symlink_forbidden')
        return path

    @staticmethod
    def _intrinsics_dict(intr):
        model_name = str(getattr(intr, "model", "none")).split(".")[-1]
        return {
            "width": int(intr.width), "height": int(intr.height),
            "ppx": float(intr.ppx), "ppy": float(intr.ppy),
            "fx": float(intr.fx), "fy": float(intr.fy),
            "model": model_name,
            "coeffs": [float(x) for x in list(intr.coeffs)],
        }

    def capture(self, *, warmup_frames: int | None = None) -> dict:
        cfg = self._config()
        c = cfg.get('camera') or {}
        camera = RealSenseCamera(c.get('device_id'), c.get('width', 640), c.get('height', 480),
                                c.get('fps', 30), c.get('frame_timeout_ms', 5000))
        try:
            camera.connect()
            camera.warmup(c.get('warmup_frames', 15) if warmup_frames is None else warmup_frames)
            return self.persist(camera.capture(), cfg, source='new_rgbd_frame')
        finally:
            camera.disconnect()

    def _prune(self, *, keep: int):
        entries = [p for p in self.state_root.glob("pcap_*") if p.is_dir()]
        entries.sort(key=lambda p: p.stat().st_mtime, reverse=True)
        for path in entries[keep:]:
            try:
                for item in path.glob("*"):
                    item.unlink(missing_ok=True)
                path.rmdir()
            except OSError:
                pass

    def frame_path(self, capture_id: str) -> Path:
        path = self._path(capture_id) / "frame.png"
        if not path.is_file() or path.is_symlink():
            raise FileNotFoundError("Place capture frame not found")
        return path

    @staticmethod
    def _deproject(intr_data: dict, u: float, v: float, depth_m: float) -> np.ndarray:
        if intr_data.get('model', 'none') == 'none':
            fx, fy = float(intr_data['fx']), float(intr_data['fy'])
            if not all(math.isfinite(x) and x > 0 for x in (fx, fy)):
                raise ValueError('invalid_camera_intrinsics')
            return np.asarray([(u-float(intr_data['ppx']))*depth_m/fx,
                               (v-float(intr_data['ppy']))*depth_m/fy, depth_m], dtype=float)
        rs = _import_rs()
        intr = rs.intrinsics()
        intr.width = int(intr_data["width"]); intr.height = int(intr_data["height"])
        intr.ppx = float(intr_data["ppx"]); intr.ppy = float(intr_data["ppy"])
        intr.fx = float(intr_data["fx"]); intr.fy = float(intr_data["fy"])
        model_name = str(intr_data.get("model", "none"))
        model = getattr(rs.distortion, model_name, None)
        if model is None:
            raise ValueError(f"Unsupported saved RealSense distortion model: {model_name}")
        intr.model = model
        intr.coeffs = [float(x) for x in intr_data.get("coeffs", [0, 0, 0, 0, 0])]
        point = rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], float(depth_m))
        return np.asarray(point, dtype=float)

    def resolve(self, payload: dict) -> dict:
        allowed = {"capture_id", "u0", "v0", "u1", "v1", "z_offset_mm"}
        if not isinstance(payload, dict) or set(payload) - allowed:
            raise ValueError("Invalid visual-place selection fields")
        capture_id = payload.get("capture_id")
        path = self._path(capture_id)
        meta_path = path / "metadata.json"
        depth_path = path / "depth_m.npy"
        if meta_path.is_symlink() or depth_path.is_symlink():
            raise ValueError("capture_symlink_forbidden")
        if not meta_path.is_file() or not depth_path.is_file():
            raise FileNotFoundError("Place capture is missing or expired")
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        depth = np.load(depth_path, allow_pickle=False)
        h, w = depth.shape[:2]

        values = []
        for key in ("u0", "v0", "u1", "v1"):
            value = payload.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                raise ValueError(f"{key} must be finite")
            values.append(float(value))
        u0, v0, u1, v1 = values
        left, right = sorted((u0, u1)); top, bottom = sorted((v0, v1))
        # A click is expanded to an 11x11 ROI so isolated depth holes do not
        # turn a valid manual selection into a false failure.
        if right - left < 2:
            left -= 5; right += 5
        if bottom - top < 2:
            top -= 5; bottom += 5
        x0 = max(0, min(w - 1, int(math.floor(left))))
        x1 = max(x0 + 1, min(w, int(math.ceil(right)) + 1))
        y0 = max(0, min(h - 1, int(math.floor(top))))
        y1 = max(y0 + 1, min(h, int(math.ceil(bottom)) + 1))
        roi = np.asarray(depth[y0:y1, x0:x1], dtype=float)
        dcfg = meta.get("depth_policy") or {}
        min_m = float(dcfg.get("min_m", 0.05)); max_m = float(dcfg.get("max_m", 2.0))
        valid = roi[np.isfinite(roi) & (roi >= min_m) & (roi <= max_m)]
        min_valid = max(5, min(int(dcfg.get("min_valid_pixels", 20)), roi.size))
        if valid.size < min_valid:
            raise ValueError(f"有效 depth pixel 不足: {valid.size} < {min_valid}")
        depth_m = float(np.median(valid))
        mad_m = float(np.median(np.abs(valid - depth_m)))
        u = (x0 + x1 - 1) / 2.0; v = (y0 + y1 - 1) / 2.0
        point_cam = self._deproject(meta["intrinsics"], u, v, depth_m)
        T = as_transform(meta.get("calibration_matrix"))
        point_robot_m = transform_point(T, point_cam)
        surface_mm = [float(x) * 1000.0 for x in point_robot_m]
        z_offset = payload.get("z_offset_mm", 0.0)
        if isinstance(z_offset, bool) or not isinstance(z_offset, (int, float)) or not math.isfinite(float(z_offset)):
            raise ValueError("z_offset_mm must be finite")
        z_offset = float(z_offset)
        if not 0 <= z_offset <= 150:
            raise ValueError("z_offset_mm must be within 0–150 mm")
        place_xyz = [surface_mm[0], surface_mm[1], surface_mm[2] + z_offset]
        workspace = meta.get("workspace_mm") or {}
        violations = []
        for axis, value in zip(("x", "y", "z"), place_xyz):
            bounds = workspace.get(axis)
            if isinstance(bounds, list) and len(bounds) == 2:
                lo, hi = map(float, bounds)
                if not lo <= value <= hi:
                    violations.append({"axis": axis, "actual_mm": value, "allowed_mm": [lo, hi]})
        result = {
            "schema_version": 1,
            "capture_id": capture_id,
            "selected_roi_px": [x0, y0, x1 - 1, y1 - 1],
            "center_uv": [u, v],
            "depth_m": depth_m,
            "depth_valid_pixels": int(valid.size),
            "depth_total_pixels": int(roi.size),
            "depth_valid_ratio": float(valid.size / roi.size),
            "depth_mad_m": mad_m,
            "camera_xyz_m": [float(x) for x in point_cam],
            "robot_surface_xyz_mm": surface_mm,
            "z_offset_mm": z_offset,
            "place_xyz_mm": place_xyz,
            "workspace_ok": not violations,
            "workspace_violations": violations,
            "calibration_rmse_mm": meta.get("calibration_rmse_mm"),
            "orientation_source": "operator_required",
            "limitations": [
                "Manual visual selection provides XYZ only; RX/RY/RZ are not inferred.",
                "Selected depth represents the visible surface, not automatic object/container clearance.",
                "No collision, support-surface, container-interior, or free-space estimation is performed.",
            ],
        }
        return result
