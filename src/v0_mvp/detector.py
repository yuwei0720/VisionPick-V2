from __future__ import annotations
from pathlib import Path
import cv2
import numpy as np
from .geometry import long_axis_angle_deg, obb_center
from .types import OBBDetection


class DetectorError(RuntimeError):
    pass


def _to_numpy(x):
    if hasattr(x, "detach"):
        x = x.detach()
    if hasattr(x, "cpu"):
        x = x.cpu()
    if hasattr(x, "numpy"):
        x = x.numpy()
    return np.asarray(x)


def parse_ultralytics_obb_result(result) -> list[OBBDetection]:
    obb = getattr(result, "obb", None)
    if obb is None or len(obb) == 0:
        return []
    corners_all = _to_numpy(obb.xyxyxyxy)
    cls_all = _to_numpy(obb.cls).astype(int)
    conf_all = _to_numpy(obb.conf).astype(float)
    out = []
    for corners, class_id, confidence in zip(corners_all, cls_all, conf_all):
        pts = np.asarray(corners, dtype=float).reshape(4, 2)
        out.append(OBBDetection(
            class_id=int(class_id),
            class_name=str(result.names[int(class_id)]),
            confidence=float(confidence),
            corners_px=pts,
            center_uv=obb_center(pts),
            long_axis_angle_deg=long_axis_angle_deg(pts),
        ))
    return out


class YOLO11OBBDetector:
    def __init__(self, weights, *, imgsz=640, confidence=0.35, iou=0.5, device=None):
        weights = Path(weights)
        if not weights.exists():
            raise DetectorError(f"找不到 YOLO11-OBB 權重: {weights}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise DetectorError("未安裝 ultralytics，請執行 pip install -r requirements.txt") from exc
        self.model = YOLO(str(weights))
        self.imgsz = int(imgsz)
        self.confidence = float(confidence)
        self.iou = float(iou)
        self.device = None if device in (None, "", "auto") else device

    @property
    def names(self):
        return self.model.names

    def detect(self, bgr: np.ndarray):
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise DetectorError(f"YOLO 輸入需為 HxWx3，收到 {bgr.shape}")
        kwargs = dict(source=bgr, imgsz=self.imgsz, conf=self.confidence, iou=self.iou, verbose=False)
        if self.device is not None:
            kwargs["device"] = self.device
        results = self.model.predict(**kwargs)
        if not results:
            return [], None
        return parse_ultralytics_obb_result(results[0]), results[0]


def canonical_target(target: str, aliases: dict | None = None) -> str:
    aliases = aliases or {}
    raw = str(target).strip()
    if raw in aliases:
        return str(aliases[raw]).strip()
    lower = {str(k).strip().lower(): str(v).strip() for k, v in aliases.items()}
    return lower.get(raw.lower(), raw)


def select_target(detections: list[OBBDetection], target: str, aliases: dict | None = None) -> OBBDetection:
    canonical = canonical_target(target, aliases).lower()
    matches = [d for d in detections if d.class_name.strip().lower() == canonical]
    if not matches:
        seen = sorted({d.class_name for d in detections})
        raise DetectorError(f"找不到目標 '{target}'（解析為 '{canonical}'）；目前偵測到: {seen or '無'}")
    return max(matches, key=lambda d: d.confidence)


def annotate(image_bgr, detections, selected=None, *, out_of_workspace_indices=()):
    out = image_bgr.copy()
    rejected = set(out_of_workspace_indices)
    for index, d in enumerate(detections):
        pts = np.rint(d.corners_px).astype(np.int32).reshape(-1, 1, 2)
        chosen = d is selected
        outside = index in rejected
        color = (0, 0, 255) if outside else (0, 255, 255) if chosen else (0, 200, 0)
        cv2.polylines(out, [pts], True, color, 3 if chosen or outside else 2)
        u, v = map(int, map(round, d.center_uv))
        cv2.circle(out, (u, v), 4, color, -1)
        text = (f"{d.class_name}: TARGET OUT OF WORKSPACE" if outside
                else f"{d.class_name} {d.confidence:.2f} axis={d.long_axis_angle_deg:.1f}")
        cv2.putText(out, text, (u + 5, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return out
