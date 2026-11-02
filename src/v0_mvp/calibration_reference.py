"""Convert a visible contact point to base coordinates before calibration fit.

Fixed-base offsets deliberately require a fixed reported orientation. No Euler
rotation convention or controller tool-reference configuration is assumed.
"""
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np


def vector3(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise ValueError(f"{name} 必須是三個有限數值")
    return a


@dataclass(frozen=True)
class FixedContactReference:
    name: str
    offset_base_mm: tuple
    reference_rpy_deg: tuple
    max_axis_delta_deg: float
    gripper_state: str
    reported_origin: str

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if data.get("mode") != "fixed_base_contact" or data.get("measured") is not True:
            raise ValueError("角點設定必須 mode=fixed_base_contact 且 measured=true；請先實測偏移")
        offset = vector3(data.get("offset_base_mm"), "offset_base_mm")
        rpy = vector3(data.get("reference_rpy_deg"), "reference_rpy_deg")
        tol = float(data.get("max_axis_delta_deg", 1.0))
        if not np.isfinite(tol) or not 0 < tol <= 5:
            raise ValueError("max_axis_delta_deg 必須 >0 且 <=5")
        for key in ("name", "gripper_state", "reported_origin"):
            if not isinstance(data.get(key), str) or not data[key].strip():
                raise ValueError(f"請記錄 {key}")
        if data["reported_origin"] != "grasp_tcp":
            raise ValueError("請先確認 get_coords/send_coords 使用抓取中心 TCP，然後設 reported_origin=grasp_tcp；法蘭座標不可直接當 TCP")
        return cls(data["name"], tuple(offset), tuple(rpy), tol,
                   data["gripper_state"], data["reported_origin"])

    def contact_xyz(self, coords):
        a = np.asarray(coords, dtype=float)
        if a.shape != (6,) or not np.isfinite(a).all():
            raise ValueError("角點模式需要完整且有限的 X Y Z RX RY RZ")
        # Conservative per-component guard, with wraparound at +/-180 degrees.
        delta = np.abs((a[3:] - self.reference_rpy_deg + 180.) % 360. - 180.)
        if np.any(delta > self.max_axis_delta_deg):
            raise ValueError(f"角點姿態改變：各軸差={delta.tolist()} deg，請恢復固定姿態")
        # Offset is reported origin -> physical contact, expressed in BASE axes.
        return a[:3] + np.asarray(self.offset_base_mm)

    def metadata(self):
        return {"mode": "fixed_base_contact", "name": self.name,
                "offset_base_mm": list(self.offset_base_mm),
                "reference_rpy_deg": list(self.reference_rpy_deg),
                "max_axis_delta_deg": self.max_axis_delta_deg,
                "gripper_state": self.gripper_state,
                "reported_origin": self.reported_origin}


def prepare_reference_metadata(csv_path, reference):
    """Refuse mixing old TCP pairs with contact pairs or different offsets."""
    csv_path = Path(csv_path)
    sidecar = csv_path.with_suffix(csv_path.suffix + ".reference.json")
    metadata = reference.metadata() if reference else {"mode": "reported_origin"}
    if csv_path.exists() and csv_path.stat().st_size:
        if sidecar.exists():
            if json.loads(sidecar.read_text(encoding="utf-8")) != metadata:
                raise ValueError("CSV 的角點/姿態設定不同，請使用新的 --out 檔案")
        elif reference is not None:
            raise ValueError("舊 CSV 沒有角點記錄，請使用新的 --out 檔案，避免混用資料")
    sidecar.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
