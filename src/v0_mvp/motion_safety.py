"""Shared calibration gate; independent of task models and CLI."""
import math
from .geometry import as_transform


def validate_motion_unlock(cfg):
    c = cfg["calibration"]
    if c.get("ready_for_motion", False) is not True:
        raise RuntimeError("拒絕實機動作：calibration.ready_for_motion=false")
    rmse = c.get("rmse_mm")
    if rmse is None:
        raise RuntimeError("拒絕實機動作：calibration.rmse_mm 尚未設定")
    limit = c.get("max_rmse_mm", 10.0)
    try:
        valid = not isinstance(rmse, bool) and not isinstance(limit, bool)
        valid = valid and math.isfinite(float(rmse)) and math.isfinite(float(limit))
        valid = valid and float(rmse) >= 0 and float(limit) > 0
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        raise RuntimeError("拒絕實機動作：Calibration RMSE 與門檻必須為有效有限數值")
    if float(rmse) > float(limit):
        raise RuntimeError(f"拒絕實機動作：Calibration RMSE={rmse} mm 超過門檻")
    as_transform(c["T_robot_from_camera"])
