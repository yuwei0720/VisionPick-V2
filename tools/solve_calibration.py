#!/usr/bin/env python3
"""Fit calibration and optionally evaluate a separately collected holdout CSV."""
from pathlib import Path
import argparse
import sys
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from v0_mvp.calibration import load_point_csv, solve_rigid_transform, CalibrationError


def evaluate_transform(T, A, B):
    if A.ndim != 2 or A.shape != B.shape or A.shape[1:] != (3,) or not len(A):
        raise CalibrationError("驗證資料必須為非空 Nx3 對應點")
    if not np.isfinite(A).all() or not np.isfinite(B).all():
        raise CalibrationError("驗證資料包含非有限數值")
    residual = ((T[:3, :3] @ A.T).T + T[:3, 3] - B) * 1000.
    errors = np.linalg.norm(residual, axis=1)
    return errors, residual


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("csv", help="camera_xyz(m) 與同一實體點 robot_xyz(mm) CSV")
    p.add_argument("--holdout", help="另外採集、未參與擬合的驗證 CSV")
    p.add_argument("--config", default=str(ROOT / "config/v0.yaml"))
    p.add_argument("--update-config", action="store_true", help="寫入矩陣/RMSE 並取消 ready_for_motion")
    a = p.parse_args(argv)
    A, B = load_point_csv(a.csv)
    T, rmse, errors = solve_rigid_transform(A, B)
    print("T_robot_from_camera =")
    for row in T:
        print("  ", [round(float(x), 9) for x in row])
    print(f"Fit RMSE = {rmse:.3f} mm; max = {float(np.max(errors)):.3f} mm")
    print("Per-point error (mm):", [round(float(x), 3) for x in errors])
    if a.holdout:
        if Path(a.csv).resolve() == Path(a.holdout).resolve():
            p.error("holdout 不可與擬合 CSV 相同")
        H, J = load_point_csv(a.holdout)
        if np.array_equal(A, H) and np.array_equal(B, J):
            p.error("holdout 不可複製擬合資料")
        held_errors, residual = evaluate_transform(T, H, J)
        print(f"Holdout RMSE = {np.sqrt(np.mean(held_errors ** 2)):.3f} mm; "
              f"max = {np.max(held_errors):.3f} mm; n = {len(H)}")
        print("Holdout mean XYZ bias (mm):", np.mean(residual, axis=0).round(3).tolist())
        print("Holdout per-point error (mm):", held_errors.round(3).tolist())
        print("請依物件尺寸與抓取容差評估；指定 holdout 不會自動開放運動。")
    else:
        print("尚無獨立 holdout；fit RMSE 不代表實際抓取精度。")
    if a.update_config:
        path = Path(a.config)
        cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
        cfg["calibration"]["T_robot_from_camera"] = T.tolist()
        cfg["calibration"]["rmse_mm"] = float(rmse)
        cfg["calibration"]["ready_for_motion"] = False
        path.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
        print("Updated:", path)
        print("ready_for_motion=false；完成獨立/實機驗證後再啟用。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
