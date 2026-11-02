#!/usr/bin/env python3
from pathlib import Path
import importlib.util
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
sys.path.insert(0, str(SRC))
from v0_mvp.geometry import as_transform

cfg = yaml.safe_load((ROOT / "config/v0.yaml").read_text(encoding="utf-8"))
checks = []
for mod in ["numpy", "cv2", "yaml", "ultralytics", "pyrealsense2", "pymycobot"]:
    checks.append((f"import {mod}", importlib.util.find_spec(mod) is not None))
checks.append(("models/best.pt", (ROOT / cfg["model"]["weights"]).exists()))
try:
    as_transform(cfg["calibration"]["T_robot_from_camera"])
    checks.append(("calibration matrix 4x4 rigid", True))
except Exception:
    checks.append(("calibration matrix 4x4 rigid", False))
for name, ok in checks:
    print(f"[{'OK' if ok else 'MISSING'}] {name}")
print(f"[INFO] ready_for_motion={cfg['calibration'].get('ready_for_motion')}, rmse_mm={cfg['calibration'].get('rmse_mm')}")
raise SystemExit(0 if all(ok for _, ok in checks) else 1)
