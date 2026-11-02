#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from planning.errors import PlanningError
from planning.model_manifest import load_verified_robot_model_manifest, load_verified_scene_manifest


def main(argv=None):
    p = argparse.ArgumentParser(description="Fail-closed VisionPick 0.9 robot/scene manifest verifier")
    p.add_argument("--robot-model", default=str(ROOT / "planning/robot_model/robot_model_manifest.yaml"))
    p.add_argument("--scene", default=str(ROOT / "planning/scene/scene_manifest.yaml"))
    args = p.parse_args(argv)
    out = {"status": "BLOCKED", "robot_model": None, "scene": None, "errors": []}
    try:
        m = load_verified_robot_model_manifest(args.robot_model)
        out["robot_model"] = {"version": m.version, "hash": m.manifest_hash, "path": str(m.path)}
    except PlanningError as exc:
        out["errors"].append({"component": "robot_model", "code": exc.status.value, "message": str(exc), "details": exc.details})
    try:
        s = load_verified_scene_manifest(args.scene)
        out["scene"] = {"version": s.version, "hash": s.manifest_hash, "path": str(s.path)}
    except PlanningError as exc:
        out["errors"].append({"component": "scene", "code": exc.status.value, "message": str(exc), "details": exc.details})
    if not out["errors"]:
        out["status"] = "PASS"
    print(json.dumps(out, ensure_ascii=False, indent=2))
    return 0 if out["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
