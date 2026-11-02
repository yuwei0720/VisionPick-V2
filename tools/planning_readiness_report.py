#!/usr/bin/env python3
"""Print VisionPick 0.9 static collision-validation readiness without contacting hardware."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from operator_console.planning_status import planning_readiness


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="print the full machine-readable status")
    args = parser.parse_args()
    status = planning_readiness(ROOT)
    if args.json:
        print(json.dumps(status, ensure_ascii=False, indent=2))
    else:
        print(f"Planning readiness: {status['status']}")
        print(status["summary"])
        print(f"Robot: {status['robot_model'].get('manufacturer')} {status['robot_model'].get('model')}")
        print(f"End effector: {status['robot_model'].get('end_effector')}")
        print(f"Scene: {status['scene'].get('planning_frame')} / RealSense {status['scene'].get('realsense_mounting')}")
        print(f"Guard enforcement: {status['guard_enforcement']} | trajectory execution: {status['trajectory_execution']}")
        for item in status.get("blocking_reasons", []):
            print(f"- [{item.get('component')}] {item.get('reason')}")
        print(status["safety_note"])
    return 0 if status["status"] == "READY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
