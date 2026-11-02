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

from planning.hardware_facts import capture_mycobot280_m5_read_only_facts, pymycobot_version


def _connect(port: str, baud: int):
    # Prefer the model-specific API when available, while preserving compatibility
    # with the legacy API currently used by VisionPick 0.8.2.
    try:
        from pymycobot.mycobot280 import MyCobot280
        return MyCobot280(port, baud), "pymycobot.mycobot280.MyCobot280"
    except (ImportError, ModuleNotFoundError):
        from pymycobot.mycobot import MyCobot
        return MyCobot(port, baud), "pymycobot.mycobot.MyCobot"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Read-only hardware facts capture for VisionPick myCobot 280 M5 Gate Review A")
    p.add_argument("--port", default="COM3")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default=str(ROOT / "planning/robot_model/hardware_facts_mycobot280_m5.json"))
    args = p.parse_args(argv)

    try:
        backend, backend_class = _connect(str(args.port), int(args.baud))
        facts = capture_mycobot280_m5_read_only_facts(
            backend, port=str(args.port), baud=int(args.baud), library_version=pymycobot_version()
        )
        facts["backend_class"] = backend_class
        facts["capture_status"] = "PASS_READ_ONLY"
    except Exception as exc:
        facts = {
            "schema_version": 1,
            "kind": "visionpick_hardware_facts_capture",
            "manufacturer": "Elephant Robotics",
            "exact_model": "myCobot 280 M5",
            "generation": "M5",
            "transport": {"port": str(args.port), "baud": int(args.baud)},
            "pymycobot_version": pymycobot_version(),
            "capture_status": "FAILED",
            "motion_commands_issued": False,
            "error": f"{type(exc).__name__}: {exc}",
        }

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(facts, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(facts, ensure_ascii=False, indent=2))
    return 0 if facts.get("capture_status") == "PASS_READ_ONLY" else 2


if __name__ == "__main__":
    raise SystemExit(main())
