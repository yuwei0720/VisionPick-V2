from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import shutil


def _indent(line: str) -> int:
    return len(line) - len(line.lstrip(" "))


def update_config(path: Path) -> Path:
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()

    section: str | None = None
    subsection: str | None = None
    found = {
        "lift_clearance_mm": False,
        "coordinate_mode": False,
        "pregrasp_coordinate_mode": False,
        "approach_coordinate_mode": False,
        "lift_coordinate_mode": False,
        "start_angle_speed": False,
        "gripper_speed": False,
        "open_speed": False,
        "close_speed": False,
        "open_state": False,
        "close_state": False,
        "pose_verify_stable_polls": False,
    }

    output: list[str] = []
    gripper_speed_output_index: int | None = None
    robot_orientation_tol_output_index: int | None = None

    for line in lines:
        stripped = line.strip()
        indent = _indent(line)

        if stripped and not stripped.startswith("#") and indent == 0 and stripped.endswith(":"):
            section = stripped[:-1]
            subsection = None
        elif section == "robot" and indent == 2 and stripped.endswith(":"):
            subsection = stripped[:-1]
        elif indent <= 2 and section == "robot" and subsection == "gripper" and stripped:
            subsection = None

        replacement = line
        if section == "grasp" and indent == 2 and stripped.startswith("lift_clearance_mm:"):
            replacement = "  lift_clearance_mm: 60.0"
            found["lift_clearance_mm"] = True
        elif section == "robot" and indent == 2 and stripped.startswith("coordinate_mode:"):
            replacement = "  coordinate_mode: 0"
            found["coordinate_mode"] = True
        elif section == "robot" and indent == 2 and stripped.startswith("pregrasp_coordinate_mode:"):
            replacement = "  pregrasp_coordinate_mode: 0"
            found["pregrasp_coordinate_mode"] = True
        elif section == "robot" and indent == 2 and stripped.startswith("approach_coordinate_mode:"):
            replacement = "  approach_coordinate_mode: 1"
            found["approach_coordinate_mode"] = True
        elif section == "robot" and indent == 2 and stripped.startswith("lift_coordinate_mode:"):
            replacement = "  lift_coordinate_mode: 1"
            found["lift_coordinate_mode"] = True
        elif section == "robot" and indent == 2 and stripped.startswith("start_angle_speed:"):
            replacement = "  start_angle_speed: 15"
            found["start_angle_speed"] = True
        elif section == "robot" and indent == 2 and stripped.startswith("pose_verify_stable_polls:"):
            replacement = "  pose_verify_stable_polls: 3"
            found["pose_verify_stable_polls"] = True
        elif section == "robot" and indent == 2 and stripped.startswith("pose_verify_orientation_tolerance_deg:"):
            robot_orientation_tol_output_index = len(output)
        elif section == "robot" and subsection == "gripper" and indent == 4:
            if stripped.startswith("speed:"):
                replacement = "    speed: 80"
                found["gripper_speed"] = True
                gripper_speed_output_index = len(output)
            elif stripped.startswith("open_speed:"):
                replacement = "    open_speed: 50"
                found["open_speed"] = True
            elif stripped.startswith("close_speed:"):
                replacement = "    close_speed: 80"
                found["close_speed"] = True
            elif stripped.startswith("open_state:"):
                replacement = "    open_state: 0"
                found["open_state"] = True
            elif stripped.startswith("close_state:"):
                replacement = "    close_state: 1"
                found["close_state"] = True

        output.append(replacement)

    required_existing = [
        "lift_clearance_mm",
        "coordinate_mode",
        "pregrasp_coordinate_mode",
        "approach_coordinate_mode",
        "lift_coordinate_mode",
        "start_angle_speed",
        "gripper_speed",
        "open_state",
        "close_state",
    ]
    missing = [name for name in required_existing if not found[name]]
    if missing:
        raise RuntimeError(
            "config 結構與預期不符，未修改。缺少欄位: " + ", ".join(missing)
        )

    if not found["pose_verify_stable_polls"]:
        if robot_orientation_tol_output_index is None:
            raise RuntimeError(
                "config 缺少 pose_verify_orientation_tolerance_deg，無法安全插入 stable polls"
            )
        output.insert(robot_orientation_tol_output_index + 1, "  pose_verify_stable_polls: 3")
        if gripper_speed_output_index is not None and gripper_speed_output_index > robot_orientation_tol_output_index:
            gripper_speed_output_index += 1

    if not found["open_speed"] or not found["close_speed"]:
        if gripper_speed_output_index is None:
            raise RuntimeError("config 缺少 robot.gripper.speed，無法插入開閉速度")
        additions: list[str] = []
        if not found["open_speed"]:
            additions.append("    open_speed: 50")
        if not found["close_speed"]:
            additions.append("    close_speed: 80")
        for offset, value in enumerate(additions, start=1):
            output.insert(gripper_speed_output_index + offset, value)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_name(f"{path.name}.bak_v6_{timestamp}")
    shutil.copy2(path, backup)
    path.write_text("\n".join(output) + "\n", encoding="utf-8")
    return backup


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Apply EdgeAI Cartesian motion v6 settings without replacing calibration data."
    )
    parser.add_argument("--config", default="config/v0.yaml")
    args = parser.parse_args()

    path = Path(args.config).resolve()
    if not path.is_file():
        raise SystemExit(f"找不到 config: {path}")
    backup = update_config(path)
    print(f"UPDATED: {path}")
    print(f"BACKUP:  {backup}")
    print("SET: lift_clearance_mm=60.0, coordinate_mode=0, start_angle_speed=15")
    print("SET: gripper open=(state 0, speed 50), close=(state 1, speed 80)")
    print("SET: pregrasp mode=0, approach mode=1, lift mode=1, stable polls=3")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
