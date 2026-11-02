#!/usr/bin/env python3
"""MVP 手動 Eye-to-Hand 兩階段點位收集工具。

目標：建立同一個固定世界點在 Camera frame 與 Robot base frame 的 3D 對應。

操作方式：
1. Robot/TCP 先移開，讓 RealSense 清楚看到固定標定點。
2. 在 RGB 視窗點選該固定點，按 C，只暫存 Camera XYZ。
3. 手動把 Robot TCP 移到「同一個實體點」，盡量保持固定、近似垂直向下的末端姿態。
4. 按 R，讀取/輸入 Robot XYZ；此時才把 Camera + Robot pair 寫入 CSV。

快捷鍵：
- 左鍵：選擇 Camera 標定點
- C：Capture Camera point（不寫 CSV）
- R：Capture Robot point 並完成一組 pair
- X：取消目前 pending Camera point
- Q / Esc：離開

這是 MVP 校正工具，不做自動棋盤格、不做自動移動，也不會開啟抓取 motion。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import argparse
import csv
import json
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from v0_mvp.camera import RealSenseCamera
from v0_mvp.robot import MyCobotController
from v0_mvp.calibration_reference import FixedContactReference, prepare_reference_metadata

WINDOW_NAME = "Calibration: click point; C=camera, R=robot, X=cancel, Q=quit"


@dataclass
class PendingCameraPoint:
    uv: tuple[int, int]
    point_cam_m: np.ndarray
    depth_m: float
    valid_pixels: int


class TwoStagePairing:
    """Small state machine: Camera capture first, Robot capture second."""

    def __init__(self):
        self.pending: PendingCameraPoint | None = None
        self.saved_pairs = 0

    def set_camera(self, measurement: PendingCameraPoint) -> None:
        self.pending = measurement

    def cancel_camera(self) -> bool:
        had_pending = self.pending is not None
        self.pending = None
        return had_pending

    def pair_robot(self, point_robot_mm) -> list[float]:
        if self.pending is None:
            raise RuntimeError("尚未 Capture Camera point；請先點選固定標定點並按 C")
        robot = [float(x) for x in point_robot_mm]
        if len(robot) != 3 or not np.isfinite(robot).all():
            raise ValueError("Robot point 必須是 X Y Z 三個值")
        row = [
            *[float(x) for x in self.pending.point_cam_m],
            *robot,
        ]
        self.pending = None
        self.saved_pairs += 1
        return row


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=str(ROOT / "calibration/calibration_points.csv"))
    p.add_argument("--read-robot", action="store_true", help="用 MyCobot get_coords() 自動讀 Robot XYZ")
    p.add_argument(
        "--free-mode",
        action="store_true",
        help="連線後釋放 MyCobot 伺服器，允許手動移動 TCP；必須搭配 --read-robot",
    )
    p.add_argument("--reference", help="已量測的固定角點 JSON；省略時維持原 TCP 收集模式")
    return p


def sample_camera_point(frame, cam, uv: tuple[int, int], roi_radius: int = 4) -> PendingCameraPoint:
    """Read robust depth around a clicked pixel and deproject to Camera XYZ."""
    u, v = uv
    r = int(roi_radius)
    roi = frame.depth_m[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
    vals = roi[np.isfinite(roi) & (roi > 0.05) & (roi < 2.0)]
    if vals.size < 5:
        raise ValueError("Depth 無效，請重新點選")
    z = float(np.median(vals))
    p_cam = np.asarray(cam.deproject(u, v, z, frame.intrinsics), dtype=float)
    if p_cam.shape != (3,) or not np.all(np.isfinite(p_cam)):
        raise ValueError(f"Camera deprojection 回傳異常: {p_cam}")
    return PendingCameraPoint(
        uv=(int(u), int(v)),
        point_cam_m=p_cam,
        depth_m=z,
        valid_pixels=int(vals.size),
    )


def _read_robot_xyz(robot: MyCobotController | None, *, full_pose=False) -> tuple[list[float], list[float] | None]:
    if robot is not None:
        coords = robot.get_coords()
        return [float(x) for x in coords[:3]], coords

    prompt = "X Y Z RX RY RZ (mm/deg)" if full_pose else "X Y Z (mm)"
    raw = input(f"輸入 Robot {prompt}，用空白分隔: ").strip().split()
    if len(raw) != (6 if full_pose else 3):
        raise ValueError("格式錯誤，請輸入完整座標")
    values = list(map(float, raw))
    return values[:3], values if full_pose else None


def _draw_status(vis, selected_uv, pairing: TwoStagePairing, contact=False) -> None:
    if selected_uv is not None:
        cv2.circle(vis, selected_uv, 6, (0, 255, 255), 2)

    if pairing.pending is None:
        status = "STEP 1: click fixed point -> C (camera)"
    else:
        u, v = pairing.pending.uv
        cv2.circle(vis, (u, v), 10, (0, 255, 0), 2)
        status = "STEP 2: align " + ("CONTACT" if contact else "TCP") + " to SAME point -> R; X=cancel"

    cv2.putText(
        vis,
        status,
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        vis,
        f"saved pairs: {pairing.saved_pairs}",
        (12, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 0),
        1,
        cv2.LINE_AA,
    )


def main(argv=None) -> int:
    p = build_parser()
    a = p.parse_args(argv)
    if a.free_mode and not a.read_robot:
        p.error("--free-mode 必須搭配 --read-robot")

    reference = FixedContactReference.load(a.reference) if a.reference else None
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    prepare_reference_metadata(out, reference)
    if reference:
        print("角點模式:", reference.metadata())
        input("請確認實際開口、角點、控制器座標原點和量測設定一致；按 Enter 繼續，Ctrl+C 取消。")

    cfg = yaml.safe_load((ROOT / "config/v0.yaml").read_text(encoding="utf-8"))
    ccfg = cfg["camera"]
    cam = RealSenseCamera(
        ccfg.get("device_id"),
        ccfg["width"],
        ccfg["height"],
        ccfg["fps"],
        ccfg.get("frame_timeout_ms", 5000),
    )

    robot = None
    if a.read_robot:
        robot = MyCobotController(cfg["robot"])
        robot.connect()
        if a.free_mode:
            print("WARNING: 即將釋放 MyCobot 所有伺服器；手臂可能因重力下墜。")
            print("請先用手支撐機械臂/TCP，再按 Enter；Ctrl+C 可取消。")
            input()
            robot.release_all_servos()
            print("Robot free mode enabled.")
            print("建議整個 calibration 過程讓夾爪/TCP 保持近似垂直向下、姿態盡量一致。")

    new_file = not out.exists() or out.stat().st_size == 0
    f = out.open("a", encoding="utf-8", newline="")
    writer = csv.writer(f)
    if new_file:
        writer.writerow([
            "camera_x_m", "camera_y_m", "camera_z_m",
            "robot_x_mm", "robot_y_mm", "robot_z_mm",
        ])

    selected = {"uv": None}
    pairing = TwoStagePairing()

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            selected["uv"] = (x, y)

    try:
        cam.connect()
        cam.warmup(10)
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, on_mouse)
        print("Two-stage calibration ready:")
        print("  1) Robot/TCP 移開，滑鼠點固定世界點，按 C 保存 Camera XYZ。")
        print("  2) 將指定角點" if reference else "  2) 將 TCP", "對準同一實體點，按 R 保存補償後 Robot XYZ。")
        print("  X=取消 pending Camera point；Q/Esc=離開。")

        while True:
            frame = cam.capture()
            vis = frame.color_bgr.copy()
            _draw_status(vis, selected["uv"], pairing, contact=reference is not None)
            cv2.imshow(WINDOW_NAME, vis)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord("x"):
                if pairing.cancel_camera():
                    print("Pending Camera point cancelled.")
                else:
                    print("目前沒有 pending Camera point。")
                continue

            if key == ord("c"):
                if selected["uv"] is None:
                    print("尚未選點：請先在 RGB 視窗用滑鼠左鍵點固定標定點。")
                    continue
                try:
                    measurement = sample_camera_point(frame, cam, selected["uv"])
                except ValueError as exc:
                    print(str(exc))
                    continue

                replacing = pairing.pending is not None
                pairing.set_camera(measurement)
                prefix = "Camera point REPLACED" if replacing else "Camera point captured"
                print(
                    f"{prefix}: pixel={measurement.uv} "
                    f"depth={measurement.depth_m * 1000.0:.2f} mm "
                    f"valid_pixels={measurement.valid_pixels}"
                )
                print("Camera XYZ (m):", measurement.point_cam_m)
                print("現在將", reference.name if reference else "TCP", "對準同一個固定實體點，保持固定姿態/開口，按 R。")
                continue

            if key == ord("r"):
                if pairing.pending is None:
                    print("尚未有 Camera point：請先點選固定標定點並按 C。")
                    continue
                try:
                    p_robot, coords = _read_robot_xyz(robot, full_pose=reference is not None)
                    raw_robot = list(p_robot)
                    if reference:
                        p_robot = reference.contact_xyz(coords).tolist()
                    if not np.isfinite(p_robot).all():
                        raise ValueError("Robot XYZ 必須是有限數值")
                except ValueError as exc:
                    print(str(exc))
                    continue

                if coords is not None:
                    print("Robot current coords:", coords)
                    if len(coords) >= 6:
                        print("Robot orientation (deg):", coords[3:6], "(建議各點盡量一致/近似垂直向下)")

                pending = pairing.pending
                row = pairing.pair_robot(p_robot)
                writer.writerow(row)
                f.flush()
                with out.with_suffix(out.suffix + ".samples.jsonl").open("a", encoding="utf-8") as audit:
                    audit.write(json.dumps({"camera_xyz_m": pending.point_cam_m.tolist(),
                        "pixel_uv": pending.uv, "valid_pixels": pending.valid_pixels,
                        "reported_xyz_mm": raw_robot, "reported_pose": coords,
                        "contact_xyz_mm": p_robot}, ensure_ascii=False) + "\n")
                print(f"PAIR #{pairing.saved_pairs} SAVED")
                print("Camera(m)=", pending.point_cam_m, "Robot(mm)=", p_robot)
                print("--- 下一點：先把 Robot/TCP 移開，再點新的固定世界點並按 C ---")
                selected["uv"] = None
                continue

    finally:
        f.close()
        cam.disconnect()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
