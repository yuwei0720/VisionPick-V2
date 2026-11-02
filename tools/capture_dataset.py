#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


def parse_args():
    p = argparse.ArgumentParser(description="RealSense dataset capture tool for YOLO11-OBB.")
    p.add_argument("--session", required=True, help="Session name, e.g. normal_light_01")
    p.add_argument("--output", default="datasets/raw", help="Root output directory")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--save-depth", action="store_true")
    p.add_argument("--device", default=None, help="Optional RealSense serial number")
    p.add_argument("--warmup", type=int, default=30)
    return p.parse_args()


def clean_session_name(name: str) -> str:
    bad = '<>:"/\\|?*'
    name = name.strip()
    for c in bad:
        name = name.replace(c, "_")
    if not name:
        raise ValueError("Session name cannot be empty.")
    return name


def next_index(rgb_dir: Path) -> int:
    nums = []
    for f in rgb_dir.glob("rgb_*.jpg"):
        try:
            nums.append(int(f.stem.split("_")[1]))
        except Exception:
            pass
    return max(nums, default=0) + 1


def draw_overlay(frame, session, count, fps_value, save_depth):
    vis = frame.copy()
    lines = [
        f"Session: {session}",
        f"Captured: {count}",
        f"Live FPS: {fps_value:.1f}",
        f"Depth save: {'ON' if save_depth else 'OFF'}",
        "[SPACE] Capture    [Q / ESC] Quit",
    ]
    y = 28
    for line in lines:
        cv2.putText(vis, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 0), 2, cv2.LINE_AA)
        y += 28
    return vis


def main():
    args = parse_args()
    session = clean_session_name(args.session)

    session_dir = Path(args.output) / session
    rgb_dir = session_dir / "rgb"
    depth_npy_dir = session_dir / "depth_npy"
    depth_png_dir = session_dir / "depth_png"

    rgb_dir.mkdir(parents=True, exist_ok=True)
    if args.save_depth:
        depth_npy_dir.mkdir(parents=True, exist_ok=True)
        depth_png_dir.mkdir(parents=True, exist_ok=True)

    idx = next_index(rgb_dir)
    count = len(list(rgb_dir.glob("rgb_*.jpg")))

    pipeline = rs.pipeline()
    config = rs.config()
    if args.device:
        config.enable_device(args.device)

    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

    profile = None

    try:
        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        sensor = profile.get_device().first_depth_sensor()
        depth_scale = float(sensor.get_depth_scale())

        try:
            serial = profile.get_device().get_info(rs.camera_info.serial_number)
        except Exception:
            serial = "unknown"

        session_meta = {
            "session": session,
            "created_unix": time.time(),
            "camera_serial": serial,
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
            "depth_scale_m_per_unit": depth_scale,
            "save_depth": bool(args.save_depth),
        }
        (session_dir / "session.json").write_text(
            json.dumps(session_meta, indent=2, ensure_ascii=False),
            encoding="utf-8"
        )

        print(f"Session: {session}")
        print(f"Output: {session_dir.resolve()}")
        print(f"Camera serial: {serial}")
        print(f"Depth scale: {depth_scale} m/unit")
        print("SPACE = capture, Q/ESC = quit")

        for _ in range(max(0, args.warmup)):
            align.process(pipeline.wait_for_frames())

        last = time.perf_counter()
        fps_smooth = 0.0

        while True:
            frames = align.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data())

            now = time.perf_counter()
            inst_fps = 1.0 / max(now - last, 1e-6)
            last = now
            fps_smooth = inst_fps if fps_smooth == 0 else 0.9 * fps_smooth + 0.1 * inst_fps

            vis = draw_overlay(color, session, count, fps_smooth, args.save_depth)
            cv2.imshow("RealSense Dataset Capture", vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break

            if key == 32:
                stem = f"{idx:06d}"
                rgb_path = rgb_dir / f"rgb_{stem}.jpg"

                if not cv2.imwrite(str(rgb_path), color, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                    print(f"[ERROR] Failed to save {rgb_path}")
                    continue

                if args.save_depth:
                    np.save(depth_npy_dir / f"depth_{stem}.npy", depth_raw)
                    cv2.imwrite(str(depth_png_dir / f"depth_{stem}.png"), depth_raw)

                h, w = depth_raw.shape
                center_depth_m = float(depth_raw[h // 2, w // 2]) * depth_scale
                count += 1
                idx += 1
                print(f"[CAPTURED #{count}] {rgb_path.name} | center depth: {center_depth_m:.3f} m")

    except RuntimeError as e:
        print("[RealSense ERROR]", e)
        raise SystemExit(1)
    finally:
        if profile is not None:
            pipeline.stop()
        cv2.destroyAllWindows()

    print(f"Done. Total images in session: {count}")
    print(f"Saved at: {session_dir.resolve()}")


if __name__ == "__main__":
    main()
