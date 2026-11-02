#!/usr/bin/env python3
"""
V0 RealSense + YOLO11-OBB live inference.

Runtime configuration is loaded from:
    config/live_yolo_obb.yaml

The Python file intentionally avoids hard-coded runtime parameters.
Only the default config-file location is defined here.

Run:
    python tools/live_yolo_obb_realsense.py

Optional alternate config:
    python tools/live_yolo_obb_realsense.py --config config/live_yolo_obb.yaml
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import torch
import yaml
from ultralytics import YOLO

from obb_depth_geometry import (
    deproject_pixel_to_camera,
    estimate_robust_depth,
    obb_center_px,
    obb_long_axis_angle_deg,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "live_yolo_obb.yaml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RealSense + YOLO11-OBB live inference"
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Path to runtime YAML config.",
    )
    return parser.parse_args()


def load_config(path: Path) -> dict:
    if not path.exists():
        raise SystemExit(f"Missing config file: {path}")

    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if not isinstance(cfg, dict):
        raise SystemExit(f"Config must be a YAML mapping: {path}")

    required_sections = {"camera", "depth", "model", "display", "controls"}
    missing = sorted(required_sections - set(cfg))
    if missing:
        raise SystemExit(
            f"Missing config section(s): {', '.join(missing)}"
        )

    return cfg


def resolve_project_path(value: str) -> Path:
    path = Path(str(value))
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    return path.resolve()


def resolve_device(value):
    """
    Convert YAML device setting into a value accepted by Ultralytics.
    """
    if value is None:
        value = "auto"

    if isinstance(value, int):
        if value >= 0 and not torch.cuda.is_available():
            raise SystemExit(
                f"Config requests CUDA device {value}, "
                "but torch.cuda.is_available() is False."
            )
        return value

    text = str(value).strip().lower()

    if text == "auto":
        return 0 if torch.cuda.is_available() else "cpu"

    if text == "cpu":
        return "cpu"

    try:
        index = int(text)
    except ValueError:
        return value

    if index >= 0 and not torch.cuda.is_available():
        raise SystemExit(
            f"Config requests CUDA device {index}, "
            "but torch.cuda.is_available() is False."
        )

    return index


def build_realsense_config(cfg: dict) -> rs.config:
    camera_cfg = cfg["camera"]
    depth_cfg = cfg["depth"]

    width = int(camera_cfg["width"])
    height = int(camera_cfg["height"])
    fps = int(camera_cfg["fps"])

    rs_config = rs.config()

    serial = camera_cfg.get("device_serial")
    if serial:
        rs_config.enable_device(str(serial))

    if bool(depth_cfg.get("enabled", True)):
        rs_config.enable_stream(
            rs.stream.depth,
            width,
            height,
            rs.format.z16,
            fps,
        )

    rs_config.enable_stream(
        rs.stream.color,
        width,
        height,
        rs.format.bgr8,
        fps,
    )

    return rs_config


def should_quit(key: int, configured_keys: list) -> bool:
    for item in configured_keys:
        text = str(item).strip().lower()

        if text == "esc" and key == 27:
            return True

        if len(text) == 1 and key == ord(text):
            return True

    return False


def get_center_depth_m(
    depth_frame,
    center_x: int,
    center_y: int,
) -> float | None:
    if depth_frame is None:
        return None

    width = depth_frame.get_width()
    height = depth_frame.get_height()

    if not (0 <= center_x < width and 0 <= center_y < height):
        return None

    depth_m = float(depth_frame.get_distance(center_x, center_y))

    if depth_m <= 0:
        return None

    return depth_m


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).resolve()
    cfg = load_config(config_path)

    camera_cfg = cfg["camera"]
    depth_cfg = cfg["depth"]
    model_cfg = cfg["model"]
    display_cfg = cfg["display"]
    controls_cfg = cfg["controls"]

    if (
        bool(depth_cfg.get("enabled", True))
        and bool(display_cfg.get("show_localization", True))
        and not bool(depth_cfg.get("align_to_color", True))
    ):
        raise SystemExit(
            "display.show_localization requires depth.align_to_color=true."
        )

    model_path = resolve_project_path(model_cfg["path"])

    if not model_path.exists():
        raise SystemExit(
            f"Model not found: {model_path}\n"
            "Place the trained model at models/best.pt "
            "or update model.path in config/live_yolo_obb.yaml."
        )

    device = resolve_device(model_cfg.get("device", "auto"))

    print("=" * 72)
    print("V0 RealSense + YOLO11-OBB Live Inference")
    print("=" * 72)
    print(f"Project root : {PROJECT_ROOT}")
    print(f"Config       : {config_path}")
    print(f"Model        : {model_path}")
    print(f"Device       : {device}")

    if torch.cuda.is_available():
        print(f"CUDA         : {torch.version.cuda}")
        print(f"GPU          : {torch.cuda.get_device_name(0)}")
    else:
        print("CUDA         : unavailable")

    print(
        f"Camera       : "
        f"{camera_cfg['width']}x{camera_cfg['height']} "
        f"@ {camera_cfg['fps']} FPS"
    )
    print(
        f"Depth        : "
        f"{'ON' if depth_cfg.get('enabled', True) else 'OFF'}"
    )
    print("=" * 72)

    model = YOLO(str(model_path))

    pipeline = rs.pipeline()
    rs_config = build_realsense_config(cfg)
    profile = None

    align = None
    if (
        bool(depth_cfg.get("enabled", True))
        and bool(depth_cfg.get("align_to_color", True))
    ):
        align = rs.align(rs.stream.color)

    try:
        profile = pipeline.start(rs_config)

        depth_scale = None
        if bool(depth_cfg.get("enabled", True)):
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale = float(depth_sensor.get_depth_scale())
            print(f"Depth scale  : {depth_scale:.9f} m/unit")

        print(
            f"Warming up camera for "
            f"{int(camera_cfg.get('warmup_frames', 0))} frame(s)..."
        )

        for _ in range(max(0, int(camera_cfg.get("warmup_frames", 0)))):
            frames = pipeline.wait_for_frames()
            if align is not None:
                align.process(frames)

        print("Ready. Press Q or ESC to quit.")

        last_time = time.perf_counter()
        fps_smooth = 0.0

        while True:
            frames = pipeline.wait_for_frames()

            if align is not None:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = (
                frames.get_depth_frame()
                if bool(depth_cfg.get("enabled", True))
                else None
            )

            if not color_frame:
                continue

            if bool(depth_cfg.get("enabled", True)) and not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())

            depth_image_m = None
            depth_intrinsics = None
            if depth_frame is not None and depth_scale is not None:
                depth_image_m = (
                    np.asanyarray(depth_frame.get_data()).astype(np.float32)
                    * depth_scale
                )
                depth_intrinsics = (
                    depth_frame.profile.as_video_stream_profile()
                    .get_intrinsics()
                )

            results = model.predict(
                source=frame,
                imgsz=int(model_cfg["imgsz"]),
                conf=float(model_cfg["confidence"]),
                iou=float(model_cfg.get("iou", 0.70)),
                max_det=int(model_cfg.get("max_det", 100)),
                device=device,
                verbose=bool(model_cfg.get("verbose", False)),
            )

            result = results[0]
            display_frame = result.plot()

            if result.obb is not None and len(result.obb) > 0:
                corners_all = result.obb.xyxyxyxy.cpu().numpy()

                class_ids = result.obb.cls.cpu().numpy().astype(int)
                confidences = result.obb.conf.cpu().numpy()

                for corners, class_id, confidence in zip(
                    corners_all,
                    class_ids,
                    confidences,
                ):
                    center_x, center_y = obb_center_px(corners)

                    if bool(display_cfg.get("show_center", True)):
                        cv2.circle(
                            display_frame,
                            (center_x, center_y),
                            int(display_cfg.get("center_radius", 5)),
                            (0, 0, 255),
                            -1,
                        )

                    class_name = result.names[int(class_id)]
                    labels = [f"{class_name} {float(confidence):.2f}"]

                    if bool(
                        display_cfg.get(
                            "show_center_coordinate",
                            True,
                        )
                    ):
                        labels.append(f"({center_x},{center_y})")

                    if bool(
                        display_cfg.get(
                            "show_center_depth",
                            False,
                        )
                    ):
                        depth_m = get_center_depth_m(
                            depth_frame,
                            center_x,
                            center_y,
                        )
                        if depth_m is not None:
                            labels.append(f"{depth_m:.3f}m")
                        else:
                            labels.append("depth:N/A")

                    if bool(
                        display_cfg.get(
                            "show_localization",
                            True,
                        )
                    ):
                        angle_deg = obb_long_axis_angle_deg(corners)
                        labels.append(f"angle={angle_deg:.1f}deg")

                        if depth_image_m is None or depth_intrinsics is None:
                            labels.append("localization:N/A depth_disabled")
                        else:
                            depth_result = estimate_robust_depth(
                                depth_image_m,
                                corners,
                                min_m=float(depth_cfg.get("min_m", 0.10)),
                                max_m=float(depth_cfg.get("max_m", 1.50)),
                                roi_scale=float(
                                    depth_cfg.get("roi_scale", 0.55)
                                ),
                                min_valid_pixels=int(
                                    depth_cfg.get("min_valid_pixels", 20)
                                ),
                                min_valid_ratio=float(
                                    depth_cfg.get("min_valid_ratio", 0.30)
                                ),
                                mad_multiplier=float(
                                    depth_cfg.get("mad_multiplier", 3.5)
                                ),
                            )

                            if depth_result.ok:
                                camera_xyz = deproject_pixel_to_camera(
                                    (center_x, center_y),
                                    float(depth_result.depth_m),
                                    depth_intrinsics,
                                    rs.rs2_deproject_pixel_to_point,
                                )
                                labels.append(
                                    f"Z={depth_result.depth_m:.3f}m "
                                    f"valid={depth_result.valid_ratio:.0%}"
                                )
                                labels.append(
                                    "XYZ=("
                                    f"{camera_xyz[0]:.3f},"
                                    f"{camera_xyz[1]:.3f},"
                                    f"{camera_xyz[2]:.3f})m"
                                )
                            else:
                                labels.append(
                                    "depth:N/A "
                                    f"{depth_result.status} "
                                    f"valid={depth_result.valid_ratio:.0%}"
                                )

                    if labels:
                        for line_no, label in enumerate(labels):
                            cv2.putText(
                                display_frame,
                                label,
                                (
                                    center_x + 8,
                                    center_y - 8 + line_no * 16,
                                ),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.45,
                                (0, 0, 255),
                                1,
                                cv2.LINE_AA,
                            )

            now = time.perf_counter()
            dt = max(now - last_time, 1e-6)
            last_time = now

            instant_fps = 1.0 / dt
            fps_smooth = (
                instant_fps
                if fps_smooth == 0.0
                else 0.9 * fps_smooth + 0.1 * instant_fps
            )

            if bool(display_cfg.get("show_fps", True)):
                cv2.putText(
                    display_frame,
                    f"FPS: {fps_smooth:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            cv2.imshow(
                str(
                    display_cfg.get(
                        "window_name",
                        "V0 - RealSense YOLO11-OBB",
                    )
                ),
                display_frame,
            )

            key = cv2.waitKey(1) & 0xFF

            if should_quit(
                key,
                controls_cfg.get("quit_keys", ["q", "esc"]),
            ):
                break

    except RuntimeError as exc:
        raise SystemExit(
            "RealSense runtime error:\n"
            f"{exc}\n\n"
            "Check USB connection, camera availability, "
            "resolution/FPS support, and device_serial."
        ) from exc

    finally:
        if profile is not None:
            pipeline.stop()

        cv2.destroyAllWindows()

    print("Camera stopped.")


if __name__ == "__main__":
    main()
