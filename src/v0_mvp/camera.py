from __future__ import annotations
import logging
from typing import Any
import cv2
import numpy as np
from .types import FrameBundle

logger = logging.getLogger(__name__)


class RealSenseError(RuntimeError):
    pass


def _import_rs():
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise RealSenseError("未安裝 pyrealsense2，請執行 pip install -r requirements.txt") from exc
    return rs


class RealSenseCamera:
    def __init__(self, device_id=None, width=640, height=480, fps=30, timeout_ms=5000):
        self.device_id = None if device_id in (None, "", "null") else str(device_id)
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.timeout_ms = int(timeout_ms)
        self.pipeline: Any = None
        self.align: Any = None
        self.depth_scale: float | None = None
        self._rs: Any = None

    def connect(self):
        rs = _import_rs()
        self._rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.device_id:
            config.enable_device(self.device_id)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        # rgb8 在學長的 RealSense 程式已實際使用；此處再轉 BGR 給 OpenCV/Ultralytics。
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        try:
            profile = self.pipeline.start(config)
        except Exception as exc:
            self.pipeline = None
            raise RealSenseError(f"RealSense 啟動失敗: {exc}") from exc
        self.depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
        self.align = rs.align(rs.stream.color)
        logger.info("RealSense connected: %dx%d @ %d FPS", self.width, self.height, self.fps)

    def warmup(self, frames=15):
        self._require_connected()
        for _ in range(max(0, int(frames))):
            self.pipeline.wait_for_frames(self.timeout_ms)

    def _require_connected(self):
        if self.pipeline is None or self.align is None or self.depth_scale is None:
            raise RealSenseError("Camera 尚未 connect()")

    def capture(self) -> FrameBundle:
        self._require_connected()
        try:
            frames = self.pipeline.wait_for_frames(self.timeout_ms)
            aligned = self.align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
        except Exception as exc:
            raise RealSenseError(f"RealSense 取 frame 失敗: {exc}") from exc
        if not color_frame or not depth_frame:
            raise RealSenseError("RealSense 回傳不完整的 RGB-D frame")
        color_rgb = np.asanyarray(color_frame.get_data())
        color_bgr = cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR)
        depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * float(self.depth_scale)
        intrinsics = depth_frame.profile.as_video_stream_profile().get_intrinsics()
        if color_bgr.shape[:2] != depth_m.shape[:2]:
            raise RealSenseError(f"對齊後 RGB/Depth 尺寸不一致: {color_bgr.shape} vs {depth_m.shape}")
        return FrameBundle(color_bgr=color_bgr, depth_m=depth_m, intrinsics=intrinsics)

    def deproject(self, u: float, v: float, depth_m: float, intrinsics) -> np.ndarray:
        if not np.isfinite(depth_m) or depth_m <= 0:
            raise RealSenseError(f"無效 depth: {depth_m}")
        if self._rs is None:
            self._rs = _import_rs()
        p = self._rs.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], float(depth_m))
        return np.asarray(p, dtype=float)

    def disconnect(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            finally:
                self.pipeline = None
                self.align = None
                self.depth_scale = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.disconnect()
        return False
