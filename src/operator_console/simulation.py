"""Deterministic synthetic RGB-D fixture and fake SDK; real V6 controller is reused."""
from __future__ import annotations

from types import SimpleNamespace
import cv2
import numpy as np

from v0_mvp.types import FrameBundle, OBBDetection


def synthetic_scene(case):
    """Explicit S3 development fixture; never a RealSense/detector backend."""
    from v0_mvp.geometry import long_axis_angle_deg
    settings = case["frame"]
    w, h = settings["width"], settings["height"]
    intr = SimpleNamespace(**settings["intrinsics"])
    image = np.full((h, w, 3), 235, dtype=np.uint8)
    depth = np.full((h, w), settings["depth_m"], dtype=np.float32)
    detections = []
    corners_by_index = []
    for index, obj in enumerate(case["objects"]):
        corners = cv2.boxPoints((tuple(obj["center_uv"]), tuple(obj["size_px"]), obj["angle_deg"])).astype(float)
        corners_by_index.append(corners)
        detections.append(OBBDetection(index, obj["class_name"], obj["confidence"], corners,
                                       tuple(obj["center_uv"]), long_axis_angle_deg(corners)))
    draw_order = sorted(
        range(len(case["objects"])),
        key=lambda i: (-float(case["objects"][i].get("depth_m", settings["depth_m"])), i),
    )
    for index in draw_order:
        obj = case["objects"][index]
        corners = corners_by_index[index]
        cv2.fillConvexPoly(image, corners.astype(np.int32),
                          (175, 145, 105) if obj["class_name"] == "pen" else (150, 170, 200))
        if "depth_m" in obj:
            cv2.fillConvexPoly(depth, corners.astype(np.int32), float(obj["depth_m"]))
    object_depths_m = {}
    for index, obj in enumerate(case["objects"]):
        d_i = float(obj.get("depth_m", settings["depth_m"]))
        poly_i = corners_by_index[index].astype(np.float32)
        occluded = any(
            j != index
            and float(case["objects"][j].get("depth_m", settings["depth_m"])) < d_i - 1e-6
            and cv2.intersectConvexConvex(poly_i, corners_by_index[j].astype(np.float32))[0] > 0
            for j in range(len(case["objects"]))
        )
        if occluded:
            layer = np.full((h, w), settings["depth_m"], dtype=np.float32)
            if "depth_m" in obj:
                cv2.fillConvexPoly(layer, corners_by_index[index].astype(np.int32), float(obj["depth_m"]))
            object_depths_m[index] = layer
    intr.object_depths_m = object_depths_m

    def deproject(u, v, z):
        return np.array([(u - intr.ppx) * z / intr.fx, (v - intr.ppy) * z / intr.fy, z])

    def project(point):
        x, y, z = point
        return (x * intr.fx / z + intr.ppx, y * intr.fy / z + intr.ppy)

    return detections, FrameBundle(image, depth, intr), deproject, project


class FakeMyCobot:
    def __init__(self, scenario="success"):
        self.scenario = scenario
        self.calls = []
        self.angles = [0, 0, 0, -90, 0, -45]
        self.coords = [210, 0, 270, -175, -5, 0]
        self.sent = False
        self.stopped = False
        self._ik_map = {}

    def sync_send_angles(self, angles, speed, timeout=None):
        self.calls.append({"method": "sync_send_angles", "angles": list(angles), "speed": speed, "timeout": timeout})
        self.angles = list(angles)
        key = tuple(round(float(v), 3) for v in angles[:6])
        if key in self._ik_map:
            self.coords = list(self._ik_map[key])
        return 1

    def send_coords(self, coords, speed, mode):
        self.calls.append({"method": "send_coords", "coords": list(coords), "speed": speed, "mode": mode})
        self.sent = True
        if self.scenario != "timeout": self.coords = list(coords)
        return 1

    def set_gripper_state(self, state, speed):
        self.calls.append({"method": "set_gripper_state", "state": state, "speed": speed})

    def set_free_mode(self, enabled):
        self.calls.append({"method": "set_free_mode", "enabled": int(enabled)})
        return 1

    def focus_all_servos(self):
        self.calls.append({"method": "focus_all_servos"})
        return 1

    def release_all_servos(self):
        self.calls.append({"method": "release_all_servos"})
        return 1

    def stop(self):
        self.stopped = True
        self.calls.append({"method": "stop"})
        return 1

    def pause(self):
        self.stopped = True
        self.calls.append({"method": "pause"})
        return 1

    def solve_inv_kinematics(self, target_coords, current_angles=None):
        self.calls.append({"method": "solve_inv_kinematics", "coords": list(target_coords), "current_angles": list(current_angles or self.angles)})
        if self.scenario == "ik_no_solution":
            return None
        solution = [
            round(float(target_coords[0]) * 0.1, 3),
            round(float(target_coords[1]) * 0.1, 3),
            round(float(target_coords[2]) * 0.1, 3),
            -95.0, 5.0, -35.0,
        ]
        self._ik_map[tuple(round(float(v), 3) for v in solution)] = list(target_coords)
        return solution

    def get_angles(self): return list(self.angles)
    def get_coords(self): return list(self.coords)
    def get_error_information(self): return 32 if self.scenario == "stale_error" or (self.scenario == "timeout" and self.sent) else 0


class SimCamera:
    def __init__(self, *args): pass
    def connect(self): pass
    def disconnect(self): pass
    def warmup(self, frames): pass

    def capture(self):
        image = np.full((480, 640, 3), (222, 229, 228), dtype=np.uint8)
        for x in range(0, 640, 40): cv2.line(image, (x, 0), (x, 480), (213, 222, 220), 1)
        for y in range(0, 480, 40): cv2.line(image, (0, y), (640, y), (213, 222, 220), 1)
        cv2.rectangle(image, (45, 65), (600, 420), (177, 194, 187), 2)
        cv2.putText(image, "SYNTHETIC RGB-D / OFFLINE FIXTURE", (48, 43), cv2.FONT_HERSHEY_SIMPLEX, .55, (70, 90, 80), 1, cv2.LINE_AA)
        for name, center, size, angle, color in OBJECTS:
            box = cv2.boxPoints((center, size, angle)).astype(np.int32)
            cv2.fillConvexPoly(image, box + (4, 6), (174, 188, 181))
            cv2.fillConvexPoly(image, box, color)
            cv2.polylines(image, [box], True, (50, 65, 55), 2)
        cv2.putText(image, "+", (293, 266), cv2.FONT_HERSHEY_SIMPLEX, .6, (222, 232, 200), 2)
        return FrameBundle(image, np.full((480, 640), .38, dtype=np.float32), SimpleNamespace(width=640, height=480, fx=580., fy=580., ppx=320., ppy=240., model="pinhole_fixture", coeffs=[0., 0., 0., 0., 0.]))

    @staticmethod
    def deproject(u, v, z, intrinsics):
        return [(u - intrinsics.ppx) * z / intrinsics.fx, (v - intrinsics.ppy) * z / intrinsics.fy, z]


OBJECTS = [("battery", (300., 265.), (37., 83.), -17., (87, 109, 78)), ("pen", (422., 200.), (14., 139.), 27., (144, 100, 37)), ("screwdriver", (504., 335.), (27., 104.), -35., (55, 92, 213))]


# Additional objects are visible in both the synthetic image and detections.
OBJECTS += [
    ("correction tape", (355., 365.), (44., 65.), 10., (120, 140, 190)),
    ("eraser", (560., 130.), (35., 50.), 5., (180, 140, 160)),
    ("marker", (350., 130.), (22., 100.), 12., (120, 100, 160)),
    ("pencil lead case", (560., 240.), (26., 65.), -10., (130, 140, 100)),
]

class SimDetector:
    def __init__(self, scenario="success", scene_file=""):
        self.scenario = scenario
        self.scene_file = scene_file
    def detect(self, image):
        from v0_mvp.geometry import long_axis_angle_deg
        if self.scenario == "no_detection": return [], None
        detections = []
        for i, (name, center, size, angle, _) in enumerate(OBJECTS):
            corners = cv2.boxPoints((center, size, angle)).astype(float)
            detections.append(OBBDetection(i, name, [.94, .91, .88, .93, .92, .90, .89][i], corners, center, long_axis_angle_deg(corners)))
        if self.scene_file:
            import json
            from pathlib import Path
            scene = json.loads(Path(self.scene_file).read_text(encoding="utf-8"))
            if set(scene) != {"visible_classes"} or not isinstance(scene["visible_classes"], list) or any(x not in {o[0] for o in OBJECTS} for x in scene["visible_classes"]):
                raise ValueError("Mock scene requires visible_classes of supported class names")
            detections = [d for d in detections if d.class_name in scene["visible_classes"]]
        return detections, None
