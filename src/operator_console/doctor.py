from __future__ import annotations

from importlib import metadata, util
from pathlib import Path
import math
import os
import platform
import subprocess
import sys

from .events import sha256


def environment(root):
    versions = {}
    for name in ("numpy", "opencv-python", "opencv-python-headless", "PyYAML", "ultralytics", "pyrealsense2", "pymycobot", "google-genai", "sounddevice"):
        try: versions[name] = metadata.version(name)
        except metadata.PackageNotFoundError: versions[name] = None
    state = {}
    for label, args in (("commit", ["rev-parse", "HEAD"]), ("branch", ["branch", "--show-current"]), ("dirty", ["status", "--porcelain=v1"])):
        try:
            proc = subprocess.run(["git", *args], cwd=root, capture_output=True, text=True, timeout=3)
            state[label] = proc.stdout.strip() if proc.returncode == 0 else "unavailable"
        except (OSError, subprocess.TimeoutExpired): state[label] = "unavailable"
    return {"python": sys.version.split()[0], "platform": platform.platform(), "packages": versions}, state


def validate_baseline(cfg):
    """Operator entry points do not expose or relax V6 policy through profiles."""
    from v0_mvp.motion_safety import validate_motion_unlock
    validate_motion_unlock(cfg)
    def finite_tree(obj):
        if isinstance(obj, dict):
            for v in obj.values(): finite_tree(v)
        elif isinstance(obj, (list, tuple)):
            for v in obj: finite_tree(v)
        elif isinstance(obj, float) and not math.isfinite(obj):
            raise ValueError("Config contains a non-finite number")
    finite_tree(cfg)
    r, g = cfg["robot"], cfg["grasp"]
    checks = {
        "pregrasp mode": (r.get("pregrasp_coordinate_mode", 0), 0),
        "approach mode": (r.get("approach_coordinate_mode", 0), 0),
        "lift mode": (r.get("lift_coordinate_mode", 0), 0),
        "approach clearance": (g["approach_clearance_mm"], 80),
        "lift clearance": (g["lift_clearance_mm"], 60),
        "gripper open state": (r["gripper"].get("open_state", 0), 0),
        "gripper close state": (r["gripper"].get("close_state", 1), 1),
        "gripper open speed": (r["gripper"].get("open_speed", 50), 50),
        "gripper close speed": (r["gripper"].get("close_speed", 80), 80),
        "model confidence": (cfg["model"].get("confidence", .35), .35),
        "model IoU": (cfg["model"].get("iou", .5), .5),
        "warmup frames": (cfg["camera"].get("warmup_frames", 15), 15),
    }
    for label, (value, expected) in checks.items():
        if isinstance(value, bool) or value != expected: raise ValueError(f"V6 policy locked: {label}={expected}")
    if r.get("move_to_start_before_pick", True) is not True:
        raise ValueError("V6 Start initialization is required")
    for key, bounds in {"x": [-280, 280], "y": [-280, 280], "z": [70, 320]}.items():
        if list(g["workspace_mm"][key]) != bounds: raise ValueError("V6 workspace is locked")
    for name in ("start_angle_speed", "coordinate_speed", "place_joint_speed"):
        speed = r.get(name, r.get("start_angle_speed", 15))
        if type(speed) is not int or not 1 <= speed <= 100:
            raise ValueError(f"{name} must be an integer within 1..100")
    if r.get("approach_strategy", "segmented") not in {"segmented", "linear"}:
        raise ValueError("Approach strategy must be segmented or linear")
    if not 0 < float(r.get("pose_verify_position_tolerance_mm", 20)) <= 20: raise ValueError("Position tolerance must stay within 20 mm")
    if not 0 < float(r.get("pose_verify_orientation_tolerance_deg", 8)) <= 8: raise ValueError("Orientation tolerance must stay within 8 degrees")
    stable = r.get("pose_verify_stable_polls", 1)
    if type(stable) is not int or stable < 1: raise ValueError("At least 1 stable poll is required")
    coarse = r.get("coarse_step_mm", 25.0)
    if isinstance(coarse, bool) or not isinstance(coarse, (int, float)) or not math.isfinite(coarse) or coarse < 1:
        raise ValueError("Coarse step must be finite and at least 1 mm")
    if float(cfg["calibration"].get("max_rmse_mm", 10)) > 15: raise ValueError("RMSE limit cannot exceed 15 mm")


def diagnose(cfg, request, root, runs_root, *, hardware_enabled=False):
    checks = []
    def add(name, status, detail): checks.append({"name": name, "status": status, "detail": detail})
    simulation = request["backend"] == "simulation"
    try:
        validate_baseline(cfg)
        add("V6 safety / calibration", "PASS", f"RMSE {cfg['calibration']['rmse_mm']:.3f} mm; V6 policy preserved")
    except Exception as exc:
        add("V6 safety / calibration", "BLOCKED", str(exc))
    camera_real = request.get("camera", "none" if simulation else "realsense") == "realsense"
    robot_real = request.get("robot", "fake" if simulation else "v6") == "v6" and request["execution"] != "perceive"
    if robot_real and not request.get('enable_motion', False):
        add('Motion authorization', 'BLOCKED', 'motion_not_enabled: explicit --enable-motion and an allowed confirmation policy are required')
    if (camera_real or robot_real) and not hardware_enabled:
        add("Backend", "BLOCKED", "Hardware disabled; use ui --hardware or the explicit mode CLI")
    else:
        add("Backend", "WARNING" if not robot_real else "PASS", f"mode={request.get('mode')}; camera={request.get('camera')}; robot={request.get('robot')}; llm={request.get('llm')}")
    speed = int(cfg["robot"].get("coordinate_speed", 15))
    if speed > 15:
        add("High-speed motion", "WARNING", f"coordinate_speed={speed}; above the historical baseline 15; operator limit is 100; joint transfer={cfg['robot'].get('place_joint_speed', 15)}; initialization=15")
    strategy = str(cfg["robot"].get("approach_strategy", "segmented"))
    if strategy == "linear":
        add("Approach strategy", "WARNING", "linear: one verified mode=1 approach target; historical 20/5 mm segmented mode is bypassed for this run")
    else:
        add("Approach strategy", "PASS", "segmented: historical 20/5 mm verified approach")
    if request.get("confirmation_mode") == "session_auto":
        add("Confirmation policy", "WARNING", "session_auto requested; runtime session qualification must match this motion profile")
    else:
        add("Confirmation policy", "PASS", "operator-selectable: two-stage or direct execution")
    for module in ("numpy", "cv2", "yaml", "pydantic", "jsonschema"):
        add(module, "PASS" if util.find_spec(module) else "BLOCKED", "Python dependency")
    if not request.get("reuse_semantic_run_id") and request.get("input_mode") != "target":
        add("opencc", "PASS" if util.find_spec("opencc") else "BLOCKED", "Required for semantic normalization")
    weights = Path(cfg["model"]["weights"])
    if not weights.is_absolute(): weights = Path(root) / weights
    if weights.is_file(): add("Model weights", "PASS", sha256(weights))
    else: add("Model weights", "BLOCKED" if camera_real else "WARNING", "Missing: " + str(weights))
    if camera_real:
        for module in ("ultralytics", "pyrealsense2"):
            add(module, "PASS" if util.find_spec(module) else "BLOCKED", "Required for real perception")
        if util.find_spec("pyrealsense2"):
            try:
                import pyrealsense2 as rs
                devices = list(rs.context().query_devices())
                add("RealSense device", "PASS" if devices else "BLOCKED", f"{len(devices)} device(s), enumeration only")
            except Exception as exc: add("RealSense device", "BLOCKED", str(exc))
    if robot_real:
        add("pymycobot", "PASS" if util.find_spec("pymycobot") else "BLOCKED", "Required for motion")
        try:
            from serial.tools.list_ports import comports
            ports = [p.device for p in comports()]
            add("Robot port", "PASS" if cfg["robot"]["port"] in ports else "BLOCKED", f"Configured {cfg['robot']['port']}; available {ports}; no robot command sent")
        except ImportError: add("Robot port", "BLOCKED", "pyserial is unavailable")
    requires_api = not request.get("reuse_semantic_run_id") and (request.get("speech") in {"file", "mic"} or (request["input_mode"] != "target" and request.get("llm", "api") == "api"))
    if requires_api:
        present = bool(os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY"))
        add("Gemini API key", "PASS" if present else "BLOCKED", "present" if present else "api_key_missing; no mock fallback")
    if not request.get("reuse_semantic_run_id") and (request["input_mode"] != "target" or request.get("speech") in {"file", "mic"}):
        try: metadata.version("google-genai"); sdk = True
        except metadata.PackageNotFoundError: sdk = False
        add("google-genai", "PASS" if sdk else "BLOCKED", "Existing parser dependency; no API call during doctor")
    if request.get("speech") == "mic":
        add("sounddevice", "PASS" if util.find_spec("sounddevice") else "BLOCKED", "Required for microphone recording")
        if util.find_spec("sounddevice"):
            try:
                import sounddevice as sd
                sd.query_devices(kind="input")
                add("Microphone", "PASS", "Default input available; not recording")
            except Exception as exc: add("Microphone", "BLOCKED", str(exc))
    for key in ("audio_file", "scene_file", "conditions_file"):
        if request.get(key):
            add(key, "PASS" if Path(request[key]).is_file() else "BLOCKED", "File exists" if Path(request[key]).is_file() else "Missing input file")
    if request.get("llm") == "mock":
        add("LLM", "WARNING", "EXPLICIT MOCK: CI only; not real AI acceptance evidence")
    try:
        Path(runs_root).mkdir(parents=True, exist_ok=True)
        import tempfile
        with tempfile.TemporaryFile(dir=runs_root) as probe: probe.write(b"write-check")
        add("Run storage", "PASS", "Writable")
    except OSError as exc: add("Run storage", "BLOCKED", str(exc))
    if request.get("execution") == "full_place":
        add(
            "Place", "WARNING",
            "Level-1 fixed robot-frame destination enabled: verified transfer, segmented descent, release and retreat; no visual destination tracking or collision avoidance",
        )
    else:
        add("Place", "PASS", "Available through full_place with an explicitly activated fixed destination profile")
    return {"status": "BLOCKED" if any(c["status"] == "BLOCKED" for c in checks) else "WARNING" if any(c["status"] == "WARNING" for c in checks) else "PASS", "checks": checks, "hardware_validated": False}
