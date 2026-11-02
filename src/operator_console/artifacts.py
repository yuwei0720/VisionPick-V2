from pathlib import Path
import json
import zipfile

from .events import redact, sha256, utc_now

INTERMEDIATES = ("frame", "perception_preview", "transcript", "raw_text", "ai_api_raw_response", "ai_request_policy", "parser_output", "validation_result",
                 "tool_or_part_resolution", "real_world_conditions", "grasp_candidates", "candidate_validation",
                 "vision_detections", "selected_target", "grasp_pose", "motion_profile", "place_profile", "place_plan", "robot_commands",
                 "blocked_reason", "provenance", "asset_snapshot", "run_bundle_path", "unknown_reason", "operator_outcome",
                 "relative_gap", "relative_place_plan", "relative_scene", "task_plan", "task_plan_state", "task_plan_commands", "performance")


def intermediate_outputs(path):
    out = {}
    for name in INTERMEDIATES:
        file = Path(path) / (name + ".json")
        if file.is_symlink(): raise ValueError("Symlink artifact rejected")
        if file.is_file(): out[name] = json.loads(file.read_text(encoding="utf-8"))
    return out


def write_png(path, image, *, rotation_deg=0):
    """Encode before writing so Windows paths may contain Chinese characters."""
    import cv2
    from integration.scene_grounding import normalize_rotation_deg
    rotation = normalize_rotation_deg(rotation_deg)
    if rotation:
        image = cv2.rotate(image, {90: cv2.ROTATE_90_CLOCKWISE,
                                  180: cv2.ROTATE_180,
                                  270: cv2.ROTATE_90_COUNTERCLOCKWISE}[rotation])
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise OSError("Could not encode PNG frame")
    Path(path).write_bytes(encoded.tobytes())


def capture_artifacts(bundle, frame, *, simulated=False, warmup_frames=None, source=None, rotation_deg=0):
    import numpy as np
    write_png(bundle.path / "raw_frame.png", frame.color_bgr)
    write_png(bundle.path / "workspace_frame.png", frame.color_bgr, rotation_deg=rotation_deg)
    np.save(bundle.path / "depth_m.npy", frame.depth_m, allow_pickle=False)
    intr = frame.intrinsics
    fields = {}
    for key in ("width", "height", "fx", "fy", "ppx", "ppy", "model", "coeffs"):
        value = getattr(intr, key, None)
        fields[key] = str(value) if key == "model" else value
    bundle.write_json("frame.json", {"captured_at": utc_now(), "depth_units": "meters", "depth_source": "fixture" if simulated else "RealSense_aligned_color", "intrinsics": fields, "warmup_frames": warmup_frames, "simulated": simulated,
        "display_image": "workspace_frame.png", "display_rotation_deg": int(rotation_deg) % 360,
        "raw_image": "raw_frame.png", "raw_coordinates_preserved": True})
    bundle.emit("perception", "frame_captured", data={"raw_frame": "raw_frame.png", "depth": "depth_m.npy", "simulated": simulated}, source=source)


def detection_artifacts(bundle, detections, *, confidence, source=None):
    bundle.write_json("detections.json", detections)
    bundle.write_json("low_conf_candidates.json", {
        "status": "unavailable", "reason": "Detector output is already confidence/NMS filtered. No lower-threshold inference is enabled.",
        "effective_confidence": confidence, "pre_threshold_candidates": None,
        "returned_detections": detections,
    })
    bundle.emit("perception", "detection_completed", data={"count": len(detections), "detections": detections, "pre_threshold_available": False}, source=source)


def safe_run_path(root, run_id):
    import re
    if not re.fullmatch(r"\d{8}_\d{6}_[a-f0-9]{10}", run_id):
        raise ValueError("Invalid run ID")
    root = Path(root).resolve()
    p = root / run_id
    if p.is_symlink() or not p.is_dir() or p.resolve().parent != root:
        raise FileNotFoundError("Run not found")
    return p


def load_run(root, run_id):
    path = safe_run_path(root, run_id)
    def read(name, fallback=None):
        file = path / name
        if file.is_symlink(): raise ValueError("Symlink artifact rejected")
        return json.loads(file.read_text(encoding="utf-8")) if file.is_file() else fallback
    info = read("run.json")
    if info.get("schema_version") != 1:
        raise ValueError("Unsupported run schema version")
    events = []
    ep = path / "motion_events.jsonl"
    if ep.is_symlink(): raise ValueError("Symlink artifact rejected")
    if ep.exists():
        for line in ep.read_text(encoding="utf-8").splitlines():
            try: row = json.loads(line)
            except json.JSONDecodeError: continue  # Writer can be between write and flush.
            if row.get("schema_version") != 1: raise ValueError("Unsupported event schema version")
            events.append(row)
    integrity = {"status": "unfinalized", "mismatches": []}
    manifest = read("manifest.json")
    if manifest:
        bad = []
        for name, expected in manifest["files"].items():
            file = path / name
            if Path(name).name != name or file.is_symlink() or not file.is_file() or sha256(file) != expected["sha256"]:
                bad.append(name)
        integrity = {"status": "PASS" if not bad else "FAIL", "mismatches": bad}
    return {"run": info, "events": events, "review": read("review.json"), "asset_snapshot": read("asset_snapshot.json"), "semantic": read("semantic.json"), "tool_resolution": read("tool_resolution.json"), "error": read("error.json"), "blocked_reason": read("blocked_reason.json"), "operator_outcome": read("operator_outcome.json"), "active_modules": read("active_modules.json", []), "integrity": integrity, "confirmation": None, "replay_only": True, "intermediates": intermediate_outputs(path)}


def export_bundle(root, run_id):
    path = safe_run_path(root, run_id)
    run = load_run(root, run_id)
    if run["run"]["status"] not in {"completed", "error", "blocked", "aborted"}:
        raise ValueError("Wait for the run to finish before exporting")
    if run["integrity"]["status"] != "PASS":
        raise ValueError("Bundle integrity failed; export blocked")
    import io
    data = io.BytesIO()
    with zipfile.ZipFile(data, "w", zipfile.ZIP_DEFLATED) as archive:
        for p in sorted(path.iterdir()):
            if p.is_symlink(): raise ValueError("Symlink artifact rejected")
            if not p.is_file() or p.suffix not in {".json", ".jsonl", ".yaml", ".log", ".png", ".npy"}: continue
            raw = p.read_bytes()
            if p.suffix in {".json", ".jsonl", ".yaml", ".log"}:
                text = raw.decode("utf-8")
                # Export never silently changes hashed evidence.
                if redact(text) != text: raise ValueError("Secret-like data detected; export blocked")
            archive.writestr(run_id + "/" + p.name, raw)
    return data.getvalue()
