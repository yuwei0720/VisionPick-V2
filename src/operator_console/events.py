"""Bounded, non-blocking event recording; images are saved outside motion polling."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import dataclasses
import hashlib
import json
import math
import os
import queue
import re
import threading
import time
import traceback
import uuid


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


SENSITIVE = re.compile(r"(?:api.?key|authorization|password|secret|access.?token|refresh.?token|confirmation.?token)", re.I)
TOKEN = re.compile(r"AIza[\w-]{20,}|\bsk-[\w-]{20,}|\bBearer\s+[\w.+/~=-]+", re.I)
ASSIGNMENT = re.compile(r"(?i)((?:api[_-]?key|x-goog-api-key|authorization|password|secret|access_token)[\"']?\s*[=:]\s*[\"']?)[^\s,;\"'}]+")

# Explicit stage registry, not a profiler; names the responsible component.
STAGE_SOURCES = {
    'camera_connected': ('v0_mvp.camera', 'RealSenseCamera.connect'),
    'frame_captured': ('v0_mvp.camera', 'RealSenseCamera.capture'),
    'detection_completed': ('v0_mvp.detector', 'YOLO11OBBDetector.detect'),
    'depth_completed': ('v0_mvp.core', 'solve_detection_to_grasp'),
    'grasp_plan_created': ('v0_mvp.grasp', 'build_top_down_plan'),
    'semantic_completed': ('operator_console.task_parser_v2', 'parse_task_plan_v2'),
    'input_received': ('operator_console.runtime_v2', 'ConsoleRuntimeV2.create'),
    'robot_pose_read': ('operator_console.runtime_v2', 'ConsoleRuntimeV2.read_pose'),
    'robot_free_mode_enabled': ('operator_console.robot_adapter', 'OperatorRobotController.enter_free_mode'),
    'robot_free_mode_disabled': ('operator_console.robot_adapter', 'OperatorRobotController.exit_free_mode'),
    'asset_snapshot_recorded': ('operator_console.assets', 'AssetRegistry.register_current'),
    'run_created': ('operator_console.events', 'RunBundle.__init__'),
    'run_completed': ('operator_console.events', 'RunBundle.finish'),
    'run_failed': ('operator_console.events', 'RunBundle.finish'),
    'run_aborted': ('operator_console.events', 'RunBundle.finish'),
}
SIM_SOURCES = {
    'camera_connected': ('operator_console.simulation', 'SimCamera.connect'),
    'frame_captured': ('operator_console.simulation', 'SimCamera.capture'),
    'detection_completed': ('operator_console.simulation', 'SimDetector.detect'),
    'semantic_completed': ('operator_console.task_parser_v2', 'validate_task_plan_v2_candidate'),
}


def redact(value):
    """Return strict JSON data, redacting nested fields and known env secret values."""
    if dataclasses.is_dataclass(value):
        value = dataclasses.asdict(value)
    if isinstance(value, dict):
        return {str(k): "[REDACTED]" if SENSITIVE.search(str(k)) else redact(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [redact(v) for v in value]
    if hasattr(value, "tolist"):
        return redact(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, (bool, int, float)) or value is None:
        return value
    text = str(value)
    text = TOKEN.sub("[REDACTED]", text)
    text = ASSIGNMENT.sub(r"\1[REDACTED]", text)
    for key in ("GEMINI_API_KEY", "GOOGLE_API_KEY", "OPENAI_API_KEY"):
        secret = os.environ.get(key)
        if secret:
            text = text.replace(secret, "[REDACTED]")
    return text


def atomic_json(path, data):
    path = Path(path)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(redact(data), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


class RecordingError(RuntimeError):
    pass


class RunBundle:
    def __init__(self, root, request, *, queue_size=2048):
        from .performance import PerformanceTracker
        self._started_monotonic = time.monotonic()
        self._performance = PerformanceTracker(self._started_monotonic, simulated=request.get("robot") != "v6")
        self.run_id = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:10]
        self.path = Path(root) / self.run_id
        self.path.mkdir(parents=True, exist_ok=False)
        self._queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.RLock()
        self._events = deque(maxlen=2048)
        self._modules = {}
        self._seq = 0
        self._failure = None
        self._closed = False
        self.info = {"schema_version": 1, "run_id": self.run_id, "created_at": utc_now(), "status": "created", "request": redact(request), "hardware_validated": False}
        self.write_json("run.json", self.info)
        self._thread = threading.Thread(target=self._write_events, name="visionpick-event-writer", daemon=True)
        self._thread.start()
        self.emit("run", "run_created", data={"request": request})

    def emit(self, stage, event, *, status="ok", data=None, source=None):
        with self._lock:
            if self._closed:
                return
            self._seq += 1
            if source is None:
                registry = {**STAGE_SOURCES, **(SIM_SOURCES if self.info["request"].get("backend") == "simulation" else {})}
                module, function = registry.get(event, ("operator_console.runtime_v2", "ConsoleRuntimeV2._event"))
                source = {"module": module, "function": function, "file": "src/" + module.replace(".", "/") + ".py"}
            row = redact({"schema_version": 1, "run_id": self.run_id, "sequence": self._seq, "timestamp": utc_now(),
                          "elapsed_s": time.monotonic() - self._started_monotonic,
                          "stage": stage, "event": event, "status": status, "source": source, "data": data or {}})
            self._performance.consume(row)
            self._events.append(row)
            key = source["module"] + ":" + source["function"]
            item = self._modules.setdefault(key, {**source, "first_event_at": row["timestamp"], "event_count": 0})
            item.update(last_event_at=row["timestamp"], last_event=event, event_count=item["event_count"] + 1)
            try:
                self._queue.put_nowait(row)
            except queue.Full:
                self._failure = "Event queue full; recording is incomplete"

    def _write_events(self):
        try:
            with (self.path / "motion_events.jsonl").open("w", encoding="utf-8") as events, (self.path / "terminal.log").open("w", encoding="utf-8") as terminal:
                while True:
                    row = self._queue.get()
                    try:
                        if row is None:
                            return
                        events.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
                        terminal.write(f"{row['timestamp']} | {row['stage']} | {row['event']} | {row['status']} | {json.dumps(row['data'], ensure_ascii=False)}\n")
                        events.flush()
                        terminal.flush()
                    finally:
                        self._queue.task_done()
        except Exception as exc:
            self._failure = str(redact(f"Event writer failed: {exc}"))

    def check(self):
        if self._failure:
            raise RecordingError(self._failure)

    def events(self, after=0):
        with self._lock:
            return [dict(e) for e in self._events if e["sequence"] > after]

    def write_json(self, name, data):
        if Path(name).name != name:
            raise ValueError("Artifact must be a filename")
        with self._lock:
            atomic_json(self.path / name, data)

    def state(self, status, **extra):
        with self._lock:
            self.info.update(status=status, updated_at=utc_now(), **redact(extra))
            self.write_json("run.json", self.info)

    def snapshot(self):
        from .artifacts import intermediate_outputs
        with self._lock:
            result = {"run": deepcopy(self.info), "events": self.events()}
            if not self.info.get("recording_error"):
                result["intermediates"] = intermediate_outputs(self.path)
                result["intermediates"]["performance"] = self._performance.result(
                    None if self._closed else time.monotonic() - self._started_monotonic)
            return result

    def fail(self, exc):
        detail = {"type": type(exc).__name__, "message": str(exc), "stack_trace": traceback.format_exc()}
        self.write_json("error.json", detail)
        self.emit("run", "error", status="error", data=detail)

    def finish(self, status):
        if self._closed:
            return
        if self._failure:
            status = "error"
        self.emit("run", {"completed": "run_completed", "aborted": "run_aborted"}.get(status, "run_failed"), status="ok" if status == "completed" else status)
        try:
            self._queue.put(None, timeout=2)
        except queue.Full:
            self._failure = "Event writer did not drain"
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            self._failure = "Event writer did not close"
        self._closed = True
        self.write_json("performance.json", self._performance.result())
        self.write_json("active_modules.json", list(self._modules.values()))
        self.state("error" if self._failure else status, completed_at=utc_now(), recording_complete=not bool(self._failure), recording_error=self._failure)
        files = {}
        for p in sorted(self.path.iterdir()):
            if p.is_file() and p.name != "manifest.json":
                files[p.name] = {"sha256": sha256(p), "bytes": p.stat().st_size}
        self.write_json("manifest.json", {"schema_version": 1, "run_id": self.run_id, "files": files, "recording_complete": not bool(self._failure)})
