from __future__ import annotations

from datetime import datetime, timezone
from importlib import metadata
from typing import Any


def _safe_read(backend: Any, method: str):
    fn = getattr(backend, method, None)
    if not callable(fn):
        return {"supported": False, "value": None, "error": None}
    try:
        return {"supported": True, "value": fn(), "error": None}
    except Exception as exc:
        return {"supported": True, "value": None, "error": f"{type(exc).__name__}: {exc}"}


def pymycobot_version() -> str | None:
    try:
        return metadata.version("pymycobot")
    except metadata.PackageNotFoundError:
        return None


def capture_mycobot280_m5_read_only_facts(backend: Any, *, port: str, baud: int, library_version: str | None = None) -> dict:
    """Collect read-only identity/state facts without issuing motion commands."""
    facts = {
        "schema_version": 1,
        "kind": "visionpick_hardware_facts_capture",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "manufacturer": "Elephant Robotics",
        "exact_model": "myCobot 280 M5",
        "generation": "M5",
        "transport": {"port": str(port), "baud": int(baud)},
        "pymycobot_version": library_version if library_version is not None else pymycobot_version(),
        "reads": {},
        "motion_commands_issued": False,
        "safety_note": "Read-only capture only; this file does not verify geometry, joint mapping, or collision safety.",
    }
    for method in ("get_angles", "get_coords", "get_system_version", "get_basic_version", "get_error_information"):
        facts["reads"][method] = _safe_read(backend, method)
    return facts
