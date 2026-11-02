"""One persisted UI override layer over config; each task gets a frozen snapshot."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import threading

from .direct_control_v2 import ArrivalSettings
from .events import atomic_json


@dataclass(frozen=True)
class FrozenArrivalState:
    settings: ArrivalSettings
    sources: tuple
    revision: str

    def to_dict(self):
        return {"values": self.settings.to_dict(), "sources": dict(self.sources), "revision": self.revision}


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate_arrival_setting_key")
        result[key] = value
    return result


class ArrivalSettingsStoreV2:
    """Persist only six arrival keys to operator_data/arrival_settings_v2.json.

    Reading missing overrides never writes. Invalid persisted/config values are
    reported rather than clamped or replaced. Saving requires the effective
    revision, including current config values and the source of each value.
    """

    def __init__(self, root, *, config_loader):
        self.path = Path(root) / "operator_data" / "arrival_settings_v2.json"
        self.config_loader = config_loader
        self._lock = threading.RLock()

    def _overrides(self):
        if self.path.is_symlink():
            raise ValueError("arrival_override_symlink_forbidden")
        if not self.path.exists():
            return {}
        data = json.loads(self.path.read_text(encoding="utf-8"), object_pairs_hook=_unique_object)
        if (not isinstance(data, dict) or set(data) != {"schema_version", "overrides"}
                or type(data["schema_version"]) is not int or data["schema_version"] != 1
                or not isinstance(data["overrides"], dict)):
            raise ValueError("invalid_arrival_override_contract")
        return data["overrides"]

    def _state(self, overrides):
        config = self.config_loader()
        if not isinstance(config, dict) or not isinstance(config.get("robot"), dict):
            raise ValueError("robot_config_required")
        robot = config["robot"]
        settings = ArrivalSettings.from_config(robot, overrides)
        if settings.motion_poll_interval_s < 0.25:
            raise ValueError("motion_poll_interval_s_must_be_at_least_0.25")
        sources = tuple((key, "ui_override" if key in overrides else "config" if key in robot else "program_default")
                        for key in settings.to_dict())
        identity = {"values": settings.to_dict(), "sources": dict(sources), "overrides": overrides}
        revision = hashlib.sha256(json.dumps(identity, sort_keys=True, allow_nan=False).encode()).hexdigest()
        return FrozenArrivalState(settings, sources, revision)

    def freeze(self):
        with self._lock:
            return self._state(self._overrides())

    def read(self):
        state = self.freeze()
        return {"schema_version": 1, **state.to_dict()}

    def save(self, overrides, *, expected_revision):
        with self._lock:
            if not isinstance(overrides, dict):
                raise ValueError("arrival_overrides_must_be_object")
            overrides = json.loads(json.dumps(overrides, allow_nan=False))
            current = self.freeze()
            if not isinstance(expected_revision, str) or expected_revision != current.revision:
                raise ValueError("stale_arrival_settings_revision")
            state = self._state(overrides)  # Validate before directory creation or write.
            self.path.parent.mkdir(parents=True, exist_ok=True)
            atomic_json(self.path, {"schema_version": 1, "overrides": overrides})
            return {"schema_version": 1, **state.to_dict()}
