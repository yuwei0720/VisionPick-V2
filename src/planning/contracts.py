from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import math
import time
import uuid

from .errors import PlanningError, PlanningStatus

SCHEMA_VERSION = 1
_ALLOWED_UNITS = {"deg", "rad"}
_ALLOWED_GRIPPER_STATES = {"OPEN", "CLOSED", "UNKNOWN"}
_ALLOWED_PAYLOAD_STATES = {"NONE", "ATTACHED"}


class AssuranceLevel(IntEnum):
    MODEL_ONLY = 0
    GOAL_VALIDATED = 1
    COMMAND_PATH_VALIDATED = 2
    EXECUTION_TRAJECTORY_VALIDATED = 3


def _finite_vector(value, n: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != n:
        raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"{label} must contain exactly {n} values")
    out = []
    for item in value:
        if isinstance(item, bool):
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"{label} must be numeric")
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"{label} must be numeric") from exc
        if not math.isfinite(number):
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"{label} must be finite")
        out.append(number)
    return tuple(out)


def _nonempty(value, label: str) -> str:
    text = str(value or "").strip()
    if not text:
        raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"{label} is required")
    return text


@dataclass(frozen=True)
class JointStateSnapshot:
    names: tuple[str, ...]
    positions: tuple[float, ...]
    units: str = "deg"
    source: str = "pymycobot.get_angles"
    captured_at_unix_ms: int = 0
    sequence: int = 0

    def __post_init__(self):
        names = tuple(str(x).strip() for x in self.names)
        if not names or any(not x for x in names) or len(set(names)) != len(names):
            raise PlanningError(PlanningStatus.JOINT_MAPPING_INVALID, "joint names must be non-empty and unique")
        positions = _finite_vector(self.positions, len(names), "current_joint_state.positions")
        units = str(self.units).lower()
        if units not in _ALLOWED_UNITS:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "joint units must be deg or rad")
        if int(self.captured_at_unix_ms) <= 0:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "captured_at_unix_ms must be > 0")
        if int(self.sequence) < 0:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "state sequence must be >= 0")
        object.__setattr__(self, "names", names)
        object.__setattr__(self, "positions", positions)
        object.__setattr__(self, "units", units)
        object.__setattr__(self, "source", _nonempty(self.source, "state source"))

    def positions_rad(self) -> tuple[float, ...]:
        if self.units == "rad":
            return self.positions
        return tuple(math.radians(v) for v in self.positions)

    def to_dict(self) -> dict:
        return {
            "names": list(self.names),
            "positions": list(self.positions),
            "units": self.units,
            "source": self.source,
            "captured_at_unix_ms": int(self.captured_at_unix_ms),
            "sequence": int(self.sequence),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "JointStateSnapshot":
        if not isinstance(data, dict):
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "current_joint_state must be an object")
        return cls(
            names=tuple(data.get("names") or ()),
            positions=tuple(data.get("positions") or ()),
            units=data.get("units", "deg"),
            source=data.get("source", "pymycobot.get_angles"),
            captured_at_unix_ms=int(data.get("captured_at_unix_ms") or 0),
            sequence=int(data.get("sequence") or 0),
        )


@dataclass(frozen=True)
class TargetPose:
    frame_id: str
    xyz_mm: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]
    label: str = "target"

    def __post_init__(self):
        object.__setattr__(self, "frame_id", _nonempty(self.frame_id, "target.frame_id"))
        object.__setattr__(self, "xyz_mm", _finite_vector(self.xyz_mm, 3, "target.xyz_mm"))
        object.__setattr__(self, "rpy_deg", _finite_vector(self.rpy_deg, 3, "target.rpy_deg"))
        object.__setattr__(self, "label", str(self.label or "target"))

    def to_dict(self) -> dict:
        return {"frame_id": self.frame_id, "xyz_mm": list(self.xyz_mm), "rpy_deg": list(self.rpy_deg), "label": self.label}

    @classmethod
    def from_dict(cls, data: dict) -> "TargetPose":
        if not isinstance(data, dict):
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "target must be an object")
        return cls(
            frame_id=data.get("frame_id"),
            xyz_mm=tuple(data.get("xyz_mm") or ()),
            rpy_deg=tuple(data.get("rpy_deg") or ()),
            label=data.get("label", "target"),
        )


@dataclass(frozen=True)
class PayloadSpec:
    state: str = "NONE"
    object_id: str | None = None
    shape: str | None = None
    dimensions_mm: tuple[float, ...] = ()
    attach_link: str | None = None
    touch_links: tuple[str, ...] = ()
    pose_xyz_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pose_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self):
        state = str(self.state).upper()
        if state not in _ALLOWED_PAYLOAD_STATES:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "payload.state must be NONE or ATTACHED")
        object.__setattr__(self, "state", state)
        if state == "NONE":
            return
        if str(self.shape or "").lower() != "box":
            raise PlanningError(PlanningStatus.PAYLOAD_GEOMETRY_UNAVAILABLE, "0.9 payload geometry currently requires shape=box")
        dims = _finite_vector(self.dimensions_mm, 3, "payload.dimensions_mm")
        if any(v <= 0 for v in dims):
            raise PlanningError(PlanningStatus.PAYLOAD_GEOMETRY_UNAVAILABLE, "payload box dimensions must be > 0")
        object.__setattr__(self, "dimensions_mm", dims)
        object.__setattr__(self, "object_id", _nonempty(self.object_id, "payload.object_id"))
        object.__setattr__(self, "attach_link", _nonempty(self.attach_link, "payload.attach_link"))
        touch = tuple(str(x).strip() for x in self.touch_links if str(x).strip())
        if not touch:
            touch = (self.attach_link,)
        object.__setattr__(self, "touch_links", tuple(dict.fromkeys(touch)))
        object.__setattr__(self, "shape", "box")
        object.__setattr__(self, "pose_xyz_mm", _finite_vector(self.pose_xyz_mm, 3, "payload.pose_xyz_mm"))
        object.__setattr__(self, "pose_rpy_deg", _finite_vector(self.pose_rpy_deg, 3, "payload.pose_rpy_deg"))

    def to_dict(self) -> dict:
        return {
            "state": self.state,
            "object_id": self.object_id,
            "shape": self.shape,
            "dimensions_mm": list(self.dimensions_mm),
            "attach_link": self.attach_link,
            "touch_links": list(self.touch_links),
            "pose_xyz_mm": list(self.pose_xyz_mm),
            "pose_rpy_deg": list(self.pose_rpy_deg),
        }

    @classmethod
    def from_dict(cls, data: dict | None) -> "PayloadSpec":
        data = data or {"state": "NONE"}
        if not isinstance(data, dict):
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "payload must be an object")
        return cls(
            state=data.get("state", "NONE"),
            object_id=data.get("object_id"),
            shape=data.get("shape"),
            dimensions_mm=tuple(data.get("dimensions_mm") or ()),
            attach_link=data.get("attach_link"),
            touch_links=tuple(data.get("touch_links") or ()),
            pose_xyz_mm=tuple(data.get("pose_xyz_mm") or (0.0, 0.0, 0.0)),
            pose_rpy_deg=tuple(data.get("pose_rpy_deg") or (0.0, 0.0, 0.0)),
        )


@dataclass(frozen=True)
class CollisionContact:
    body_1: str
    body_2: str
    body_type_1: str | None = None
    body_type_2: str | None = None
    depth_m: float | None = None

    def to_dict(self) -> dict:
        return {
            "body_1": self.body_1,
            "body_2": self.body_2,
            "body_type_1": self.body_type_1,
            "body_type_2": self.body_type_2,
            "depth_m": self.depth_m,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "CollisionContact":
        return cls(
            body_1=str(data.get("body_1", "")), body_2=str(data.get("body_2", "")),
            body_type_1=data.get("body_type_1"), body_type_2=data.get("body_type_2"),
            depth_m=float(data["depth_m"]) if data.get("depth_m") is not None else None,
        )


@dataclass(frozen=True)
class ValidationRequest:
    request_id: str
    current_joint_state: JointStateSnapshot
    target: TargetPose
    robot_model_version: str
    robot_model_hash: str
    scene_version: str
    scene_hash: str
    gripper_state: str = "UNKNOWN"
    payload: PayloadSpec = field(default_factory=PayloadSpec)
    validation_level: AssuranceLevel = AssuranceLevel.GOAL_VALIDATED
    max_state_age_ms: int = 1000
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self):
        if int(self.schema_version) != SCHEMA_VERSION:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"unsupported planning schema {self.schema_version}")
        object.__setattr__(self, "request_id", _nonempty(self.request_id, "request_id"))
        object.__setattr__(self, "robot_model_version", _nonempty(self.robot_model_version, "robot_model_version"))
        object.__setattr__(self, "robot_model_hash", _nonempty(self.robot_model_hash, "robot_model_hash"))
        object.__setattr__(self, "scene_version", _nonempty(self.scene_version, "scene_version"))
        object.__setattr__(self, "scene_hash", _nonempty(self.scene_hash, "scene_hash"))
        gripper = str(self.gripper_state).upper()
        if gripper not in _ALLOWED_GRIPPER_STATES:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "gripper_state must be OPEN, CLOSED, or UNKNOWN")
        object.__setattr__(self, "gripper_state", gripper)
        level = AssuranceLevel(int(self.validation_level))
        if level > AssuranceLevel.GOAL_VALIDATED:
            raise PlanningError(
                PlanningStatus.VALIDATION_LEVEL_INSUFFICIENT,
                "VisionPick 0.9 Phase 2 supports MODEL_ONLY/GOAL_VALIDATED only; command-path validation is not implemented yet",
            )
        object.__setattr__(self, "validation_level", level)
        if self.current_joint_state.source != "pymycobot.get_angles":
            raise PlanningError(
                PlanningStatus.PROTOCOL_ERROR,
                "VisionPick 0.9 hardware validation requires state source=pymycobot.get_angles",
            )
        if int(self.max_state_age_ms) <= 0:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "max_state_age_ms must be > 0")

    @classmethod
    def new(cls, **kwargs) -> "ValidationRequest":
        kwargs.setdefault("request_id", uuid.uuid4().hex)
        return cls(**kwargs)

    def state_age_ms(self, now_unix_ms: int | None = None) -> int:
        now = int(time.time() * 1000) if now_unix_ms is None else int(now_unix_ms)
        return max(0, now - int(self.current_joint_state.captured_at_unix_ms))

    def to_dict(self) -> dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "request_id": self.request_id,
            "current_joint_state": self.current_joint_state.to_dict(),
            "target": self.target.to_dict(),
            "robot_model_version": self.robot_model_version,
            "robot_model_hash": self.robot_model_hash,
            "scene_version": self.scene_version,
            "scene_hash": self.scene_hash,
            "gripper_state": self.gripper_state,
            "payload": self.payload.to_dict(),
            "validation_level": int(self.validation_level),
            "max_state_age_ms": int(self.max_state_age_ms),
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ValidationRequest":
        if not isinstance(data, dict):
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "planning request must be an object")
        return cls(
            schema_version=int(data.get("schema_version", 0)),
            request_id=data.get("request_id"),
            current_joint_state=JointStateSnapshot.from_dict(data.get("current_joint_state")),
            target=TargetPose.from_dict(data.get("target")),
            robot_model_version=data.get("robot_model_version"),
            robot_model_hash=data.get("robot_model_hash"),
            scene_version=data.get("scene_version"),
            scene_hash=data.get("scene_hash"),
            gripper_state=data.get("gripper_state", "UNKNOWN"),
            payload=PayloadSpec.from_dict(data.get("payload")),
            validation_level=AssuranceLevel(int(data.get("validation_level", 1))),
            max_state_age_ms=int(data.get("max_state_age_ms", 1000)),
        )


@dataclass(frozen=True)
class ValidationResponse:
    request_id: str
    status: PlanningStatus
    validation_level_requested: AssuranceLevel
    validation_level_achieved: AssuranceLevel
    message: str = ""
    state_used: JointStateSnapshot | None = None
    state_age_ms: int | None = None
    robot_model_version_used: str | None = None
    robot_model_hash_used: str | None = None
    scene_version_used: str | None = None
    scene_hash_used: str | None = None
    ik_found: bool | None = None
    ik_joint_names: tuple[str, ...] = ()
    ik_positions_rad: tuple[float, ...] = ()
    collision: bool | None = None
    collision_type: str | None = None
    contacts: tuple[CollisionContact, ...] = ()
    payload_collision: bool | None = None
    search_exhaustive: bool = False
    diagnostics: dict = field(default_factory=dict)
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self):
        if int(self.schema_version) != SCHEMA_VERSION:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"unsupported planning response schema {self.schema_version}")
        object.__setattr__(self, "request_id", _nonempty(self.request_id, "response.request_id"))
        try:
            status = self.status if isinstance(self.status, PlanningStatus) else PlanningStatus(self.status)
            requested = self.validation_level_requested if isinstance(self.validation_level_requested, AssuranceLevel) else AssuranceLevel(int(self.validation_level_requested))
            achieved = self.validation_level_achieved if isinstance(self.validation_level_achieved, AssuranceLevel) else AssuranceLevel(int(self.validation_level_achieved))
        except (ValueError, TypeError) as exc:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"invalid planning response enum: {exc}") from exc
        object.__setattr__(self, "status", status)
        object.__setattr__(self, "validation_level_requested", requested)
        object.__setattr__(self, "validation_level_achieved", achieved)
        if status == PlanningStatus.VALID and achieved >= AssuranceLevel.GOAL_VALIDATED:
            if self.ik_found is not True:
                raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "VALID goal response must report ik_result.found=true")
            if self.collision is not False:
                raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "VALID goal response must report collision=false")
            if self.payload_collision is True:
                raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "VALID goal response cannot report payload_collision=true")

    @property
    def valid(self) -> bool:
        return self.status == PlanningStatus.VALID and self.validation_level_achieved >= self.validation_level_requested

    def to_dict(self) -> dict:
        return {
            "schema_version": SCHEMA_VERSION,
            "request_id": self.request_id,
            "status": self.status.value,
            "valid": self.valid,
            "message": self.message,
            "validation_level_requested": int(self.validation_level_requested),
            "validation_level_achieved": int(self.validation_level_achieved),
            "state_used": self.state_used.to_dict() if self.state_used else None,
            "state_age_ms": self.state_age_ms,
            "robot_model_version_used": self.robot_model_version_used,
            "robot_model_hash_used": self.robot_model_hash_used,
            "scene_version_used": self.scene_version_used,
            "scene_hash_used": self.scene_hash_used,
            "ik_result": {
                "found": self.ik_found,
                "joint_names": list(self.ik_joint_names),
                "positions_rad": list(self.ik_positions_rad),
                "search_exhaustive": bool(self.search_exhaustive),
            },
            "collision_result": {
                "collision": self.collision,
                "collision_type": self.collision_type,
                "contacts": [x.to_dict() for x in self.contacts],
            },
            "payload_collision": self.payload_collision,
            "diagnostics": self.diagnostics,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ValidationResponse":
        ik = data.get("ik_result") or {}
        collision = data.get("collision_result") or {}
        return cls(
            schema_version=int(data.get("schema_version", SCHEMA_VERSION)),
            request_id=str(data.get("request_id", "")),
            status=PlanningStatus(data.get("status", "INTERNAL_ERROR")),
            message=str(data.get("message", "")),
            validation_level_requested=AssuranceLevel(int(data.get("validation_level_requested", 1))),
            validation_level_achieved=AssuranceLevel(int(data.get("validation_level_achieved", 0))),
            state_used=JointStateSnapshot.from_dict(data["state_used"]) if data.get("state_used") else None,
            state_age_ms=int(data["state_age_ms"]) if data.get("state_age_ms") is not None else None,
            robot_model_version_used=data.get("robot_model_version_used"),
            robot_model_hash_used=data.get("robot_model_hash_used"),
            scene_version_used=data.get("scene_version_used"),
            scene_hash_used=data.get("scene_hash_used"),
            ik_found=ik.get("found"),
            ik_joint_names=tuple(ik.get("joint_names") or ()),
            ik_positions_rad=tuple(float(x) for x in (ik.get("positions_rad") or ())),
            collision=collision.get("collision"),
            collision_type=collision.get("collision_type"),
            contacts=tuple(CollisionContact.from_dict(x) for x in (collision.get("contacts") or ())),
            payload_collision=data.get("payload_collision"),
            search_exhaustive=bool(ik.get("search_exhaustive", False)),
            diagnostics=dict(data.get("diagnostics") or {}),
        )
