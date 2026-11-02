from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol
import time

from .contracts import AssuranceLevel, CollisionContact, ValidationRequest, ValidationResponse
from .errors import PlanningError, PlanningStatus


@dataclass(frozen=True)
class BackendGoalResult:
    status: PlanningStatus
    message: str = ""
    ik_found: bool | None = None
    ik_joint_names: tuple[str, ...] = ()
    ik_positions_rad: tuple[float, ...] = ()
    collision: bool | None = None
    collision_type: str | None = None
    contacts: tuple[CollisionContact, ...] = ()
    payload_collision: bool | None = None
    search_exhaustive: bool = False
    diagnostics: dict = field(default_factory=dict)


class StaticCollisionBackend(Protocol):
    def validate_goal(self, request: ValidationRequest) -> BackendGoalResult: ...


class StatelessCollisionValidator:
    """Protocol-level fail-closed validator around a MoveIt/static backend.

    The backend never owns the authoritative current robot state. Every request
    contains a fresh state captured by VisionPick from the physical controller.
    """

    def __init__(self, backend: StaticCollisionBackend, *, robot_model_version: str, robot_model_hash: str,
                 scene_version: str, scene_hash: str, now_ms=None):
        self.backend = backend
        self.robot_model_version = str(robot_model_version)
        self.robot_model_hash = str(robot_model_hash)
        self.scene_version = str(scene_version)
        self.scene_hash = str(scene_hash)
        self._now_ms = now_ms or (lambda: int(time.time() * 1000))

    def _response(self, request: ValidationRequest, status: PlanningStatus, message: str, *, achieved=AssuranceLevel.MODEL_ONLY, **kwargs):
        return ValidationResponse(
            request_id=request.request_id,
            status=status,
            message=message,
            validation_level_requested=request.validation_level,
            validation_level_achieved=AssuranceLevel(int(achieved)),
            state_used=request.current_joint_state,
            state_age_ms=request.state_age_ms(self._now_ms()),
            robot_model_version_used=self.robot_model_version,
            robot_model_hash_used=self.robot_model_hash,
            scene_version_used=self.scene_version,
            scene_hash_used=self.scene_hash,
            **kwargs,
        )

    def validate_target(self, request: ValidationRequest) -> ValidationResponse:
        try:
            now_ms = self._now_ms()
            signed_age = int(now_ms) - int(request.current_joint_state.captured_at_unix_ms)
            if signed_age < -request.max_state_age_ms:
                return self._response(
                    request, PlanningStatus.STATE_STALE,
                    f"current robot state timestamp is {-signed_age} ms in the future; check host clock synchronization",
                )
            age = request.state_age_ms(now_ms)
            if age > request.max_state_age_ms:
                return self._response(request, PlanningStatus.STATE_STALE, f"current robot state is {age} ms old")
            if request.robot_model_version != self.robot_model_version or request.robot_model_hash != self.robot_model_hash:
                return self._response(request, PlanningStatus.ROBOT_MODEL_MISMATCH, "robot model version/hash mismatch")
            if request.scene_version != self.scene_version or request.scene_hash != self.scene_hash:
                return self._response(request, PlanningStatus.SCENE_VERSION_MISMATCH, "planning scene version/hash mismatch")
            result = self.backend.validate_goal(request)
            achieved = AssuranceLevel.GOAL_VALIDATED if result.status == PlanningStatus.VALID else AssuranceLevel.MODEL_ONLY
            return self._response(
                request, result.status, result.message, achieved=achieved,
                ik_found=result.ik_found, ik_joint_names=result.ik_joint_names,
                ik_positions_rad=result.ik_positions_rad, collision=result.collision,
                collision_type=result.collision_type, contacts=result.contacts,
                payload_collision=result.payload_collision, search_exhaustive=result.search_exhaustive,
                diagnostics=dict(result.diagnostics),
            )
        except PlanningError as exc:
            return self._response(request, exc.status, str(exc), diagnostics=exc.details)
        except Exception as exc:  # pragma: no cover - backend/platform specific
            return self._response(request, PlanningStatus.INTERNAL_ERROR, str(exc), diagnostics={"exception_type": type(exc).__name__})
