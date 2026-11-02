from __future__ import annotations

from abc import ABC, abstractmethod
import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .contracts import ValidationRequest, ValidationResponse
from .errors import PlanningError, PlanningStatus


class PlanningClient(ABC):
    @abstractmethod
    def validate_target(self, request: ValidationRequest) -> ValidationResponse:
        raise NotImplementedError


class DisabledPlanningClient(PlanningClient):
    def validate_target(self, request: ValidationRequest) -> ValidationResponse:
        raise PlanningError(PlanningStatus.PLANNER_UNAVAILABLE, "planning integration is disabled")


class HttpPlanningClient(PlanningClient):
    """Small ROS-agnostic transport for the Ubuntu MoveIt validation bridge."""

    def __init__(self, base_url: str, *, timeout_s: float = 2.0):
        self.base_url = str(base_url).rstrip("/")
        self.timeout_s = float(timeout_s)
        if not self.base_url.startswith(("http://", "https://")):
            raise ValueError("planning base_url must start with http:// or https://")

    def validate_target(self, request: ValidationRequest) -> ValidationResponse:
        payload = json.dumps(request.to_dict(), ensure_ascii=False).encode("utf-8")
        req = Request(
            self.base_url + "/v1/validate-target", data=payload, method="POST",
            headers={"Content-Type": "application/json", "Accept": "application/json"},
        )
        try:
            with urlopen(req, timeout=self.timeout_s) as res:
                raw = res.read()
        except (HTTPError, URLError, TimeoutError, OSError) as exc:
            raise PlanningError(PlanningStatus.PLANNER_UNAVAILABLE, f"planning bridge unavailable: {exc}") from exc
        try:
            data = json.loads(raw.decode("utf-8"))
            response = ValidationResponse.from_dict(data)
        except Exception as exc:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, f"invalid planning bridge response: {exc}") from exc
        if response.request_id != request.request_id:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "planning response request_id mismatch")
        if response.validation_level_requested != request.validation_level:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "planning response validation_level_requested mismatch")
        if (response.robot_model_version_used, response.robot_model_hash_used) != (request.robot_model_version, request.robot_model_hash):
            raise PlanningError(PlanningStatus.ROBOT_MODEL_MISMATCH, "planning response robot model version/hash mismatch")
        if (response.scene_version_used, response.scene_hash_used) != (request.scene_version, request.scene_hash):
            raise PlanningError(PlanningStatus.SCENE_VERSION_MISMATCH, "planning response scene version/hash mismatch")
        if response.state_used != request.current_joint_state:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "planning response state_used does not match requested current state")
        if response.status == PlanningStatus.VALID and not response.valid:
            raise PlanningError(PlanningStatus.PROTOCOL_ERROR, "planning bridge returned VALID without satisfying requested assurance")
        return response
