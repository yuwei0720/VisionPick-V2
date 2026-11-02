from __future__ import annotations

import time
from .contracts import JointStateSnapshot
from .errors import PlanningError, PlanningStatus


class RobotStateSequencer:
    """Creates monotonic state sequence numbers for stateless planning requests."""
    def __init__(self, start: int = 0):
        self._sequence = int(start)

    def capture(self, robot, joint_names, *, now_ms=None) -> JointStateSnapshot:
        angles = robot._try_get_angles() if hasattr(robot, "_try_get_angles") else None
        if not angles or len(angles) < len(joint_names):
            raise PlanningError(PlanningStatus.STATE_UNAVAILABLE, "pymycobot.get_angles() did not return a complete joint state")
        self._sequence += 1
        stamp = int(time.time() * 1000) if now_ms is None else int(now_ms)
        return JointStateSnapshot(
            names=tuple(joint_names), positions=tuple(float(x) for x in angles[:len(joint_names)]),
            units="deg", source="pymycobot.get_angles", captured_at_unix_ms=stamp, sequence=self._sequence,
        )
