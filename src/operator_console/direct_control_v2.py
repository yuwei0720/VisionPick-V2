"""Direct V2 pick/place with endpoint geometry and fixed-wait completion.

Long transfers prefer joints via Preferred IK, standard IK, then Cartesian.
Vertical motion remains Cartesian; solver results never claim path validation.
V2 releases motion after a fixed physical wait. Feedback is optional telemetry,
never arrival proof. This class cannot interrupt a blocking SDK transport call
or certify the actual trajectory, arrival, grasp or physical placement.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import math
import time

from v0_mvp.robot import MyCobotController, RobotError, vertical_motion_settings
from .robot_adapter import OperatorRobotController, dispatch_result_failed


@dataclass(frozen=True)
class ArrivalSettings:
    pose_verify_stable_polls: int = 1
    pose_verify_position_tolerance_mm: float = 20.0
    pose_verify_orientation_tolerance_deg: float = 8.0
    motion_timeout_s: float = 12.0
    motion_poll_interval_s: float = 0.5
    place_joint_verify_tolerance_deg: float = 5.0

    def __post_init__(self):
        for key, value in asdict(self).items():
            if key == "pose_verify_stable_polls":
                if type(value) is not int or value <= 0:
                    raise ValueError("stable_polls_must_be_positive_integer")
            elif isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key}_must_be_finite_positive")

    @classmethod
    def from_config(cls, robot_config, overrides=None):
        allowed = set(cls.__dataclass_fields__)
        overrides = {} if overrides is None else overrides
        if not isinstance(overrides, dict) or set(overrides) - allowed:
            raise ValueError("unknown_arrival_setting")
        return cls(**({k: v for k, v in robot_config.items() if k in allowed} | overrides))

    def to_dict(self):
        return asdict(self)


def _six(value, name):
    try:
        if len(value) != 6 or any(isinstance(x, bool) for x in value):
            raise ValueError
        result = [float(x) for x in value]
        if not all(math.isfinite(x) for x in result):
            raise ValueError
        return result
    except (TypeError, ValueError, OverflowError) as exc:
        raise RobotError(f"{name}_requires_six_finite_values") from exc


class DirectRobotControllerV2(OperatorRobotController):
    """Frozen settings; serial timeouts permit one retry before fixed-wait completion.

    Only an upstream, current-frame geometry pass permits a complete task. All
    generated endpoint poses are bounded again before initialization or gripping.
    Start dispatches configured joints and waits; telemetry never proves arrival.
    """

    def __init__(self, config, *, backend=None, settings=None, clock_fn=time.monotonic,
                 sleep_fn=time.sleep, event_sink=None, checkpoint=None):
        robot_config = deepcopy(config["robot"])
        self.arrival = settings or ArrivalSettings.from_config(robot_config)
        if not isinstance(self.arrival, ArrivalSettings):
            raise ValueError("validated_arrival_settings_required")
        self._clock = clock_fn
        self._workspace = {}
        for axis in ("x", "y", "z"):
            bounds = config["grasp"]["workspace_mm"][axis]
            if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                    or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in bounds)
                    or bounds[0] >= bounds[1]):
                raise ValueError("invalid_workspace_bounds")
            self._workspace[axis] = tuple(bounds)
        robot_config.update(self.arrival.to_dict())
        super().__init__(robot_config, backend=backend, clock_fn=clock_fn, sleep_fn=sleep_fn, event_sink=event_sink, checkpoint=checkpoint)
        self.gripper_holding_uncertain = False
        self.at_start = False
        self.command_count = 0
        self.transfer_evidence = []
        self._last_movement_activity = None
        self._vertical_continuation = None
        self.motion_completion_policy = "fixed_wait"

    def _event(self, stage, event, *, status="ok", **data):
        # The inherited gripper method emits this after its last checkpoint and
        # immediately before dispatch. Persisting here avoids pre-close locks.
        closing = event == "gripper_commanded" and data.get("gripper_state") == 1
        previous = self.gripper_holding_uncertain
        if closing:
            self.gripper_holding_uncertain = True
        if self.event_sink:
            try:
                self.event_sink(stage, event, status=status, data=data,
                    source={"module": "operator_console.direct_control_v2", "function": stage,
                            "file": "src/operator_console/direct_control_v2.py"})
            except Exception:
                if closing:
                    self.gripper_holding_uncertain = previous  # No command was dispatched.
                raise

    def _pose(self, pose):
        values = _six(pose, "pose")
        if any(not self._workspace[a][0] <= values[i] <= self._workspace[a][1] for i, a in enumerate(("x", "y", "z"))):
            raise RobotError("endpoint_outside_workspace")
        return values

    def _speed(self, key, default=15):
        value = self.cfg.get(key, default)
        if type(value) is not int or not 1 <= value <= 100:
            raise RobotError(f"{key}_must_be_integer_1_100")
        return value

    def _mode(self, key, default):
        value = self.cfg.get(key, default)
        if type(value) is not int or value not in (0, 1):
            raise RobotError("coordinate_mode_must_be_0_or_1")
        return value

    def _feedback(self, kind):
        # The installed SDK already retries its transport up to three times.
        actual = self._try_get_angles() if kind == "joint" else self._try_get_coords()
        self._last_movement_activity = self._clock()
        return actual

    def _move(self, kind, target, *, label, speed, mode=None, endpoint=None, stable_count=None, grasp_endpoint=False,
              vertical_stage=None, waypoint_index=None, intermediate=False):
        self._require()
        continuation = self._vertical_continuation
        self._vertical_continuation = None  # Failed or unrelated moves cannot resume a segment.
        if type(intermediate) is not bool or (vertical_stage is None and (intermediate or waypoint_index is not None)):
            raise RobotError("invalid_vertical_movement_metadata")
        if vertical_stage is not None and (kind != "cartesian" or vertical_stage not in {
                "grasp_descent", "grasp_descend", "grasp_lift", "place_descent", "place_retreat"}
                or type(waypoint_index) is not int or waypoint_index < 1):
            raise RobotError("invalid_vertical_movement_metadata")
        continuing = (vertical_stage is not None and waypoint_index > 1
                      and continuation == (vertical_stage, waypoint_index - 1))
        required = self.arrival.pose_verify_stable_polls if stable_count is None else stable_count
        if type(required) is not int or required < 1:
            raise RobotError("stable_count_must_be_positive_integer")
        target = _six(target, "angles") if kind == "joint" else self._pose(target)
        if kind == "joint" and endpoint is not None:
            self._pose(endpoint)  # Taught-angle geometry never invokes FK/IK.
        send = getattr(self.mc, "send_angles" if kind == "joint" else "send_coords", None)
        if not callable(send):
            raise RobotError(f"direct_{kind}_dispatch_unavailable")
        if self._last_movement_activity is not None and not continuing:
            gap = 0.25 - (self._clock() - self._last_movement_activity)
            if gap > 0:
                self.sleep(gap)
        self._before_command()
        started = self._clock()
        self._event("motion", "movement_started", label=label, kind=kind, target=target,
                    speed=speed, mode=mode, intermediate=intermediate, vertical_stage=vertical_stage,
                    arrival_settings=self.arrival.to_dict())
        self.command_count += 1  # Count application dispatch attempts, including failures.
        vertical_evidence = None
        if vertical_stage is not None:
            policy, sdk_mode, _ = vertical_motion_settings(self.cfg)
            if mode != sdk_mode:
                raise RobotError('vertical_sdk_mode_mismatch')
            vertical_evidence = {'record_type':'vertical_motion', 'label':label,
                'vertical_motion_mode':policy, 'sdk_mode':sdk_mode, 'target_pose_mm_deg':list(target),
                'motion_completed':False, 'ik_validated':False, 'path_validated':False}
            self.transfer_evidence.append(vertical_evidence)
        self.at_start = False
        result = None
        send_results = []
        dispatch_attempts = 1
        self._clear_serial_buffer()
        try:
            result = send(target, speed) if kind == "joint" else send(target, speed, mode)
            send_results.append(result)
            if result == -1 and getattr(self.mc, "_serial_port", None) is not None:
                self._clear_serial_buffer()
                self.sleep(0.2)
                self._before_command()
                self._event("motion", "dispatch_retry", label=label, kind=kind, target=target,
                            mode=mode, attempt=2, previous_send_result=result, retry_wait_s=0.2)
                self.command_count += 1
                dispatch_attempts += 1
                result = send(target, speed) if kind == "joint" else send(target, speed, mode)
                send_results.append(result)
            if dispatch_result_failed(result):
                raise RobotError(f'{label} rejected send result: {result!r}')
        except Exception as exc:
            self._last_movement_activity = self._clock()
            self._event("motion", "dispatch_failed", status="error", label=label, error_type=type(exc).__name__,
                        send_result=result, target=target, send_results=send_results, dispatch_attempts=dispatch_attempts,
                        controller_error=self._try_get_error_information())
            raise RobotError(f"{label}_dispatch_failed") from exc
        self._last_movement_activity = self._clock()
        # The bounded serial retry adds no coordinate polling or arrival gate.
        vertical_wait = float(self.cfg.get("vertical_endpoint_wait_s", self.cfg.get("vertical_wait_s", 4.0)))
        transfer_wait = float(self.cfg.get("transfer_wait_s", self.cfg.get("cross_zone_wait_s", 3.0)))
        intermediate_wait = float(self.cfg.get("intermediate_wait_s", 0.35))
        wait_s = intermediate_wait if intermediate else vertical_wait if vertical_stage is not None or grasp_endpoint else transfer_wait
        self.sleep(wait_s)
        self._before_command()
        actual, errors = None, {}
        if not intermediate:
            try:
                actual = self._feedback(kind)  # Exactly one best-effort sample.
            except Exception as exc:
                errors["telemetry_error_type"] = type(exc).__name__
            if kind == "joint":
                errors["joint_error_deg"] = max(self._angle_error_deg(a, b) for a, b in zip(actual, target)) if actual else None
            else:
                pos, ori = self._pose_errors(target, actual)
                errors.update(position_error_mm=pos if math.isfinite(pos) else None,
                              orientation_error_deg=ori if math.isfinite(ori) else None,
                              z_error_mm=abs(actual[2] - target[2]) if actual is not None else None,
                              grasp_endpoint=grasp_endpoint)
            self._event("motion", "feedback_sampled", label=label, kind=kind, actual=actual,
                        best_effort=True, arrival_verified=False, elapsed_s=self._clock() - started, **errors)
        self._before_command()
        self._event("motion", "movement_completed", label=label, kind=kind, actual=actual,
                    send_result=result, send_results=send_results, dispatch_attempts=dispatch_attempts,
                    intermediate=intermediate, wait_s=wait_s,
                    completion_basis="fixed_wait", arrival_verified=False, controller_error=None)
        # Optional telemetry must never populate a verified-arrival cache.
        self._last_movement_activity = self._clock()
        if intermediate:
            self._vertical_continuation = (vertical_stage, waypoint_index)
        if vertical_evidence is not None:
            vertical_evidence.update(motion_completed=True, completion_basis="fixed_wait", arrival_verified=False)
        return actual

    def move_pose(self, pose, *, mode=None, label="pose", stable_count=None, grasp_endpoint=False,
                  vertical_stage=None, waypoint_index=None, intermediate=False):
        mode = self._mode("coordinate_mode", 0) if mode is None else mode
        if type(mode) is not int or mode not in (0, 1):
            raise RobotError("coordinate_mode_must_be_0_or_1")
        return self._move("cartesian", pose, label=label, speed=self._speed("coordinate_speed"), mode=mode,
                          stable_count=stable_count, grasp_endpoint=grasp_endpoint, vertical_stage=vertical_stage,
                          waypoint_index=waypoint_index, intermediate=intermediate)

    def move_joint_target(self, angles, *, endpoint_pose=None, label, speed=None):
        speed = self._speed("place_joint_speed", self.cfg.get("start_angle_speed", 20)) if speed is None else speed
        if type(speed) is not int or not 1 <= speed <= 100:
            raise RobotError("joint_speed_must_be_integer_1_100")
        return self._move("joint", angles, endpoint=endpoint_pose, label=label, speed=speed)

    def _vertical_commands(self, start, end, label, mode, fine):
        start, end = self._pose(start), self._pose(end)
        if type(mode) is not int or mode not in (0, 1):
            raise RobotError("coordinate_mode_must_be_0_or_1")
        _, mode, _ = vertical_motion_settings(self.cfg)
        grasp_endpoint = label in {"grasp_descent", "grasp_descend"}
        waypoints = self._build_vertical_waypoints(start, end, fine_near_end=fine) or [end]
        return [("pose", self._pose(p), label if len(waypoints) == 1 else f"{label}[{i}]",
                 (mode, 1, grasp_endpoint and i == len(waypoints), label, i, i < len(waypoints)))
                for i, p in enumerate(waypoints, 1)]

    def initialize_for_pick(self):
        """Empty-hand Start before capture; start_verified is a legacy lifecycle flag.

        Fixed-wait release never certifies the actual joints or TCP arrival.
        """
        self._require()
        if self.gripper_holding_uncertain:
            raise RobotError("gripper_holding_uncertain")
        if not self.cfg.get("move_to_start_before_pick", True):
            raise RobotError("v2_requires_start_before_capture")
        angles = _six(self.cfg.get("start_angles_deg"), "start_angles")
        if any(abs(a) > 360 for a in angles):
            raise RobotError("invalid_start_joint_range")
        endpoint = None  # Start TCP is optional diagnostic data, not a motion gate.
        speed = self._speed("start_angle_speed")
        gripper = self.cfg.get("gripper", {})
        open_speed = gripper.get("open_speed", gripper.get("speed", 80))
        wait = gripper.get("wait_s", self.cfg.get("fallback_wait_s", 4.0))
        if (type(open_speed) is not int or not 1 <= open_speed <= 100
                or gripper.get("open_state", 0) != 0
                or isinstance(wait, bool) or not isinstance(wait, (int, float))
                or not math.isfinite(wait) or wait < 0):
            raise RobotError("invalid_initialization_gripper_settings")
        self._before_command()
        if self._gripper_open_confirmed is True:
            self._event("initialization", "initialization_open_reused", holding=False)
        else:
            self.command_count += 1
            self.open_gripper()
        actual = self.move_joint_target(angles, endpoint_pose=endpoint,
                                       label="initialization_before_capture", speed=speed)
        settle_s = float(self.cfg.get("start_settle_s", 0.0))
        if settle_s > 0:
            self.sleep(settle_s)
        self._event("initialization", "start_verified_before_capture", actual_angles=actual,
                    completion_basis="fixed_wait", arrival_verified=False)
        self.at_start = True
        return {"control_completed": True, "start_verified": True}

    def _program(self, grasp, place, initialize, return_to_start=False):
        if type(return_to_start) is not bool:
            raise RobotError("return_to_start_must_be_boolean")
        for pose in (grasp.pregrasp_pose_mm_deg, grasp.grasp_pose_mm_deg, grasp.lift_pose_mm_deg,
                     place.pre_place_pose_mm_deg, place.place_pose_mm_deg, place.retreat_pose_mm_deg):
            self._pose(pose)
        self._speed("coordinate_speed")
        self._speed("place_joint_speed", self.cfg.get("start_angle_speed", 20))
        g = self.cfg.get("gripper", {})
        for key in ("open_speed", "close_speed"):
            speed = g.get(key, g.get("speed", 80))
            if type(speed) is not int or not 1 <= speed <= 100:
                raise RobotError("invalid_gripper_speed")
        if g.get("open_state", 0) != 0 or g.get("close_state", 1) != 1:
            raise RobotError("invalid_gripper_state_mapping")
        self._gripper_wait_seconds(0)
        self._gripper_wait_seconds(1)
        commands = []
        if initialize:
            commands.append(("open", None, "initialization_open", None))
            if self.cfg.get("move_to_start_before_pick", True):
                angles = _six(self.cfg.get("start_angles_deg"), "start_angles")
                if any(abs(a) > 360 for a in angles):
                    raise RobotError("invalid_start_joint_range")
                endpoint = None
                commands.append(("joint", angles, "initialization", (endpoint, self._speed("start_angle_speed"))))
        reference = getattr(place, "preferred_ik_reference_angles_deg", None)
        commands.append(("transfer", self._pose(grasp.pregrasp_pose_mm_deg), "pick_pregrasp", (reference, None)))
        commands.extend(self._vertical_commands(grasp.pregrasp_pose_mm_deg, grasp.grasp_pose_mm_deg, "grasp_descent", self._mode("approach_coordinate_mode", 1), True))
        commands.append(("close", None, "grasp_close", None))
        commands.extend(self._vertical_commands(grasp.grasp_pose_mm_deg, grasp.lift_pose_mm_deg, "grasp_lift", self._mode("lift_coordinate_mode", 1), False))
        mode = getattr(place, "planner_mode", "cartesian")
        if mode in {"joint", "hybrid", "cartesian"}:
            for key, default in (("transfer_mode", 0), ("place_mode", 1), ("retreat_mode", 1)):
                value = getattr(place, key, default)
                if type(value) is not int or value not in (0, 1):
                    raise RobotError("coordinate_mode_must_be_0_or_1")
            taught = getattr(place, "pre_place_joint_angles_deg", None)
            if taught is not None:
                _six(taught, "pre_place_angles")
                if self._valid_joint_solution(taught) is None: raise RobotError("invalid_taught_pre_place_angles")
            commands.append(("transfer", self._pose(place.pre_place_pose_mm_deg), "place_pre_place", (reference, taught)))
            commands.extend(self._vertical_commands(place.pre_place_pose_mm_deg, place.place_pose_mm_deg, "place_descent", getattr(place, "place_mode", 1), True))
            commands.append(("release", None, "place_release", None))
            commands.extend(self._vertical_commands(place.place_pose_mm_deg, place.retreat_pose_mm_deg, "place_retreat", getattr(place, "retreat_mode", 1), False))
        else:
            raise RobotError("invalid_place_planner_mode")
        if return_to_start:
            angles = _six(self.cfg.get("start_angles_deg"), "start_angles")
            if any(abs(a) > 360 for a in angles):
                raise RobotError("invalid_start_joint_range")
            commands.append(("joint", angles, "completion_return_to_start", (None, self._speed("start_angle_speed"))))
        return commands

    def execute_pick_and_place(self, grasp, place, *, geometry_passed, initialize=True, return_to_start=False):
        """V2 runtime explicitly requests return on its last task; preserve standalone callers."""
        self._require()
        if geometry_passed is not True:
            raise RobotError("current_frame_geometry_pass_required")
        if self.gripper_holding_uncertain:
            raise RobotError("gripper_holding_uncertain")
        commands = self._program(grasp, place, initialize, return_to_start)  # Validate before any command.
        self.transfer_evidence = []
        actual = None
        for kind, target, label, options in commands:
            self._before_command()
            if kind == "pose":
                actual = self.move_pose(target, mode=options[0], label=label, stable_count=options[1], grasp_endpoint=options[2],
                                        vertical_stage=options[3], waypoint_index=options[4], intermediate=options[5])
            elif kind == "joint":
                actual = self.move_joint_target(target, endpoint_pose=options[0], label=label, speed=options[1])
                if label == "completion_return_to_start":
                    self.at_start = True
                    self._event("completion", "empty_hand_returned_to_start", actual_angles=actual, holding=False, at_start=True)
            elif kind == "transfer":
                actual = self.move_hybrid_transfer(target, label=label, reference_angles=options[0], taught_angles=options[1])
            else:
                if kind == "close":
                    self.sleep(0.35)
                    self._before_command()
                    self._event("grasp", "pre_grasp_settled", delay_s=0.35)
                    self.command_count += 1
                    self.close_gripper()
                else:
                    if kind == "open" and self._gripper_open_confirmed is True and not self.gripper_holding_uncertain:
                        self._event("initialization", "initialization_open_reused", holding=False)
                    else:
                        self.command_count += 1
                        self.open_gripper()
                    if kind == "release":
                        self.gripper_holding_uncertain = False
                        self._event("place_release", "object_released_at_place", object_verified=False)
        transfer_records = [e for e in self.transfer_evidence if e.get("record_type") != "vertical_motion"]
        return {"control_completed": True, "actual_feedback": actual,
                "holding": self.gripper_holding_uncertain, "at_start": self.at_start,
                "object_released_commanded": not self.gripper_holding_uncertain,
                "outcome_verified": False, "ik_validated": bool(transfer_records) and all(e["ik_validated"] for e in transfer_records), "path_validated": False,
                "transfer_evidence": deepcopy(self.transfer_evidence),
                "legacy_transit_executed": False,
                "legacy_transit_policy": "retained_asset_not_executed_by_hybrid_transfer",
                "collision_validated_3d": False, "arrival_settings": self.arrival.to_dict(),
                "completion_basis": "fixed_wait", "arrival_verified": False}
