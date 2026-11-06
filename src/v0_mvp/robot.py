from __future__ import annotations
import logging
import math
import time


logger = logging.getLogger(__name__)


class RobotError(RuntimeError):
    pass


class MyCobotController:
    def __init__(self, cfg: dict, backend=None, sleep_fn=time.sleep, *, event_sink=None, checkpoint=None, clock_fn=None):
        self.cfg = cfg
        self.mc = backend
        self.sleep = sleep_fn
        self._last_verified_angles = None
        self._last_verified_coords = None
        self.event_sink = event_sink
        self.checkpoint = checkpoint
        self._motion_clock = clock_fn or time.monotonic
        self._gripper_open_confirmed = False

    def _event(self, stage, event, *, status="ok", **data):
        if self.event_sink:
            self.event_sink(stage, event, status=status, data=data, source={
                "module": "v0_mvp.robot", "function": "_set_gripper_state" if stage == "gripper" else stage,
                "file": "src/v0_mvp/robot.py",
            })

    def _before_command(self):
        # A soft abort suppresses the next command; it does not claim to stop
        # an already moving physical robot. No extra SDK command is issued.
        if self.checkpoint:
            self.checkpoint()

    def connect(self):
        if self.mc is not None:
            return
        try:
            from pymycobot.mycobot import MyCobot
        except ImportError as exc:
            raise RobotError("未安裝 pymycobot，請執行 pip install -r requirements.txt") from exc
        try:
            self.mc = MyCobot(str(self.cfg["port"]), int(self.cfg.get("baud", 115200)))
        except Exception as exc:
            raise RobotError(f"MyCobot 連線失敗: {exc}") from exc

    def _require(self):
        if self.mc is None:
            raise RobotError("Robot 尚未 connect()")

    def _wait(self):
        g = self.cfg.get("gripper", {})
        self.sleep(float(g.get("wait_s", self.cfg.get("fallback_wait_s", 4.0))))

    def _gripper_wait_seconds(self, state):
        g = self.cfg.get("gripper", {})
        key = "open_wait_s" if state == 0 else "close_wait_s" if state == 1 else "wait_s"
        value = g.get(key, g.get("wait_s", self.cfg.get("fallback_wait_s", 4.0)))
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise RobotError("invalid_gripper_wait")
        return float(value)

    def _clear_serial_buffer(self):
        """Discard already-buffered input only at completed synchronous boundaries."""
        try:
            port = getattr(self.mc, "_serial_port", None)
            if port is None:
                return
            waiting = getattr(port, "in_waiting", None)
            if waiting is None and callable(getattr(port, "inWaiting", None)):
                waiting = port.inWaiting()
            if waiting is None or waiting <= 0:
                return
            reset = getattr(port, "reset_input_buffer", None) or getattr(port, "flushInput", None)
            if callable(reset):
                reset()
        except Exception:
            logger.debug("conditional serial buffer clear failed", exc_info=True)

    def _try_get_feedback(self, method_name, attempts: int, retry_delay_s: float):
        """Read fresh six-axis feedback; diagnostic caches are never a fallback."""
        method = getattr(self.mc, method_name, None)
        if not callable(method):
            return None
        attempts = max(0, min(3, int(attempts)))
        for attempt in range(attempts):
            self._clear_serial_buffer()
            try:
                value = method()
                if isinstance(value, (list, tuple)) and len(value) >= 6 and not any(isinstance(x, bool) for x in value[:6]):
                    result = [float(x) for x in value[:6]]
                    if all(math.isfinite(x) for x in result):
                        return result
            except Exception:
                logger.debug("%s() fresh feedback failed", method_name, exc_info=True)
            if attempt < attempts - 1:
                self.sleep(retry_delay_s)
        return None

    def _try_get_angles(self, attempts: int = 1, retry_delay_s: float = 0.08):
        return self._try_get_feedback("get_angles", attempts, retry_delay_s)

    def _try_get_coords(self, attempts: int = 1, retry_delay_s: float = 0.08):
        return self._try_get_feedback("get_coords", attempts, retry_delay_s)

    def _try_get_error_information(self):
        try:
            if hasattr(self.mc, "get_error_information"):
                return self.mc.get_error_information()
        except Exception:
            logger.debug("get_error_information() diagnostic failed", exc_info=True)
        return None

    def _prepare_controller_error(self, label: str):
        """Best-effort controller-error housekeeping before Cartesian motion.

        Legacy ``pymycobot.mycobot.MyCobot`` backends may expose
        ``get_error_information()`` but not ``clear_error_information()``.  A
        controller error can also remain latched after an older command.  Such
        a pre-existing value must not prevent a new waypoint from being tested.

        Clearing is therefore best-effort only.  The returned baseline is used
        for diagnostics; actual Cartesian pose convergence remains the authority
        for PASS/FAIL.
        """
        self._require()
        before = self._try_get_error_information()
        baseline = before
        clear_result = None
        after = None

        if not hasattr(self.mc, "clear_error_information"):
            if before not in (None, 0):
                logger.warning(
                    "ROBOT CONTROLLER ERROR STALE: %s before_clear=%s "
                    "clear_supported=false; continuing with pose verification",
                    label,
                    before,
                )
            else:
                logger.debug(
                    "ROBOT CONTROLLER ERROR RESET SKIPPED: %s clear_supported=false before_clear=%s",
                    label,
                    before,
                )
            return {
                "before": before,
                "after": None,
                "baseline": baseline,
                "clear_result": None,
                "clear_supported": False,
            }

        try:
            clear_result = self.mc.clear_error_information()
            settle_s = max(
                0.0, float(self.cfg.get("controller_error_clear_settle_s", 0.1))
            )
            if settle_s > 0:
                self.sleep(settle_s)
            after = self._try_get_error_information()
            baseline = after if after is not None else before
            if after not in (None, 0):
                logger.warning(
                    "ROBOT CONTROLLER ERROR CLEAR INCOMPLETE: %s before_clear=%s "
                    "after_clear=%s clear_result=%s; continuing with pose verification",
                    label,
                    before,
                    after,
                    clear_result,
                )
            else:
                logger.info(
                    "ROBOT CONTROLLER ERROR RESET: %s before_clear=%s after_clear=%s clear_result=%s",
                    label,
                    before,
                    after,
                    clear_result,
                )
        except Exception as exc:
            # Do not fail motion solely because an optional controller-error
            # housekeeping API is unavailable or broken.  The waypoint still
            # has to prove itself by converging to the requested TCP pose.
            baseline = before
            logger.warning(
                "ROBOT CONTROLLER ERROR CLEAR FAILED: %s before_clear=%s error=%s; "
                "continuing with pose verification",
                label,
                before,
                exc,
            )

        return {
            "before": before,
            "after": after,
            "baseline": baseline,
            "clear_result": clear_result,
            "clear_supported": True,
        }

    @staticmethod
    def _angle_error_deg(actual: float, target: float) -> float:
        """Return the smallest absolute angular difference in degrees."""
        return abs((float(actual) - float(target) + 180.0) % 360.0 - 180.0)

    @classmethod
    def _pose_errors(cls, target, actual):
        if actual is None or len(actual) < 6:
            return math.inf, math.inf
        position_error_mm = math.sqrt(
            sum((float(actual[i]) - float(target[i])) ** 2 for i in range(3))
        )
        orientation_error_deg = max(
            cls._angle_error_deg(actual[i], target[i]) for i in range(3, 6)
        )
        return position_error_mm, orientation_error_deg

    def _verify_cartesian_pose(self, label: str, target, actual, *, position_tolerance_mm=None, orientation_tolerance_deg=None):
        pos_tol = float(self.cfg.get("pose_verify_position_tolerance_mm", 10.0) if position_tolerance_mm is None else position_tolerance_mm)
        ori_tol = float(self.cfg.get("pose_verify_orientation_tolerance_deg", 5.0) if orientation_tolerance_deg is None else orientation_tolerance_deg)
        pos_err, ori_err = self._pose_errors(target, actual)
        if pos_err > pos_tol or ori_err > ori_tol:
            raise RobotError(
                f"{label} 實際 pose 超出容許誤差："
                f"position_error_mm={pos_err:.3f} (tol={pos_tol}), "
                f"orientation_error_deg={ori_err:.3f} (tol={ori_tol}), "
                f"commanded={list(target)}, actual={actual}, "
                f"controller_error={self._try_get_error_information()}"
            )
        return pos_err, ori_err

    def _raise_waypoint_failure(self, label: str, *, commanded, result, actual=None, mode=None):
        controller_error = self._try_get_error_information()
        mode_text = "" if mode is None else f", mode={mode}"
        raise RobotError(
            f"{label} 未到達：sync_result={result}{mode_text}, "
            f"commanded={list(commanded)}, actual={actual}, "
            f"controller_error={controller_error}"
        )

    def move_start(self):
        """Move to the configured joint start pose and fail closed on timeout."""
        self._require()
        angles = [float(x) for x in self.cfg["start_angles_deg"]]
        speed = int(self.cfg.get("start_angle_speed", 30))
        timeout = float(self.cfg.get("motion_timeout_s", 12))

        self._before_command()
        self._event("move_start", "robot_start_started", commanded_angles=angles, speed=speed)
        deadline = self._motion_clock() + timeout
        if hasattr(self.mc, "sync_send_angles"):
            result = self.mc.sync_send_angles(angles, speed, timeout=timeout)
            if result != 1:
                self._raise_waypoint_failure(
                    "start pose",
                    commanded=angles,
                    result=result,
                    actual=self._try_get_angles(),
                )
        else:
            if not hasattr(self.mc, "send_angles"):
                raise RobotError("目前 MyCobot backend 不支援 send_angles()")
            self.mc.send_angles(angles, speed)
            result = None

        # A send result or elapsed wait alone is never arrival evidence.
        actual = None
        tolerance = float(self.cfg.get("place_joint_verify_tolerance_deg", 5.0))
        poll_s = max(0.05, float(self.cfg.get("motion_poll_interval_s", 0.5)))
        while self._motion_clock() < deadline:
            self._before_command()
            actual = self._try_get_angles()
            if self._motion_clock() >= deadline:
                break
            if actual is not None and max(self._angle_error_deg(a, b) for a, b in zip(actual, angles)) <= tolerance:
                break
            self.sleep(min(poll_s, max(0.0, deadline - self._motion_clock())))
        else:
            actual = None
        if (self._motion_clock() >= deadline or actual is None
                or max(self._angle_error_deg(a, b) for a, b in zip(actual, angles)) > tolerance):
            self._raise_waypoint_failure("start pose", commanded=angles, result=result, actual=actual)

        logger.info("ROBOT WAYPOINT OK: start commanded=%s actual=%s", angles, actual)
        self._last_verified_angles = list(actual) if actual else None
        self._last_verified_coords = self._try_get_coords()
        self._event("move_start", "robot_start_completed", commanded_angles=angles, actual_angles=actual, speed=speed)
        return actual

    def move_pose(self, pose, *, mode: int | None = None, label: str = "pose"):
        """Send one Cartesian waypoint and verify arrival with project tolerances.

        Do not use pymycobot.sync_send_coords() here.  That helper waits on the
        firmware/SDK is_in_position() criterion, which can time out even when the
        measured TCP pose is already inside this project's verified Cartesian
        tolerance.  We therefore send once with send_coords() and poll get_coords()
        ourselves.
        """
        self._require()
        coords = [float(x) for x in pose]
        speed = int(self.cfg.get("coordinate_speed", 15))
        if mode is None:
            mode = int(self.cfg.get("coordinate_mode", 0))
        mode = int(mode)
        timeout = float(self.cfg.get("motion_timeout_s", 12))
        poll_s = max(0.05, float(self.cfg.get("motion_poll_interval_s", 0.2)))
        pos_tol = float(self.cfg.get("pose_verify_position_tolerance_mm", 10.0))
        ori_tol = float(self.cfg.get("pose_verify_orientation_tolerance_deg", 5.0))
        stable_required = max(
            1, int(self.cfg.get("pose_verify_stable_polls", 3))
        )

        if mode not in (0, 1):
            raise RobotError(f"coordinate mode 必須是 0 或 1，收到 {mode}")
        if not hasattr(self.mc, "send_coords"):
            raise RobotError("目前 MyCobot backend 不支援 send_coords()")

        # Controller-error cleanup is best-effort.  Legacy backends may not be
        # able to clear a latched historical error, so pose convergence is the
        # primary PASS/FAIL criterion.
        self._before_command()
        error_state = self._prepare_controller_error(label)
        baseline_error = error_state.get("baseline")
        self._before_command()
        phase = "approach_waypoint" if label.startswith("approach") else label.split()[0]
        self._event("move_pose", phase + "_started", status="warning" if baseline_error not in (None, 0) else "ok", label=label, commanded_pose=coords, speed=speed, mode=mode, controller_error_lifecycle=error_state)
        deadline = self._motion_clock() + timeout
        send_result = self.mc.send_coords(coords, speed, mode)
        actual = None
        last_pos_err = math.inf
        last_ori_err = math.inf
        stable_count = 0
        last_reported_controller_error = None
        controller_error = baseline_error

        attempt = 0
        while self._motion_clock() < deadline:
            self._before_command()
            attempt += 1
            actual = self._try_get_coords()
            if self._motion_clock() >= deadline:
                break
            last_pos_err, last_ori_err = self._pose_errors(coords, actual)
            in_tolerance = last_pos_err <= pos_tol and last_ori_err <= ori_tol
            if in_tolerance:
                stable_count += 1
            else:
                stable_count = 0

            controller_error = self._try_get_error_information()
            if self._motion_clock() >= deadline:
                break
            self._event("move_pose", "pose_polled", label=label, commanded_pose=coords,
                        actual_pose=actual, position_error_mm=last_pos_err,
                        orientation_error_deg=last_ori_err, stable_polls=stable_count,
                        stable_required=stable_required, poll=attempt, speed=speed, mode=mode,
                        send_result=send_result, controller_error=controller_error,
                        controller_error_lifecycle=error_state)
            if (
                controller_error not in (None, 0)
                and controller_error != baseline_error
                and controller_error != last_reported_controller_error
            ):
                logger.warning(
                    "ROBOT CONTROLLER ERROR NEW: %s error=%s baseline_error=%s mode=%d "
                    "commanded=%s actual=%s position_error_mm=%.3f orientation_error_deg=%.3f; "
                    "continuing until pose converges or timeout",
                    label,
                    controller_error,
                    baseline_error,
                    mode,
                    coords,
                    actual,
                    last_pos_err,
                    last_ori_err,
                )
                last_reported_controller_error = controller_error

            if stable_count >= stable_required:
                self._event("move_pose", phase + "_completed", label=label,
                            commanded_pose=coords, actual_pose=actual, mode=mode,
                            position_error_mm=last_pos_err, orientation_error_deg=last_ori_err,
                            stable_polls=stable_count, stable_required=stable_required,
                            controller_error=controller_error, controller_error_lifecycle=error_state)
                logger.info(
                    "ROBOT WAYPOINT OK: %s strategy=send_coords mode=%d commanded=%s actual=%s "
                    "position_error_mm=%.3f orientation_error_deg=%.3f poll=%d stable=%d/%d "
                    "send_result=%s baseline_controller_error=%s controller_error=%s",
                    label,
                    mode,
                    coords,
                    actual,
                    last_pos_err,
                    last_ori_err,
                    attempt,
                    stable_count,
                    stable_required,
                    send_result,
                    baseline_error,
                    controller_error,
                )
                self._last_verified_coords = list(actual) if actual else None
                angles_now = self._try_get_angles()
                if angles_now:
                    self._last_verified_angles = list(angles_now[:6])
                return actual

            remaining = deadline - self._motion_clock()
            if remaining > 0:
                self.sleep(min(poll_s, remaining))

        raise RobotError(
            f"{label} Cartesian motion timeout: timeout_s={timeout}, mode={mode}, "
            f"send_result={send_result}, commanded={coords}, actual={actual}, "
            f"position_error_mm={last_pos_err:.3f} (tol={pos_tol}), "
            f"orientation_error_deg={last_ori_err:.3f} (tol={ori_tol}), "
            f"stable={stable_count}/{stable_required}, baseline_controller_error={baseline_error}, "
            f"controller_error={self._try_get_error_information()}"
        )

    def _build_vertical_waypoints(self, start_pose, end_pose, *, fine_near_end: bool):
        return build_vertical_waypoints(self.cfg, start_pose, end_pose, fine_near_end=fine_near_end)

    def move_vertical_segmented(
        self,
        start_pose,
        end_pose,
        *,
        label: str,
        fine_near_end: bool,
        mode: int = 0,
    ):
        """Execute validated vertical targets using the configured SDK mode."""
        self._require()
        _, mode, _ = vertical_motion_settings(self.cfg)
        waypoints = self._build_vertical_waypoints(
            start_pose, end_pose, fine_near_end=fine_near_end
        )
        if not waypoints:
            return self._try_get_coords()

        actual = None
        total = len(waypoints)
        for index, waypoint in enumerate(waypoints, start=1):
            segment_label = f"{label}[{index}/{total}]"
            actual = self.move_pose(
                waypoint,
                mode=mode,
                label=segment_label,
            )

        logger.info(
            "ROBOT SEGMENTED CARTESIAN COMPLETE: %s mode=%d segments=%d start_z=%.3f end_z=%.3f",
            label,
            mode,
            total,
            float(start_pose[2]),
            float(end_pose[2]),
        )
        return actual

    def get_coords(self):
        self._require()
        coords = self.mc.get_coords()
        if not coords or len(coords) < 3:
            raise RobotError(f"get_coords() 回傳異常: {coords}")
        return [float(x) for x in coords]

    def release_all_servos(self):
        """Release all robot servos for manual calibration positioning."""
        self._require()
        if not hasattr(self.mc, "release_all_servos"):
            raise RobotError("目前 MyCobot backend 不支援 release_all_servos()")
        try:
            return self.mc.release_all_servos()
        except Exception as exc:
            raise RobotError(f"釋放 MyCobot 伺服器失敗: {exc}") from exc

    def _set_gripper_state(self, state: int, *, speed: int | None = None):
        """Open/close the gripper using the native MyCobot gripper-state API."""
        self._require()
        if not hasattr(self.mc, "set_gripper_state"):
            raise RobotError("目前 MyCobot backend 不支援 set_gripper_state()")
        g = self.cfg.get("gripper", {})
        state = int(state)
        if state not in (0, 1, 254):
            raise RobotError(f"gripper state 必須是 0(open)、1(close) 或 254(release)，收到 {state}")

        if speed is None:
            if state == int(g.get("open_state", 0)):
                speed = int(g.get("open_speed", g.get("speed", 80)))
            elif state == int(g.get("close_state", 1)):
                speed = int(g.get("close_speed", g.get("speed", 80)))
            else:
                speed = int(g.get("speed", 80))
        speed = int(speed)

        wait = self._gripper_wait_seconds(state)
        self._before_command()
        self._gripper_open_confirmed = False
        if hasattr(self, "_vertical_continuation"):
            self._vertical_continuation = None
        self._event("gripper", "gripper_commanded", gripper_state=state, speed=speed)
        try:
            result = self.mc.set_gripper_state(state, speed)
            from numbers import Real
            if result is False or (isinstance(result, Real) and result < 0):
                raise RobotError(f"gripper_dispatch_rejected: {result!r}")
        except Exception as exc:
            raise RobotError(f"set_gripper_state(state={state}, speed={speed}) 失敗: {exc}") from exc
        self.sleep(wait)
        self._clear_serial_buffer()
        self._gripper_open_confirmed = state == 0
        self._event("gripper", "gripper_opened" if state == 0 else "gripper_closed", gripper_state=state, speed=speed)
        logger.info("ROBOT GRIPPER OK: state=%d speed=%d", state, speed)

    def open_gripper(self, *, speed: int | None = None):
        g = self.cfg.get("gripper", {})
        self._set_gripper_state(int(g.get("open_state", 0)), speed=speed)

    def close_gripper(self, *, speed: int | None = None):
        g = self.cfg.get("gripper", {})
        self._set_gripper_state(int(g.get("close_state", 1)), speed=speed)

    def initialize_for_pick(self):
        """Return to the configured start pose and open the gripper.

        The pipeline calls this before starting camera perception so the arm does
        not occlude the target.  The same method remains available to callers
        that execute a pick directly through ``execute_pick_stage``.
        """
        self._require()
        if bool(self.cfg.get("move_to_start_before_pick", True)):
            self.move_start()
        self.open_gripper()
        logger.info(
            "ROBOT PICK INITIALIZED: start_speed=%s gripper_open_speed=%s",
            self.cfg.get("start_angle_speed", 15),
            self.cfg.get("gripper", {}).get(
                "open_speed", self.cfg.get("gripper", {}).get("speed", 80)
            ),
        )

    def execute_pick_stage(self, plan, stage: str = "full", *, initialize: bool = True):
        """Execute a bounded portion of the pick trajectory.

        Motion strategy:
        - initialization: configured joint start pose, then open gripper
        - start -> pregrasp: Cartesian send_coords mode=0 (non-linear/angular)
        - pregrasp -> grasp: coarse segmented Cartesian send_coords mode=0
        - gripper: set_gripper_state()
        - grasp -> lift: coarse segmented Cartesian send_coords mode=0
        - future place/transfer: continue after lift with mode=0 unless a linear
          segment is specifically required

        No explicit Python-side IK calculation is used in this pick path. The
        controller handles Cartesian motion internally. Every Cartesian target
        is checked against get_coords() before the sequence continues.
        """
        self._require()
        if stage not in {"pregrasp", "approach", "full"}:
            raise RobotError(f"未知 motion stage: {stage}")

        if initialize:
            self.initialize_for_pick()

        pregrasp_mode = int(self.cfg.get("pregrasp_coordinate_mode", 0))
        approach_mode = 0
        lift_mode = 0

        self.move_pose(
            plan.pregrasp_pose_mm_deg,
            mode=pregrasp_mode,
            label="pregrasp",
        )
        if stage == "pregrasp":
            return

        self.move_vertical_segmented(
            plan.pregrasp_pose_mm_deg,
            plan.grasp_pose_mm_deg,
            label="approach",
            fine_near_end=True,
            mode=approach_mode,
        )
        if stage == "approach":
            return

        self.close_gripper()
        actual = self.move_vertical_segmented(
            plan.grasp_pose_mm_deg,
            plan.lift_pose_mm_deg,
            mode=lift_mode,
            label="lift",
            fine_near_end=False,
        )
        logger.info(
            "ROBOT COARSE CARTESIAN COMPLETE: lift mode=%d start_z=%.3f end_z=%.3f",
            lift_mode,
            float(plan.grasp_pose_mm_deg[2]),
            float(plan.lift_pose_mm_deg[2]),
        )
        return actual

    def execute_pick(self, plan):
        return self.execute_pick_stage(plan, stage="full")


def vertical_motion_settings(cfg):
    """One policy for V2, adapter dispatch and waypoint-count diagnostics.

    Modes B/C always send one endpoint. A zero step disables mode A segmentation.
    Older explicit
    approach_strategy=linear remains single-endpoint until new keys are set.
    """
    policy = cfg.get('vertical_motion_mode', 'segmented_mode0')
    if policy not in ('segmented_mode0', 'linear_mode1', 'single_mode0'):
        raise RobotError('invalid_vertical_motion_mode')
    # Existing callers without the new keys retain their legacy coarse step.
    # V2's explicit project setting is 35 mm.
    step = cfg.get('vertical_segment_step_mm', cfg.get('coarse_step_mm', 25.0))
    if type(step) not in (int, float) or not math.isfinite(step) or not 0 <= step <= 500:
        raise RobotError('invalid_vertical_segment_setting')
    if policy == 'segmented_mode0' and 'vertical_segment_step_mm' not in cfg and step == 0:
        raise RobotError('invalid_vertical_segment_setting')
    if 'vertical_motion_mode' not in cfg and 'vertical_segment_step_mm' not in cfg and cfg.get('approach_strategy') == 'linear':
        step = 0.0
    if policy in ('linear_mode1', 'single_mode0'):
        step = 0.0
    return policy, 1 if policy == 'linear_mode1' else 0, float(step)


def build_vertical_waypoints(cfg, start_pose, end_pose, *, fine_near_end: bool):
    """Build conservative Z-only Cartesian targets for segmented motion.

    Each generated pose keeps the end pose's X/Y/orientation and changes
    only Z. The method rejects a request that is not approximately vertical;
    this helper is not a generic path planner.
    """
    start = [float(x) for x in start_pose]
    end = [float(x) for x in end_pose]
    if len(start) < 6 or len(end) < 6:
        raise RobotError("分段垂直移動需要完整 6D pose")
    if not all(math.isfinite(x) for x in start + end):
        raise RobotError('vertical_pose_requires_finite_values')

    xy_tol = float(cfg.get("segmented_xy_tolerance_mm", 2.0))
    ori_tol = float(cfg.get("segmented_orientation_tolerance_deg", 2.0))
    xy_delta = math.hypot(end[0] - start[0], end[1] - start[1])
    ori_delta = max(MyCobotController._angle_error_deg(end[i], start[i]) for i in range(3, 6))
    if xy_delta > xy_tol or ori_delta > ori_tol:
        raise RobotError(
            "分段 Cartesian 只允許近似垂直 move："
            f"xy_delta_mm={xy_delta:.3f} (tol={xy_tol}), "
            f"orientation_delta_deg={ori_delta:.3f} (tol={ori_tol}), "
            f"start={start}, end={end}"
        )

    # fine_near_end is retained for caller compatibility; microsteps are retired.
    _, _, coarse = vertical_motion_settings(cfg)

    dz = end[2] - start[2]
    if abs(dz) < 1e-9:
        return []
    # Equal partitions bounded by the requested step; never a remainder microstep.
    steps = max(1, int(math.ceil(abs(dz) / coarse))) if coarse else 1
    waypoints = []
    for index in range(1, steps + 1):
        pose = list(end[:6])
        pose[2] = end[2] if index == steps else start[2] + dz * index / steps
        waypoints.append(pose)

    return waypoints
