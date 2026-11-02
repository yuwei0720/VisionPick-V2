"""Operator-console motion adapter.

The frozen V6 controller remains the default implementation. This adapter adds
explicit operator-selected approach behavior and a best-effort software stop
without changing the legacy CLI motion path.
"""
from __future__ import annotations

import logging
import math
from numbers import Real
import time

from v0_mvp.robot import MyCobotController, RobotError, build_vertical_waypoints, vertical_motion_settings
from .preferred_ik import deduplicate_candidates, score_candidate
from .hybrid_transfer import move_hybrid_transfer

logger = logging.getLogger(__name__)


def dispatch_result_failed(result):
    """SDK sends may return 0/None; only False or a negative code fails."""
    return result is False or (isinstance(result, Real) and result < 0)


class OperatorRobotController(MyCobotController):
    """V6 controller with console-only policy extensions.

    ``segmented`` uses shared coarse vertical mode=0 targets.
    ``linear`` sends one verified mode=0 Cartesian approach target. All existing
    workspace, calibration, pose-tolerance, stable-poll and motion-authorization
    gates remain outside or inside the inherited controller.
    """

    def __init__(self, *args, clock_fn=time.monotonic, workspace_mm=None, **kwargs):
        super().__init__(*args, clock_fn=clock_fn, **kwargs)
        self._feedback_clock = clock_fn
        self._telemetry_label = None
        self._hybrid_workspace = workspace_mm or self.cfg.get("workspace_mm")
        self.transfer_evidence = []

    def move_hybrid_transfer(self, pose, *, label, reference_angles=None, taught_angles=None):
        return move_hybrid_transfer(self, pose, label=label, reference_angles=reference_angles, taught_angles=taught_angles)

    def _sample_feedback(self, label, *, coords=None, tcp_time=None):
        if tcp_time is None:
            coords = super()._try_get_coords()
            tcp_time = self._feedback_clock()
        angles = super()._try_get_angles()
        joint_time = self._feedback_clock()
        self._event("telemetry", "motion_telemetry", label=label, coords=coords, angles=angles,
            tcp_time_s=tcp_time, joints_time_s=joint_time, sample_time_s=joint_time,
            source=self.cfg.get("feedback_source", "controller_feedback" if type(self.mc).__module__.startswith("pymycobot") else "synthetic_feedback"))
        return angles

    def _try_get_coords(self):
        coords = super()._try_get_coords()
        t = self._feedback_clock()
        if self._telemetry_label:
            self._sample_feedback(self._telemetry_label, coords=coords, tcp_time=t)
        return coords

    def move_pose(self, pose, *, mode=None, label="pose"):
        if not self.cfg.get("collect_motion_metrics", False):
            return super().move_pose(pose, mode=mode, label=label)
        previous = self._telemetry_label
        self._telemetry_label = label
        try:
            self._sample_feedback(label)
            return super().move_pose(pose, mode=mode, label=label)
        finally:
            self._telemetry_label = previous

    def move_start(self):
        settle_s = float(self.cfg.get("start_settle_s", 0.0))
        if not self.cfg.get("collect_motion_metrics", False):
            actual = super().move_start()
            if settle_s > 0:
                self.sleep(settle_s)
            return actual
        target = list(self.cfg["start_angles_deg"])
        speed = int(self.cfg.get("start_angle_speed", 15))
        self._event("move_start", "robot_start_started", commanded_angles=target, speed=speed)
        actual = self.move_joint_target(target, label="initialization", speed=speed, event_stage="move_start")
        if settle_s > 0:
            self.sleep(settle_s)
        self._event("move_start", "robot_start_completed", commanded_angles=target, actual_angles=actual, speed=speed)
        return actual

    def _send_sampled_joint_motion(self, target, speed, *, label, timeout, tolerance, event_stage='place_transfer'):
        def dispatch(method, *args, **kwargs):
            result = None
            try:
                result = method(*args, **kwargs)
                if dispatch_result_failed(result):
                    raise RobotError(f'{label} rejected send result: {result!r}')
            except Exception as exc:
                self._event(event_stage, 'dispatch_failed', status='error', label=label,
                    target=target, error_type=type(exc).__name__, send_result=result)
                raise RobotError(f'{label}_dispatch_failed') from exc
            return result

        if not self.cfg.get("collect_motion_metrics", False):
            if callable(getattr(self.mc, "sync_send_angles", None)):
                return dispatch(self.mc.sync_send_angles, target, speed, timeout=timeout)
            if callable(getattr(self.mc, "send_angles", None)):
                result = dispatch(self.mc.send_angles, target, speed)
                self._wait()
                return result
            raise RobotError("目前 MyCobot backend 不支援 joint-space transfer")
        self._sample_feedback(label)
        self._before_command()
        if not callable(getattr(self.mc, "send_angles", None)):
            if not callable(getattr(self.mc, "sync_send_angles", None)):
                raise RobotError("目前 MyCobot backend 不支援 joint-space transfer")
            result = dispatch(self.mc.sync_send_angles, target, speed, timeout=timeout)
            self._sample_feedback(label)
            return result
        dispatch(self.mc.send_angles, target, speed)
        poll_s = max(.05, float(self.cfg.get("motion_poll_interval_s", .2)))
        stable_required = max(1, int(self.cfg.get("pose_verify_stable_polls", 3)))
        stable = 0
        started = self._feedback_clock()
        for _ in range(max(1, int(math.ceil(timeout / poll_s)))):
            self._before_command()
            actual = self._valid_joint_solution(self._sample_feedback(label))
            within = actual is not None and max(self._angle_error_deg(actual[i], target[i]) for i in range(6)) <= tolerance
            firmware_check = getattr(self.mc, "is_in_position", None)
            if within and callable(firmware_check):
                try:
                    within = firmware_check(target, 0) == 1
                except Exception:
                    within = False
            stable = stable + 1 if within else 0
            if stable >= stable_required:
                return 1
            if self._feedback_clock() - started >= timeout:
                break
            self.sleep(poll_s)
        raise RobotError(f'{label} joint motion failed: verification timed out')

    def initialize_for_pick(self):
        """Console initialization: open the gripper first, then move to Start.

        The protected V6 controller is left byte-identical. This console-only
        override implements the operator-requested initialization order while
        preserving the existing start pose, speed and gripper parameters.
        """
        self._require()
        self.open_gripper()
        if bool(self.cfg.get("move_to_start_before_pick", True)):
            self.move_start()
        logger.info(
            "ROBOT CONSOLE INITIALIZED: gripper_open_first=True start_speed=%s",
            self.cfg.get("start_angle_speed", 15),
        )

    def enter_free_mode(self):
        """Enter manufacturer-supported manual/free teaching mode."""
        self._require()
        method = getattr(self.mc, "set_free_mode", None)
        try:
            if callable(method):
                result = method(1)
                used = "set_free_mode"
            else:
                fallback = getattr(self.mc, "release_all_servos", None)
                if not callable(fallback):
                    raise RobotError("目前 MyCobot backend 不支援 set_free_mode() 或 release_all_servos()")
                result = fallback()
                used = "release_all_servos"
        except RobotError:
            raise
        except Exception as exc:
            raise RobotError(f"進入示教／Free mode 失敗: {exc}") from exc
        self._event("teaching", "robot_free_mode_enabled", status="warning", method=used, result=result)
        return {"method": used, "result": result}

    def exit_free_mode(self):
        """Exit manual/free mode and restore servo torque without moving."""
        self._require()
        actions = []
        try:
            mode = getattr(self.mc, "set_free_mode", None)
            if callable(mode):
                actions.append({"method": "set_free_mode", "result": mode(0)})
            focus_all = getattr(self.mc, "focus_all_servos", None)
            if callable(focus_all):
                actions.append({"method": "focus_all_servos", "result": focus_all()})
            elif not actions:
                focus_one = getattr(self.mc, "focus_servo", None)
                if not callable(focus_one):
                    raise RobotError("目前 MyCobot backend 不支援離開 Free mode 所需的 servo enable API")
                for servo_id in range(1, 7):
                    actions.append({"method": f"focus_servo({servo_id})", "result": focus_one(servo_id)})
        except RobotError:
            raise
        except Exception as exc:
            raise RobotError(f"離開示教／Free mode 失敗: {exc}") from exc
        self._event("teaching", "robot_free_mode_disabled", method_chain=actions)
        return {"actions": actions}

    def move_vertical_segmented(self, start_pose, end_pose, *, label, fine_near_end, mode=0):
        policy, sdk_mode, _ = vertical_motion_settings(self.cfg)
        record = {'record_type':'vertical_motion', 'label':label,
            'vertical_motion_mode':policy, 'sdk_mode':sdk_mode,
            'motion_completed':False, 'ik_validated':False, 'path_validated':False}
        self.transfer_evidence.append(record)
        actual = super().move_vertical_segmented(start_pose, end_pose, label=label,
            fine_near_end=fine_near_end, mode=sdk_mode)
        record['motion_completed'] = True
        self._event('motion_profile','vertical_motion_completed', **record)
        return actual

    def execute_pick_stage(
        self, plan, stage: str = "full", *, initialize: bool = True,
        preferred_ik_reference_angles=None, preferred_ik_extra_seed=None,
    ):
        self._require()
        if stage not in {"pregrasp", "approach", "full"}:
            raise RobotError(f"未知 motion stage: {stage}")

        if initialize:
            self.initialize_for_pick()

        pregrasp_mode = int(self.cfg.get("pregrasp_coordinate_mode", 0))
        _, approach_mode, _ = vertical_motion_settings(self.cfg)
        lift_mode = approach_mode
        strategy = str(self.cfg.get("approach_strategy", "segmented"))
        if strategy not in {"segmented", "linear"}:
            raise RobotError(f"未知 approach strategy: {strategy}")

        self._event(
            "motion_profile",
            "motion_profile_selected",
            strategy=strategy,
            coordinate_speed=int(self.cfg.get("coordinate_speed", 15)),
            start_speed=int(self.cfg.get("start_angle_speed", 15)),
        )

        self.move_hybrid_transfer(plan.pregrasp_pose_mm_deg, label="pick_pregrasp",
            reference_angles=preferred_ik_reference_angles)
        if stage == "pregrasp":
            return

        started = time.monotonic()
        actual = self.move_vertical_segmented(
            plan.pregrasp_pose_mm_deg, plan.grasp_pose_mm_deg,
            label="approach", fine_near_end=True, mode=approach_mode)
        self._event(
            "motion_profile",
            "approach_strategy_completed",
            strategy=strategy,
            elapsed_s=round(time.monotonic() - started, 3),
        )
        if stage == "approach":
            return actual

        self.close_gripper()
        actual = self.move_vertical_segmented(
            plan.grasp_pose_mm_deg, plan.lift_pose_mm_deg,
            mode=lift_mode, label="lift", fine_near_end=False,
        )
        logger.info(
            "ROBOT COARSE CARTESIAN COMPLETE: lift mode=%d start_z=%.3f end_z=%.3f",
            lift_mode,
            float(plan.grasp_pose_mm_deg[2]),
            float(plan.lift_pose_mm_deg[2]),
        )
        return actual


    @staticmethod
    def _valid_joint_solution(value):
        if not isinstance(value, (list, tuple)) or len(value) < 6:
            return None
        out = []
        for item in value[:6]:
            if isinstance(item, bool):
                return None
            try:
                number = float(item)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(number) or abs(number) > 360:
                return None
            out.append(number)
        return out

    def _normalize_ik_solution(self, raw):
        """Normalize IK output across maintained and legacy pymycobot backends.

        ``pymycobot.mycobot.MyCobot`` (the legacy class still used by the
        frozen controller) returns ``SOLVE_INV_KINEMATICS`` values in the
        protocol's centi-degree integer representation.  The maintained
        ``MyCobot280`` class decodes that response to degrees before returning
        it.  v0.8 treated the legacy integers as degrees and therefore rejected
        valid solutions such as ``5255`` (= 52.55 deg) as >360 deg.

        Only the known legacy class receives the centi-degree compatibility
        conversion.  Other backends must continue returning normal degree
        values, so a malformed large result still fails closed.
        """
        module = getattr(self.mc.__class__, "__module__", "")
        name = getattr(self.mc.__class__, "__name__", "")
        legacy_mycobot = module == "pymycobot.mycobot" and name == "MyCobot"

        if legacy_mycobot:
            if not isinstance(raw, (list, tuple)) or len(raw) < 6:
                return None, "legacy_centidegree_invalid"
            decoded = []
            for item in raw[:6]:
                if isinstance(item, bool):
                    return None, "legacy_centidegree_invalid"
                try:
                    number = float(item) / 100.0
                except (TypeError, ValueError):
                    return None, "legacy_centidegree_invalid"
                if not math.isfinite(number):
                    return None, "legacy_centidegree_invalid"
                decoded.append(round(number, 3))
            return self._valid_joint_solution(decoded), "legacy_centidegree"

        return self._valid_joint_solution(raw), "degrees"

    @staticmethod
    def _ik_solution_suspicious(solution):
        """Reject numeric-looking legacy responses that are clearly implausible.

        Some controller/SDK combinations can return the same sentinel-like
        centi-degree value for all six joints even when IK has failed. A real
        six-axis solution may contain equal joints, so reject only all-zero
        failure placeholders or nearly identical values with large magnitude.
        """
        if not isinstance(solution, (list, tuple)) or len(solution) < 6:
            return True
        values = [float(v) for v in solution[:6]]
        return (all(abs(v) < 1e-9 for v in values)
                or max(values) - min(values) < 0.05 and abs(values[0]) >= 70.0)

    def _controller_error_label(self, code):
        labels = {
            16: "collision_protection_joint_1", 17: "collision_protection_joint_2",
            18: "collision_protection_joint_3", 19: "collision_protection_joint_4",
            32: "inverse_kinematics_no_solution",
            33: "linear_motion_no_adjacent_solution",
            34: "linear_motion_no_adjacent_solution",
        }
        try:
            return labels.get(int(code), "controller_error") if code not in (None, 0) else "ok"
        except Exception:
            return "controller_error"

    def _ik_query(self, method, *args, **kwargs):
        """Space synchronous IK-related queries from the preceding query return."""
        previous = getattr(self, "_last_ik_query_activity", None)
        if previous is not None:
            remaining = 0.04 - (self._feedback_clock() - previous)
            if remaining > 0:
                self.sleep(remaining)
        self._before_command()
        try:
            return method(*args, **kwargs)
        finally:
            self._last_ik_query_activity = self._feedback_clock()

    def _current_ik_attempt(self, target):
        context = getattr(self, "_ik_attempt_context", None)
        if (context is not None and context["backend"] is self.mc and context["target"] == tuple(target)
                and context["dispatch_count"] == getattr(self, "command_count", None)):
            return context
        return None

    def _read_ik_current_angles(self, target):
        """At most three fresh reads per transfer; never consult arrival caches."""
        context = self._current_ik_attempt(target)
        if context is not None and context.get("current_read_attempted"):
            return context.get("current_angles")
        current = None
        for index in range(3):
            current = self._valid_joint_solution(self._ik_query(self._try_get_angles))
            if current is not None:
                break
            if index < 2:
                self.sleep(0.08)
        if context is not None:
            context.update(current_read_attempted=True, current_angles=current)
        return current

    def _solve_ik_seed(self, target, seed, *, label: str, attempt=None):
        attempt = {} if attempt is None else attempt
        attempt.update(raw_solution=None, parsed_angles=None, pre_error=None, post_error=None)
        attempt["pre_error"] = self._ik_query(self._try_get_error_information)
        solver = getattr(self.mc, "solve_inv_kinematics", None)
        if not callable(solver):
            raise RobotError(f"{label} IK unavailable: backend does not expose solve_inv_kinematics()")
        try:
            try:
                raw = self._ik_query(solver, list(target), list(seed[:6]))
            except TypeError:
                raw = self._ik_query(solver, target_coords=list(target), current_angles=list(seed[:6]))
        except Exception as exc:
            code = self._ik_query(self._try_get_error_information)
            attempt["post_error"] = code
            raise RobotError(
                f"{label} IK failed: {exc}; controller_error={code} "
                f"({self._controller_error_label(code)}), target={list(target)}"
            ) from exc
        attempt["raw_solution"] = raw
        solution, solution_format = self._normalize_ik_solution(raw)
        code = self._ik_query(self._try_get_error_information)
        attempt["post_error"] = code
        if solution is not None and self._ik_solution_suspicious(solution):
            solution = None
            solution_format = solution_format + "_suspicious"
        # Legacy MyCobot cannot clear a latched 32. A valid numeric result is
        # authoritative for candidate availability; diagnostics are not arrival proof.
        if solution is None and code == 32:
            solution_format = solution_format + "_controller_no_solution"
        attempt.update(parsed_angles=solution, solution_format=solution_format,
                       controller_error_32_ignored=solution is not None and code == 32)
        return solution, raw, solution_format

    def select_preferred_ik(self, pose, *, reference_angles, label: str, extra_seed=None):
        """Choose among a few seeded IK solutions using a soft posture preference.

        This is intentionally small-scope: it does not perform collision checking
        or path search.  Different seeds are only used to ask the backend for
        alternative IK branches; duplicate results are removed before scoring.
        """
        self._require()
        target = [float(x) for x in pose]
        current = self._read_ik_current_angles(target)
        reference = self._valid_joint_solution(reference_angles)
        if current is None:
            raise RobotError(f"{label} Preferred IK unavailable: current joint angles are invalid")
        if reference is None:
            raise RobotError(f"{label} Preferred IK requires a valid six-axis reference posture")

        seeds = [("current", current), ("preferred_reference", reference)]
        extra = self._valid_joint_solution(extra_seed)
        if extra is not None:
            seeds.append(("safe_transit", extra))
        # Avoid asking the controller the same seeded IK question twice.
        unique_seeds = []
        seen_seeds = set()
        for source, seed in seeds:
            key = tuple(round(float(v), 3) for v in seed[:6])
            if key not in seen_seeds:
                seen_seeds.add(key)
                unique_seeds.append((source, seed))

        self._ik_query(self._prepare_controller_error, label + "_preferred_ik")
        context = self._current_ik_attempt(target)
        scored = []
        attempts = []
        for source, seed in unique_seeds:
            attempt = {"seed_name": source, "seed_angles": list(seed),
                       "seed_source": source, "seed_angles_deg": list(seed)}
            if source == "current" and context is not None and callable(getattr(self.mc, "solve_inv_kinematics", None)):
                context.update(current_attempted=True, current_angles=list(current), current_solution=None)
            try:
                solution, raw, solution_format = self._solve_ik_seed(
                    target, seed, label=f"{label}:{source}", attempt=attempt)
                error = None
            except RobotError as exc:
                solution, raw, solution_format, error = None, None, "error", str(exc)
            attempt.update({
                "raw_solution": raw, "solution_format": solution_format,
                "valid": solution is not None, "error": error,
            })
            attempts.append(attempt)
            if source == "current" and context is not None:
                context["current_solution"] = list(solution) if solution is not None else None
            if solution is not None:
                scored.append(score_candidate(
                    solution, current_angles=current, reference_angles=reference, seed_source=source,
                ))

        scored = deduplicate_candidates(scored)
        if not scored:
            code = self._ik_query(self._try_get_error_information)
            error = RobotError(
                f"{label} Preferred IK found no valid candidate; controller_error={code} "
                f"({self._controller_error_label(code)}), target={target}"
            )
            error.attempts = attempts
            error.details = {"attempts": attempts, "controller_error": code, "target_pose": target}
            raise error
        scored.sort(key=lambda item: item.total_cost)
        selected = scored[0]
        self._event(
            "preferred_ik", "preferred_ik_selected",
            label=label, target_pose=target, current_angles=current, reference_angles=reference,
            attempts=attempts, candidates=[item.as_dict() for item in scored],
            selected=selected.as_dict(),
            scoring_policy={
                "continuity_weight": 0.35, "j2_j3_j4_reference_weight": 0.55,
                "wrist_flip_weight": 0.10, "collision_aware": False,
            },
        )
        return list(selected.angles)

    def preflight_place_ik(self, pose, *, label: str, seed_source: str | None = None):
        """Resolve a place Cartesian pose to joints before any place motion.

        This is deliberately limited to the operator-console place path. It does
        not alter the frozen pick controller. A missing/invalid IK result fails
        closed instead of waiting for a Cartesian timeout (notably controller
        error 32 on MyCobot firmware).
        """
        self._require()
        target = [float(x) for x in pose]
        context = self._current_ik_attempt(target)
        if context is not None and context.get("current_attempted"):
            solution = context.get("current_solution")
            self._event("place_preflight", "standard_ik_current_seed_reused", label=label,
                        target_pose=target, solution_angles=solution, duplicate_query_skipped=True)
            if solution is None:
                raise RobotError(f"{label} unreachable: current seed already queried in this transfer")
            return list(solution)
        solver = getattr(self.mc, "solve_inv_kinematics", None)
        if not callable(solver):
            raise RobotError(f"{label} IK preflight unavailable: backend does not expose solve_inv_kinematics()")
        current = self._read_ik_current_angles(target)
        if not current or len(current) < 6:
            raise RobotError(f"{label} IK preflight unavailable: get_angles() returned {current}")
        # Clear stale diagnostics best-effort, but never treat a clear as proof of reachability.
        lifecycle = self._ik_query(self._prepare_controller_error, label + "_ik_preflight")
        attempt = {"seed_name": seed_source or "current", "seed_angles": list(current[:6])}
        solution, raw, solution_format = self._solve_ik_seed(target, current, label=label, attempt=attempt)
        code = attempt["post_error"]
        if solution is None:
            detail = self._controller_error_label(code)
            error = RobotError(
                f"{label} unreachable: inverse kinematics returned no valid 6-axis solution; "
                f"controller_error={code} ({detail}), target={target}, raw_solution={raw}, "
                f"solution_format={solution_format}"
            )
            error.attempts = [attempt]
            error.details = {"attempts": [attempt], "controller_error": code, "target_pose": target}
            raise error
        self._event(
            "place_preflight", "place_ik_preflight_passed",
            label=label, target_pose=target, current_angles=list(current[:6]),
            seed_source=seed_source or "current_robot_angles",
            raw_solution=raw, solution_format=solution_format, solution_angles=solution,
            controller_error=code, controller_error_lifecycle=lifecycle, attempts=[attempt],
        )
        return solution

    def preflight_place_vertical_path(self, start_pose, end_pose, *, label: str, fine_near_end: bool):
        """IK-check every segmented place waypoint before sending any of them."""
        waypoints = self._build_vertical_waypoints(start_pose, end_pose, fine_near_end=fine_near_end)
        solutions = []
        for index, waypoint in enumerate(waypoints, start=1):
            solutions.append(self.preflight_place_ik(waypoint, label=f"{label}[{index}/{len(waypoints)}]"))
        return {"waypoints": waypoints, "solutions": solutions}

    def move_joint_target(self, angles, *, label: str, speed: int | None = None, event_stage: str = "place_transfer"):
        """Move to a verified joint target for console-only guarded transfer."""
        self._require()
        target = self._valid_joint_solution(angles)
        if target is None:
            raise RobotError(f"{label} invalid joint target: {angles}")
        if speed is None:
            speed = self.cfg.get("place_joint_speed", self.cfg.get("start_angle_speed", 15))
        if type(speed) is not int or not 1 <= speed <= 100:
            raise RobotError("Joint transfer speed must be an integer within 1..100")
        speed = int(speed)
        timeout = float(self.cfg.get("motion_timeout_s", 12.0))
        joint_tol = float(self.cfg.get("place_joint_verify_tolerance_deg", 5.0))
        self._before_command()
        lifecycle = self._prepare_controller_error(label)
        self._event(event_stage, "joint_target_started", status="warning", label=label, commanded_angles=target, speed=speed, controller_error_lifecycle=lifecycle)
        self._send_sampled_joint_motion(target, speed, label=label, timeout=timeout,
            tolerance=joint_tol, event_stage=event_stage)
        actual = self._try_get_angles()
        if not actual or len(actual) < 6:
            raise RobotError(f"{label} joint verification unavailable: actual={actual}")
        max_error = max(self._angle_error_deg(float(actual[i]), target[i]) for i in range(6))
        if max_error > joint_tol:
            code = self._try_get_error_information()
            raise RobotError(f"{label} joint target not reached: max_joint_error_deg={max_error:.3f} (tol={joint_tol}), controller_error={code} ({self._controller_error_label(code)}), commanded={target}, actual={actual}")
        self._last_verified_angles = list(actual[:6])
        self._last_verified_coords = self._try_get_coords()
        self._event(event_stage, "joint_target_completed", label=label, commanded_angles=target, actual_angles=list(actual[:6]), max_joint_error_deg=max_error, speed=speed)
        return list(actual[:6])

    def _release_object_at_place(self, place_plan, *, planner_mode: str) -> None:
        self.open_gripper()
        self._event(
            "place_release",
            "object_released_at_place",
            profile_id=place_plan.profile_id,
            commanded_pose=list(place_plan.place_pose_mm_deg),
            planner_mode=planner_mode,
        )

    def execute_pick_and_place(self, grasp_plan, place_plan, *, initialize: bool = True):
        """Execute the simplified DEV8 pick/place flow.

        The operator-selected place target is authoritative. Safe-transit is
        optional legacy data; hybrid mode transfers directly from the verified
        post-pick posture to the IK-resolved pre-place posture. The global
        approach strategy controls the final Cartesian descent/retreat:
        ``segmented`` uses coarse mode=0 waypoints, while ``linear`` sends one
        verified target. Critical controller/workspace/IK checks remain active.
        """
        self._require()
        self.transfer_evidence = []
        speed = self.cfg.get("coordinate_speed", 15)
        strategy = str(self.cfg.get("approach_strategy", "segmented"))
        if type(speed) is not int or not 1 <= speed <= 100:
            raise RobotError("Cartesian coordinate_speed must stay within 1..100")
        if strategy not in {"segmented", "linear"}:
            raise RobotError(f"Unsupported place descent strategy: {strategy}")

        planner_mode = str(getattr(place_plan, "planner_mode", "hybrid") or "hybrid").lower()
        if planner_mode not in {"joint", "cartesian", "hybrid"}:
            raise RobotError(f"Unsupported Place Planner mode: {planner_mode}")
        safe_transit = self._valid_joint_solution(getattr(place_plan, "safe_transit_joint_angles_deg", None))
        transfer_strategy = "joint_preferred_with_cartesian_fallback"
        self._event(
            "place", "place_sequence_started", status="warning",
            profile_id=place_plan.profile_id, label=place_plan.label,
            planner_mode="hybrid", stored_planner_mode=planner_mode, fixed_robot_frame=True,
            collision_avoidance="guarded_no_general_planner",
            visual_destination_tracking=False, transfer_strategy=transfer_strategy,
            coordinate_speed=speed, descent_strategy=strategy,
            legacy_transit_present=safe_transit is not None, legacy_transit_executed=False,
            legacy_transit_policy="retained_asset_not_executed_by_hybrid_transfer",
        )
        preferred_reference = (
            getattr(place_plan, "preferred_ik_reference_angles_deg", None)
            if bool(getattr(place_plan, "preferred_ik_enabled", False)) else None
        )
        self.execute_pick_stage(
            grasp_plan, stage="full", initialize=initialize,
            preferred_ik_reference_angles=preferred_reference,
            preferred_ik_extra_seed=safe_transit if preferred_reference is not None else None,
        )

        actual = self.move_hybrid_transfer(place_plan.pre_place_pose_mm_deg,
            label="place_pre_place", reference_angles=preferred_reference,
            taught_angles=getattr(place_plan, "pre_place_joint_angles_deg", None))
        self._last_verified_coords = list(actual) if actual else None
        self._event("place_transfer", "place_transfer_completed", profile_id=place_plan.profile_id, actual_pose=actual, strategy=transfer_strategy)

        self._event("place_descent", "place_descent_started", status="warning", profile_id=place_plan.profile_id, commanded_pose=list(place_plan.place_pose_mm_deg), strategy=strategy)
        actual = self.move_vertical_segmented(
            place_plan.pre_place_pose_mm_deg, place_plan.place_pose_mm_deg,
            label="place_descent", fine_near_end=True)
        self._event("place_descent", "place_descent_completed", profile_id=place_plan.profile_id, actual_pose=actual)

        self._release_object_at_place(place_plan, planner_mode=planner_mode)

        actual = self.move_vertical_segmented(
            place_plan.place_pose_mm_deg, place_plan.retreat_pose_mm_deg,
            label="place_retreat", fine_near_end=False)
        self._event("place_retreat", "place_retreat_completed", profile_id=place_plan.profile_id, actual_pose=actual)
        self._event("place", "place_sequence_completed", profile_id=place_plan.profile_id, label=place_plan.label, planner_mode=planner_mode, actual_pose=actual, transfer_strategy=transfer_strategy,
            transfer_evidence=self.transfer_evidence, ik_validated=all(e["ik_validated"] for e in self.transfer_evidence if e.get("record_type") != "vertical_motion"), path_validated=False)
        return actual

    def emergency_stop(self):
        """Send the fastest supported software stop command.

        This is a best-effort software command and is not a substitute for the
        robot's physical emergency-stop circuit. ``stop()`` is preferred;
        ``pause()`` is used only when the backend has no stop method. The gripper
        state is intentionally left unchanged to avoid dropping a held object.
        """
        self._require()
        errors = []
        for method_name in ("stop", "pause"):
            method = getattr(self.mc, method_name, None)
            if not callable(method):
                continue
            try:
                result = method()
                self._event(
                    "safety",
                    "software_emergency_stop_sent",
                    status="warning",
                    method=method_name,
                    result=result,
                    physical_estop_required=True,
                )
                logger.warning("SOFTWARE EMERGENCY STOP SENT: method=%s result=%s", method_name, result)
                return {"sent": True, "method": method_name, "result": result}
            except Exception as exc:  # pragma: no cover - backend specific
                errors.append(f"{method_name}: {exc}")
        detail = "; ".join(errors) if errors else "backend exposes neither stop() nor pause()"
        self._event(
            "safety",
            "software_emergency_stop_failed",
            status="error",
            detail=detail,
            physical_estop_required=True,
        )
        raise RobotError("軟體緊急停止命令無法送出：" + detail)


def planned_vertical_waypoint_count(robot_cfg: dict, start_pose, end_pose, *, fine_near_end: bool) -> int:
    """Count the shared coarse path without connecting a controller."""
    return len(build_vertical_waypoints(robot_cfg, start_pose, end_pose, fine_near_end=fine_near_end))


def planned_approach_waypoint_count(robot_cfg: dict, plan) -> int:
    """Pure telemetry count using the same path builder as motion dispatch."""
    strategy = str(robot_cfg.get("approach_strategy", "segmented"))
    explicit_vertical = 'vertical_motion_mode' in robot_cfg or 'vertical_segment_step_mm' in robot_cfg
    if strategy == "linear" and not explicit_vertical:
        return 1
    if strategy not in {"segmented", "linear"}:
        raise RobotError(f"未知 approach strategy: {strategy}")
    return planned_vertical_waypoint_count(robot_cfg, plan.pregrasp_pose_mm_deg,
        plan.grasp_pose_mm_deg, fine_near_end=True)
