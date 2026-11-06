"""Joint transfer with solver-only fallback; motion failures are never retried."""
import math

from v0_mvp.robot import RobotError


def transfer_pose(controller, pose):
    if not isinstance(pose, (list, tuple)) or len(pose) != 6 or any(isinstance(v, bool) for v in pose):
        raise RobotError("hybrid_target_requires_six_finite_values")
    values = [float(v) for v in pose]
    if not all(math.isfinite(v) for v in values):
        raise RobotError("hybrid_target_requires_six_finite_values")
    workspace = getattr(controller, "_workspace", None) or getattr(controller, "_hybrid_workspace", None)
    if workspace:
        for i, axis in enumerate(("x", "y", "z")):
            if not workspace[axis][0] <= values[i] <= workspace[axis][1]:
                raise RobotError("endpoint_outside_workspace")
    return values, bool(workspace)


def move_hybrid_transfer(controller, pose, *, label, reference_angles=None, taught_angles=None):
    target, bounded = transfer_pose(controller, pose)
    controller._before_command()
    record = {"label": label, "target_pose": target, "workspace_validated": bounded,
              "ik_validated": False, "path_validated": False, "fallback_mode": None,
              "solver_failures": [], "motion_completed": False}
    angles = None
    if taught_angles is not None:
        angles = controller._valid_joint_solution(taught_angles)
        if angles is None:
            raise RobotError("invalid_taught_pre_place_angles")
        record["transfer_mode"] = "taught_joint"
    else:
        reference = reference_angles if reference_angles is not None else controller.cfg.get(
            "preferred_ik_reference_angles_deg", [0., -25., -55., -10., 0., -45.])
        previous_context = getattr(controller, "_ik_attempt_context", None)
        # This context expires before dispatch. It is never an arrival cache and
        # is not shared with another transfer, target, backend or motion command.
        controller._ik_attempt_context = {"backend": controller.mc, "target": tuple(target),
            "dispatch_count": getattr(controller, "command_count", None)}
        try:
            for mode in ("preferred_ik", "standard_ik"):
                controller._before_command()
                try:
                    candidate = (controller.select_preferred_ik(target, reference_angles=reference, label=label)
                                 if mode == "preferred_ik" else controller.preflight_place_ik(target, label=label))
                    angles = controller._valid_joint_solution(candidate)
                    if angles is None:
                        raise RobotError("invalid_ik_joint_solution")
                except Exception as exc:
                    attempts = getattr(exc, "attempts", [])
                    details = getattr(exc, "details", {})
                    record["solver_failures"].append({"mode": mode, "error": str(exc),
                                                      "attempts": attempts, "error_details": details})
                    controller._event("hybrid_transfer", "ik_solver_failed", status="warning",
                                      label=label, solver_mode=mode, error=str(exc), attempts=attempts,
                                      error_details=details, ik_validated=False, path_validated=False)
                controller._before_command()
                if angles is not None:
                    record.update(transfer_mode=mode, ik_validated=mode == "preferred_ik",
                                  fallback_mode="standard_ik" if mode == "standard_ik" else None,
                                  selected_ik_solution_available=True)
                    break
        finally:
            controller._ik_attempt_context = previous_context
    if angles is None:
        if not bounded:
            raise RobotError("cartesian_fallback_requires_workspace_bounds")
        record.update(transfer_mode="cartesian", fallback_mode="cartesian")
    controller.transfer_evidence = getattr(controller, "transfer_evidence", [])
    controller.transfer_evidence.append(record)
    controller._event("hybrid_transfer", "transfer_selected", **record)
    # Dispatch is deliberately outside solver exception handlers: an uncertain
    # physical command must never trigger another motion command as a fallback.
    if angles is not None:
        controller.move_joint_target(angles, label=label)
        policy = getattr(controller, "motion_completion_policy", None)
        if policy in {"fixed_wait", "minimum_wait_then_feedback"}:
            # Reuse V2's joint arrival evidence; this does not prove TCP arrival.
            actual = None
            record.update(completion_basis=policy,
                          arrival_verified=getattr(controller, '_last_arrival_verified', False),
                          arrival_basis='joint_feedback', tcp_arrival_verified=False)
        else:
            actual = controller._try_get_coords()
            if hasattr(controller, "_last_movement_activity"):
                controller._last_movement_activity = controller._feedback_clock()
            pos, ori = controller._verify_cartesian_pose(label, target, actual,
                position_tolerance_mm=35.0, orientation_tolerance_deg=15.0)
            record.update(tcp_position_error_mm=pos, tcp_orientation_error_deg=ori,
                          tcp_position_tolerance_mm=35.0, tcp_orientation_tolerance_deg=15.0)
    else:
        actual = controller.move_pose(target, mode=0, label=label)
        policy = getattr(controller, "motion_completion_policy", None)
        if policy in {"fixed_wait", "minimum_wait_then_feedback"}:
            record.update(completion_basis=policy,
                          arrival_verified=getattr(controller, '_last_arrival_verified', False),
                          arrival_basis='cartesian_feedback')
    record["motion_completed"] = True
    controller._event("hybrid_transfer", "transfer_completed", **record, actual_pose=actual)
    return actual
