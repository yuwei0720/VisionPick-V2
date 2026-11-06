"""One V2 execution loop for the complete bounded plan, with instance selection.

Camera/geometry/controller implementations are explicit injected boundaries.
Optional local checks allow one empty-hand step retry with the original endpoint.
Controller completion remains separate from the observed object result.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math
import threading
import time


class SequenceV2Error(ValueError):
    pass


class _AwaitingInstance(Exception):
    pass


@dataclass(frozen=True)
class ObservationV2:
    frame_id: str
    captured_at_unix_ms: float
    instances: tuple
    payload: object = None
    display: dict | None = None


def _json_copy(value):
    return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))


def eligible_for_role(instance, role):
    """References need measured geometry; targets also need grasp reachability."""
    return instance.get('reference_eligible', instance['eligible']) is True if role == 'reference' else instance['eligible'] is True


class TaskSequenceV2:
    """A single worker advances one whole-plan claim, with no default init policy.

    `prepare(task, observation, selected)` must retain every scene object for
    occupancy checks and return geometry_passed/frame_id plus controller plans.
    `execute(prepared, initialize=..., checkpoint=...)` must call checkpoint
    before subsequent commands and return the direct-controller result contract.
    Snapshots/cancel remain available while camera or controller calls block.
    """

    def __init__(self, pending, *, run_id, observe, prepare, execute,
                 initialization_policy, max_scene_age_ms=30000.0, clock_ms=None,
                 initialize_before_observe=None, step_recovery_enabled=False, persist_boundary=None,
                 check_result=None, observe_completion=None):
        if initialization_policy not in {"each_task", "once"}:
            raise SequenceV2Error("explicit_initialization_policy_required")
        if not isinstance(run_id, str) or not run_id:
            raise SequenceV2Error("run_id_required")
        if (isinstance(max_scene_age_ms, bool) or not isinstance(max_scene_age_ms, (int, float))
                or not math.isfinite(max_scene_age_ms) or max_scene_age_ms <= 0):
            raise SequenceV2Error("invalid_scene_age_limit")
        self.pending = pending
        self.run_id = run_id
        self.observe, self.prepare, self.execute = observe, prepare, execute
        self.check_result, self.observe_completion = check_result, observe_completion
        if initialize_before_observe is not None and not callable(initialize_before_observe):
            raise SequenceV2Error("initialization_callback_must_be_callable")
        self.initialize_before_observe = initialize_before_observe
        if persist_boundary is not None and not callable(persist_boundary):
            raise SequenceV2Error("persistence_callback_must_be_callable")
        self.persist_boundary = persist_boundary
        if type(step_recovery_enabled) is not bool:
            raise SequenceV2Error('step_recovery_capability_must_be_boolean')
        self.step_recovery_enabled = step_recovery_enabled
        self._safe_start_step = None
        self._initialized_step = None
        self._refresh_start_required = False
        self.policy = initialization_policy
        self.max_age = float(max_scene_age_ms)
        self.clock_ms = clock_ms or (lambda: time.time() * 1000.0)
        self._lock = threading.RLock()
        self._cancelled = threading.Event()
        self._busy = False
        self._status = "idle"
        self._claim = None
        self._step = 0
        self._observation = None
        self._last_display = None
        self._selected = {}
        self._question = None
        self._steps = []
        self._history = []
        self._error = None
        self._initial_instances = None
        self._initial_frame_id = None

    def _record(self, event, **data):
        self._history.append({"event": event, "step_id": self._step + 1, **_json_copy(data)})

    def checkpoint(self):
        if self._cancelled.is_set():
            raise SequenceV2Error("sequence_cancelled")

    def _age_check(self, observation):
        age = self.clock_ms() - observation.captured_at_unix_ms
        if not math.isfinite(age) or not 0 <= age <= self.max_age:
            raise SequenceV2Error("scene_snapshot_stale")

    def _validate_observation(self, observation):
        if (not isinstance(observation, ObservationV2) or not isinstance(observation.frame_id, str)
                or not observation.frame_id or isinstance(observation.captured_at_unix_ms, bool)
                or not isinstance(observation.captured_at_unix_ms, (int, float))
                or not math.isfinite(observation.captured_at_unix_ms)):
            raise SequenceV2Error("invalid_observation")
        ids = []
        for instance in observation.instances:
            if (not isinstance(instance, dict) or not isinstance(instance.get("instance_id"), str)
                    or not instance["instance_id"] or not isinstance(instance.get("class_name"), str)
                    or type(instance.get("eligible")) is not bool
                    or ('reference_eligible' in instance and type(instance['reference_eligible']) is not bool)):
                raise SequenceV2Error("invalid_instance_contract")
            ids.append(instance["instance_id"])
        if len(ids) != len(set(ids)):
            raise SequenceV2Error("duplicate_instance_id")
        _json_copy(observation.instances)
        _json_copy(observation.display)
        self._age_check(observation)

    def begin(self, *, version):
        with self._lock:
            if self._status != "idle":
                raise SequenceV2Error("sequence_already_started")
            self._claim = self.pending.claim(version=version)
            self._steps = [{"step_id": i + 1, "task": deepcopy(task), "status": "pending", "outcome_verified": False}
                           for i, task in enumerate(self._claim["plan"]["tasks"])]
            overrides = self._claim['settings'].get('placement_orientation_overrides',{})
            for step in self._steps:
                step['orientation_override'] = deepcopy(overrides.get(str(step['step_id'])))
            self._status = "running"
            self._record("whole_plan_claimed", version=version, initialization_policy=self.policy)
            return self.snapshot()

    def _roles(self):
        step = self._steps[self._step]
        task = step["task"]
        if step.get('retry_count'):
            from integration.relative_placement import match_result_target
            target = match_result_target(step['result_expectation'], self._observation.instances)
            if target is None or not target['eligible']:
                raise SequenceV2Error('retry_target_not_graspable')
            self._selected['target'] = target['instance_id']
            self._selected['locked_destination'] = deepcopy(step['result_expectation'])
        if task['destination_type'] == 'relative' and task['target'] == task['reference']:
            matches = [i for i in self._observation.instances if i['class_name'] == task['target']]
            if not any(t['eligible'] and any(r['instance_id'] != t['instance_id'] and eligible_for_role(r, 'reference') for r in matches) for t in matches):
                if self._pause_missing_step('reference', task['reference'], detected=bool(matches)):
                    return False
                raise SequenceV2Error('no_eligible_reference_instance')
        for role, field in (("target", "target"), ("reference", "reference")):
            if role == "reference" and task["destination_type"] != "relative":
                continue
            if role in self._selected:
                continue
            initial = role == 'reference' and task.get('relation') == 'at_initial'
            source = self._initial_instances if initial else self._observation.instances
            matches = [deepcopy(i) for i in source if i["class_name"] == task[field]]
            for item in matches:
                item['eligible'] = eligible_for_role(item, role)
                if role == 'reference':
                    item['rejection_reason'] = item.get('reference_rejection_reason', item.get('rejection_reason', ''))
            if role == "reference" and not initial:
                for i in matches:
                    if i["instance_id"] == self._selected.get("target"):
                        i.update(eligible=False, rejection_reason="target_and_reference_must_differ")
            if not any(i["eligible"] for i in matches):
                if self._pause_missing_step(role,task[field],detected=bool(matches)):
                    return False
                raise SequenceV2Error(f"no_eligible_{role}_instance")
            if step.get('retry_count') and role == 'reference' and not initial:
                original = step['reference_snapshot']['xyz_mm'][:2]
                matches = [min((i for i in matches if i['eligible']), key=lambda i:
                    math.dist(i['xyz_mm'][:2], original))]
            if len(matches) == 1:
                self._selected[role] = matches[0]["instance_id"]
                if initial:
                    self._selected['initial_reference'] = deepcopy(matches[0])
                self._record("unique_instance", role=role, instance_id=matches[0]["instance_id"])
            else:
                self._status = "waiting_instance"
                self._question = {"run_id": self.run_id, "step_id": self._step + 1,
                    "frame_id": self._observation.frame_id, "plan_version": self._claim["version"],
                    "role": role, "class_name": task[field], "choices": matches,
                    "message": f"第 {self._step + 1} 步：選擇{'要抓取' if role == 'target' else '作為參考'}的{task[field]}；選取後將繼續已確認計畫。"}
                return False
        if task['destination_type'] == 'self_relative' and 'self_anchor' not in self._selected:
            anchor = next(i for i in self._observation.instances if i['instance_id'] == self._selected['target'])
            self._selected['self_anchor'] = {**deepcopy(anchor),'frame_id':self._observation.frame_id}
            self._record('step_pre_pick_snapshot', anchor=self._selected['self_anchor'])
        return True

    def _retry_current_step(self, reason, observation=None):
        step = self._steps[self._step]
        if step.get('retry_count', 0) >= 1:
            raise SequenceV2Error('step_retry_exhausted: ' + reason)
        step.update(retry_count=1, status='pending')
        self._observation = observation
        self._selected = {}
        self._question = None
        self._initialized_step = self._safe_start_step = self._step if observation else None
        self._refresh_start_required = observation is None
        self._record('step_retry_scheduled', reason=reason, retry_count=1)

    def _retry_perception_once(self, error, *, phase):
        """One fresh observation per step/phase, with no motion or plan replay."""
        self.checkpoint()
        reason = getattr(error, 'reason', str(error))
        recoverable = reason in {
            'unreliable_object_depth', 'unreliable_detection_geometry', 'invalid_rgbd_frame',
            'result_target_depth_unavailable', 'result_target_not_observed',
            'destination_depth_unavailable', 'camera_capture_timeout',
        }
        if type(error).__name__ == 'RealSenseError':
            recoverable = '取 frame 失敗' in reason or '不完整的 RGB-D frame' in reason
        if phase == 'before' and reason.startswith('no_eligible_') and self._question:
            matches = [i for i in self._observation.instances if i['class_name'] == self._question['class_name']]
            key = 'reference_rejection_reason' if self._question['role'] == 'reference' else 'rejection_reason'
            recoverable = not matches or all(i.get(key) in {
                'unreliable_object_depth', 'unreliable_detection_geometry', 'invalid_depth'
            } for i in matches)
        step = self._steps[self._step]
        counter = 'perception_recaptures_' + phase
        if not recoverable or step.get(counter, 0) >= 1 or self._safe_start_step != self._step:
            return False
        step[counter] = 1
        if phase == 'before':
            self._observation = None
            self._selected = {}
            self._question = None
            self._status = 'running'
            step['status'] = 'pending'
            if self._step == 0 and 'result_expectation' not in step:
                self._initial_instances = self._initial_frame_id = None
        self._record('perception_recapture_scheduled', phase=phase, reason=reason, count=1)
        return True

    def _check_step_result(self):
        step = self._steps[self._step]
        final = self._step == len(self._steps) - 1
        if not final or step.pop('result_requires_start', False):
            initialized = self.initialize_before_observe(self._step + (1 if final else 2), checkpoint=self.checkpoint)
            self.checkpoint()
            if (initialized.get('control_completed') is not True or initialized.get('start_verified') is not True
                    or initialized.get('empty_hand_confirmed') is not True):
                raise SequenceV2Error('result_check_requires_empty_start')
            self._record('start_verified_before_observation')
        force = False
        previous_frame = None
        previous_time = step['captured_at_unix_ms']
        while True:
            try:
                observed = (self.observe_completion(force=True) if force else self.observe_completion()) if final else self.observe(self._step + 2)
                self.checkpoint()
                self._validate_observation(observed)
                if observed.frame_id == previous_frame or observed.captured_at_unix_ms <= previous_time:
                    raise SequenceV2Error('completion_observation_not_new')
                previous_frame, previous_time = observed.frame_id, observed.captured_at_unix_ms
                checked = self.check_result(step['result_expectation'], observed)
                break
            except (ValueError, RuntimeError) as exc:
                if not self._retry_perception_once(exc, phase='result'):
                    raise
                force = True
        step['local_result_check'] = _json_copy(checked)
        self._record('local_result_checked', frame_id=observed.frame_id, **checked)
        if not checked['passed']:
            self._last_display = _json_copy(observed.display)
            self._retry_current_step('local_result_outside_tolerance', observed)
            return
        with self._lock:
            self.checkpoint()
            step['status'] = 'completed'
            self._record('step_completed', control_completed=step['result']['control_completed'], outcome_verified=False)
            self._step += 1
            self._observation = None if final else observed
            self._selected = {}
            self._question = None
            if final:
                self._status = 'completed'
                self.pending.finish(outcome='completed')
            else:
                self._initialized_step = self._safe_start_step = self._step
                self._last_display = _json_copy(observed.display)
                self._record('observed', frame_id=observed.frame_id)

    def advance(self):
        with self._lock:
            if self._status != "running":
                return self.snapshot()
            if self._busy:
                raise SequenceV2Error("sequence_worker_busy")
            self._busy = True
        try:
            self.checkpoint()
            if self._steps[self._step]['status'] == 'checking_result':
                self._check_step_result()
                raise _AwaitingInstance
            if (self.initialize_before_observe is not None
                    and self._initialized_step != self._step
                    and (self.policy == "each_task" or self._step == 0 or self._refresh_start_required)):
                initialized = self.initialize_before_observe(self._step + 1, checkpoint=self.checkpoint)
                self.checkpoint()
                if (not isinstance(initialized, dict) or initialized.get("control_completed") is not True
                        or initialized.get("start_verified") is not True):
                    raise SequenceV2Error("start_not_verified_before_observation")
                with self._lock:
                    self._initialized_step = self._step
                    self._refresh_start_required = False
                    self._safe_start_step = self._step if initialized.get('empty_hand_confirmed') is True else None
                    self._record("start_verified_before_observation")
            if self._observation is None:
                observation = self.observe(self._step + 1)
                self._validate_observation(observation)
                with self._lock:
                    self.checkpoint()
                    self._observation = observation
                    self._last_display = _json_copy(observation.display)
                    if self._initial_instances is None:
                        frozen = _json_copy(observation.instances)
                        # Resolve all original anchors before any object is moved.
                        for step in self._steps:
                            original = step['task']
                            if original.get('relation') != 'at_initial':
                                continue
                            matches = [i for i in frozen if i['class_name'] == original['reference']]
                            if len(matches) != 1 or not matches[0]['eligible']:
                                if not any(i['eligible'] for i in matches) and self._pause_missing_step('reference',original['reference'],detected=bool(matches)):
                                    raise _AwaitingInstance
                                raise SequenceV2Error('initial_reference_not_unique_or_missing')
                            anchor = matches[0]
                            xyz, polygon = anchor.get('xyz_mm'), anchor.get('footprint_robot_xy_mm')
                            if (not isinstance(xyz, list) or len(xyz) != 3
                                    or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in xyz)
                                    or not isinstance(polygon, list) or len(polygon) != 4
                                    or any(not isinstance(p, list) or len(p) != 2
                                        or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in p)
                                        for p in polygon)):
                                raise SequenceV2Error('initial_reference_geometry_missing')
                            crosses = [(polygon[(i+1)%4][0]-polygon[i][0])*(polygon[(i+2)%4][1]-polygon[(i+1)%4][1])
                                -(polygon[(i+1)%4][1]-polygon[i][1])*(polygon[(i+2)%4][0]-polygon[(i+1)%4][0]) for i in range(4)]
                            if not (all(v>1e-6 for v in crosses) or all(v < -1e-6 for v in crosses)):
                                raise SequenceV2Error('initial_reference_footprint_invalid')
                        self._initial_instances = frozen
                        self._initial_frame_id = observation.frame_id
                        self._record('initial_pose_snapshot', frame_id=observation.frame_id, instances=frozen)
                    self._selected = {}
                    self._record("observed", frame_id=observation.frame_id)
            with self._lock:
                self.checkpoint()
                self._age_check(self._observation)
                if not self._roles():
                    raise _AwaitingInstance
                task = deepcopy(self._steps[self._step]["task"])
                selected = deepcopy(self._selected)
                observation = self._observation
            prepared = self.prepare(task, observation, selected)
            if (not isinstance(prepared, dict) or prepared.get("geometry_passed") is not True
                    or prepared.get("frame_id") != observation.frame_id):
                raise SequenceV2Error("current_frame_geometry_pass_required")
            self.checkpoint()
            self._age_check(observation)
            with self._lock:
                step = self._steps[self._step]
                if self.check_result:
                    step.setdefault('result_expectation', _json_copy(prepared['result_expectation']))
                    if 'reference' in selected and 'reference_snapshot' not in step:
                        step['reference_snapshot'] = deepcopy(selected.get('initial_reference') or next(
                            i for i in observation.instances if i['instance_id'] == selected['reference']))
                step.update(status="executing", frame_id=observation.frame_id, selected=selected,
                            captured_at_unix_ms=observation.captured_at_unix_ms)
                self._record("step_started", frame_id=observation.frame_id, selected=selected)
            # Outside the sequence lock: runtime snapshots also acquire this lock.
            # Persist executing/step_started before execute can dispatch hardware.
            if self.persist_boundary:
                self.persist_boundary()
            self.checkpoint()
            try:
                result = self.execute(prepared, initialize=self.initialize_before_observe is None and (self.policy == "each_task" or self._step == 0),
                                      checkpoint=self.checkpoint)
            except Exception as exc:
                self.checkpoint()  # A cancellation wrapped by the transport is never retryable.
                if self.check_result and getattr(exc, 'retry_empty_hand', False):
                    self._retry_current_step(str(exc))
                    raise _AwaitingInstance
                if self.check_result and getattr(exc, 'check_released_result', False):
                    step.update(status='checking_result', result={'control_completed':False,
                                'outcome_verified':False, 'arrival_verified':False, 'ik_validated':False,
                                'path_validated':False, 'collision_validated_3d':False},
                                result_requires_start=True, motion_error=str(exc))
                    raise _AwaitingInstance
                raise
            self.checkpoint()
            evidence = result.get("transfer_evidence") if isinstance(result, dict) else None
            transfer_records = [e for e in evidence if isinstance(e, dict) and e.get("record_type") != "vertical_motion"] if isinstance(evidence, list) else []
            proven_ik = bool(transfer_records) and all(
                isinstance(e, dict) and e.get("ik_validated") is True and e.get("transfer_mode") == "preferred_ik"
                and e.get("fallback_mode") is None and e.get("motion_completed") is True for e in transfer_records)
            valid_transfers = bool(transfer_records) and all(
                e.get("motion_completed") is True and (
                    e.get("transfer_mode") in {"preferred_ik", "standard_ik", "taught_joint"}
                    or e.get("transfer_mode") in {"cartesian", "fallback_cartesian"}
                    and (e.get("fallback_mode") == "cartesian" or e.get("fallback_used") is True
                         or e.get("transfer_mode") == "fallback_cartesian")) for e in transfer_records)
            malformed_evidence = evidence is not None and (
                not isinstance(evidence, list) or any(not isinstance(e, dict) for e in evidence)
                or any(e.get("motion_completed") is not True for e in evidence)
                or bool(transfer_records) and not valid_transfers)
            if (not isinstance(result, dict) or result.get("control_completed") is not True
                    or type(result.get("ik_validated")) is not bool
                    or malformed_evidence
                    or result.get("ik_validated") is True and not (proven_ik or valid_transfers)
                    or any(result.get(k) is not False for k in ("path_validated", "collision_validated_3d", "outcome_verified"))):
                raise SequenceV2Error("direct_execution_result_contract_invalid")
            if transfer_records:
                # Legal fallback completes control flow without becoming IK proof.
                result = {**result, "ik_validated": proven_ik}
            with self._lock:
                self.checkpoint()
                if self.check_result:
                    step.update(status='checking_result', result=_json_copy(result))
                    raise _AwaitingInstance
                self._steps[self._step].update(status="completed", result=_json_copy(result), outcome_verified=False)
                self._record("step_completed", control_completed=True, outcome_verified=False)
                self._step += 1
                self._observation = None
                self._selected = {}
                self._question = None
                if self._step == len(self._steps):
                    self._status = "completed"
                    self.pending.finish(outcome="completed")
        except _AwaitingInstance:
            if self._status == 'waiting_step_recovery' and self._question:
                self._retry_perception_once(self._question['code'], phase='before')
        except Exception as exc:
            with self._lock:
                before_motion = (self._step < len(self._steps)
                                 and self._steps[self._step]['status'] not in {'executing', 'checking_result'})
                recapturing = (not self._cancelled.is_set() and before_motion
                              and self._retry_perception_once(exc, phase='before'))
                if not self._cancelled.is_set() and not recapturing:
                    self._status = "failed"
                    self._error = {"type": type(exc).__name__, "reason": str(exc), "step_id": self._step + 1}
                    if self._step < len(self._steps):
                        self._steps[self._step].update(status="failed", error=self._error)
                    self._record("step_failed", error=self._error)
                    if self.pending.snapshot()["status"] == "executing":
                        self.pending.finish(outcome="failed")
                self._question = None
        finally:
            with self._lock:
                self._busy = False
        if self.persist_boundary:
            self.persist_boundary()  # completion, failure, instance/recovery waits
        return self.snapshot()

    def select(self, *, run_id, step_id, frame_id, plan_version, role, instance_id):
        with self._lock:
            self.checkpoint()
            if self._busy or self._status != "waiting_instance" or self._question is None:
                raise SequenceV2Error("not_waiting_for_instance")
            q = self._question
            if (run_id != q["run_id"] or type(step_id) is not int or step_id != q["step_id"]
                    or frame_id != q["frame_id"] or type(plan_version) is not int or plan_version != q["plan_version"]
                    or role != q["role"]):
                raise SequenceV2Error("stale_or_wrong_role_selection")
            self._age_check(self._observation)
            match = next((i for i in q["choices"] if i["instance_id"] == instance_id), None)
            if match is None or match["eligible"] is not True:
                raise SequenceV2Error("instance_not_eligible_for_role")
            self._selected[role] = instance_id
            self._record("instance_selected", role=role, instance_id=instance_id, frame_id=frame_id)
            self._question = None
            self._status = "running"
            return self.snapshot()

    def refresh(self):
        with self._lock:
            if self._busy or self._status != "waiting_instance":
                raise SequenceV2Error("not_waiting_for_instance")
            self._observation = None
            self._initialized_step = None
            self._refresh_start_required = True
            self._safe_start_step = None
            self._selected = {}
            self._question = None
            self._status = "running"
            self._record("observation_refresh_requested")
            return self.snapshot()

    def _can_skip_step(self):
        task = self._steps[self._step]['task']
        dependencies = {t[key] for step in self._steps[self._step:] for t in [step['task']]
                        if t.get('relation') == 'at_initial' for key in ('target','reference')}
        return task.get('relation') != 'at_initial' and task['target'] not in dependencies

    def _pause_missing_step(self, role, name, *, detected):
        if not self.step_recovery_enabled or self._safe_start_step != self._step:
            return False
        self._status = 'waiting_step_recovery'
        self._steps[self._step]['status'] = 'waiting_step_recovery'
        self._question = {'run_id':self.run_id,'step_id':self._step+1,
            'frame_id':self._observation.frame_id,'plan_version':self._claim['version'],
            'role':role,'class_name':name,'code':f'no_eligible_{role}_instance',
            'skip_allowed':self._can_skip_step(),'safe_start_verified':True,'empty_hand_confirmed':True,
            'detected_but_ineligible':detected}
        self._record('waiting_step_recovery',**{k:v for k,v in self._question.items() if k!='step_id'})
        return True

    def recover_step(self, action, *, run_id, step_id, frame_id, plan_version):
        with self._lock:
            self.checkpoint()
            if self._busy or self._status != 'waiting_step_recovery' or self._question is None:
                raise SequenceV2Error('not_waiting_for_step_recovery')
            q = self._question
            if (run_id != self.run_id or type(step_id) is not int or step_id != q['step_id']
                    or frame_id != q['frame_id'] or type(plan_version) is not int or plan_version != q['plan_version']):
                raise SequenceV2Error('stale_step_recovery')
            if self._safe_start_step != self._step: raise SequenceV2Error('safe_empty_start_required')
            if action not in {'retry_step','skip_step'}: raise SequenceV2Error('invalid_step_recovery_action')
            if action == 'skip_step':
                if not self._can_skip_step(): raise SequenceV2Error('swap_step_cannot_be_skipped')
                self._steps[self._step].update(status='skipped',skip_reason=q['code'],missing_role=q['role'],missing_class=q['class_name'])
                self._record('step_skipped',reason=q['code'])
                self._step += 1
            else:
                self._steps[self._step]['status'] = 'pending'
                self._record('step_retry_requested')
            self._initialized_step = self._safe_start_step = None
            self._observation = None
            self._selected = {}
            self._question = None
            self._status = 'running'
            if self._step == len(self._steps):
                self._status = 'completed'
                self.pending.finish(outcome='completed')
            return self.snapshot()

    def cancel(self):
        self._cancelled.set()
        with self._lock:
            if self._status not in {"completed", "failed", "cancelled"}:
                self._status = "cancelled"
                self._question = None
                self.pending.cancel()
                self._record("cancelled")
            return self.snapshot()

    def snapshot(self):
        with self._lock:
            return {"run_id": self.run_id, "status": self._status, "worker_busy": self._busy,
                "step_id": min(self._step + 1, len(self._steps)) if self._steps else None,
                "plan_version": self._claim["version"] if self._claim else None, "steps": _json_copy(self._steps),
                "question": _json_copy(self._question), "selected": _json_copy(self._selected),
                "display": _json_copy(self._observation.display if self._observation else self._last_display),
                "initial_pose_snapshot": {"frame_id": self._initial_frame_id, "instances": _json_copy(self._initial_instances)},
                "history": _json_copy(self._history), "error": _json_copy(self._error), "outcome_verified": False}
