"""Hybrid routing and solver failure recovery using fresh controlled feedback."""
from copy import deepcopy
from types import SimpleNamespace
import pytest

from operator_console.direct_control_v2 import DirectRobotControllerV2
from operator_console.robot_adapter import OperatorRobotController
from v0_mvp.robot import RobotError

POSE = [100., 80., 240., -175., -5., 20.]
Q = [1., -20., -50., -10., -5., -45.]
CFG = {"robot": {"coordinate_speed": 15, "start_angle_speed": 15,
    "start_angles_deg": [0, 0, 0, -90, 0, -45], "fallback_wait_s": 0,
    "gripper": {"wait_s": 0}, "motion_timeout_s": 1., "motion_poll_interval_s": .01,
    "approach_strategy": "linear", "pose_verify_position_tolerance_mm": 1.,
    "pose_verify_orientation_tolerance_deg": 1.},
    "grasp": {"workspace_mm": {"x": [-280, 280], "y": [-280, 280], "z": [70, 320]}}}


class Clock:
    def __init__(self): self.now = 0.
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class Backend:
    def __init__(self): self.calls=[]; self.angles=[0,0,0,-90,0,-45]; self.coords=list(POSE); self.joint_pose=list(POSE)
    def get_angles(self): return list(self.angles)
    def get_coords(self): return list(self.coords)
    def get_error_information(self): return 0
    def set_gripper_state(self, state, speed): self.calls.append(('gripper',state))
    def send_angles(self, angles, speed): self.calls.append(('joint',list(angles))); self.angles=list(angles); self.coords=list(self.joint_pose)
    def sync_send_angles(self, angles, speed, timeout=None): self.send_angles(angles,speed); return 1
    def send_coords(self, pose, speed, mode): self.calls.append(('tcp',list(pose),mode)); self.coords=list(pose); return 1


def make(cls):
    clock=Clock();backend=Backend()
    if cls is DirectRobotControllerV2:
        ctrl=cls(deepcopy(CFG),backend=backend,clock_fn=clock,sleep_fn=clock.sleep)
    else:
        ctrl=cls(deepcopy(CFG['robot']),backend=backend,clock_fn=clock,sleep_fn=clock.sleep,
                 workspace_mm=CFG['grasp']['workspace_mm'])
    return ctrl,backend


@pytest.mark.parametrize('cls',[DirectRobotControllerV2,OperatorRobotController])
@pytest.mark.parametrize('route',['preferred_ik','standard_ik','cartesian'])
def test_solver_chain_dispatches_once_and_reports_actual_route(cls, route):
    ctrl,backend=make(cls);solvers=[]
    def preferred(*a,**k):
        solvers.append('preferred_ik')
        if route!='preferred_ik':raise RobotError('preferred failed')
        return Q
    def standard(*a,**k):
        solvers.append('standard_ik')
        if route=='cartesian':raise RobotError('standard failed')
        return Q
    ctrl.select_preferred_ik=preferred;ctrl.preflight_place_ik=standard
    actual=ctrl.move_hybrid_transfer(POSE,label='transfer')
    assert actual==(None if cls is DirectRobotControllerV2 and route!='cartesian' else POSE)
    record=ctrl.transfer_evidence[-1]
    assert record['transfer_mode']==route and record['motion_completed']
    assert record['ik_validated']==(route=='preferred_ik') and record['path_validated'] is False
    assert record['fallback_mode']==(None if route=='preferred_ik' else route)
    assert len(backend.calls)==1 and backend.calls[0][0]==('tcp' if route=='cartesian' else 'joint')
    assert solvers==(['preferred_ik'] if route=='preferred_ik' else ['preferred_ik','standard_ik'])


@pytest.mark.parametrize('cls',[DirectRobotControllerV2,OperatorRobotController])
def test_paired_pre_place_angles_skip_solvers_and_verify_tcp(cls):
    ctrl,backend=make(cls)
    ctrl.select_preferred_ik=lambda *a,**k:pytest.fail('taught pose called preferred IK')
    ctrl.preflight_place_ik=lambda *a,**k:pytest.fail('taught pose called standard IK')
    ctrl.move_hybrid_transfer(POSE,label='taught',taught_angles=Q)
    assert backend.calls==[('joint',Q)]
    assert ctrl.transfer_evidence[-1]['transfer_mode']=='taught_joint'
    assert ctrl.transfer_evidence[-1]['ik_validated'] is False


@pytest.mark.parametrize('cls',[DirectRobotControllerV2,OperatorRobotController])
def test_motion_failure_is_not_caught_as_solver_failure_or_redispatched(cls):
    ctrl,backend=make(cls);ctrl.select_preferred_ik=lambda *a,**k:Q
    def fail(*a,**k):backend.calls.append(('failed_joint',));raise RobotError('motion failed')
    ctrl.move_joint_target=fail
    with pytest.raises(RobotError,match='motion failed'):ctrl.move_hybrid_transfer(POSE,label='failed')
    assert backend.calls==[('failed_joint',)] and ctrl.transfer_evidence[-1]['motion_completed'] is False


@pytest.mark.parametrize('cls',[DirectRobotControllerV2,OperatorRobotController])
def test_tcp_gate_is_retained_for_legacy_and_skipped_for_fixed_wait(cls):
    ctrl,backend=make(cls);backend.joint_pose[2]=100
    if cls is DirectRobotControllerV2:
        backend.get_coords=lambda:pytest.fail('fixed wait transfer added a TCP read')
        ctrl.move_hybrid_transfer(POSE,label='wrong',taught_angles=Q)
        assert ctrl.transfer_evidence[-1]['arrival_verified'] is False
    else:
        with pytest.raises(RobotError):ctrl.move_hybrid_transfer(POSE,label='wrong',taught_angles=Q)
    assert len(backend.calls)==1 and backend.calls[0][0]=='joint'


@pytest.mark.parametrize('cls',[DirectRobotControllerV2,OperatorRobotController])
def test_outside_workspace_rejected_before_solver_or_motion(cls):
    ctrl,backend=make(cls);ctrl.select_preferred_ik=lambda *a,**k:pytest.fail('outside endpoint reached IK')
    with pytest.raises(RobotError,match='outside_workspace'):ctrl.move_hybrid_transfer([100,80,1000,-175,-5,20],label='outside')
    assert backend.calls==[]


def test_start_without_tcp_uses_only_fresh_joint_arrival():
    ctrl,backend=make(DirectRobotControllerV2)
    backend.get_coords=lambda:None
    assert ctrl.initialize_for_pick()['start_verified']
    assert [c[0] for c in backend.calls]==['gripper','joint']


def test_optional_stale_start_tcp_does_not_override_valid_start_joints():
    ctrl,backend=make(DirectRobotControllerV2);ctrl.cfg['start_pose_mm_deg']=[0,0,1000,-180,0,0]
    assert ctrl.initialize_for_pick()['start_verified']
    assert backend.calls[-1]==('joint',CFG['robot']['start_angles_deg'])


@pytest.mark.parametrize('angles',[[1,2],[True,0,0,0,0,0],[float('nan')]*6,[1000]*6])
def test_bad_start_angles_have_zero_commands(angles):
    ctrl,backend=make(DirectRobotControllerV2);ctrl.cfg['start_angles_deg']=angles
    with pytest.raises(RobotError):ctrl.initialize_for_pick()
    assert backend.calls==[]


@pytest.mark.parametrize('cls',[DirectRobotControllerV2,OperatorRobotController])
@pytest.mark.parametrize('kind',['joint','cartesian'])
def test_feedback_accepts_six_finite_values_and_ignores_sdk_tail(cls,kind):
    ctrl,backend=make(cls)
    if kind=='joint':backend.get_angles=lambda:tuple(Q+[None,True,float('nan')])
    else:backend.get_coords=lambda:tuple(POSE+[None,True,float('nan')])
    value=ctrl._feedback(kind) if cls is DirectRobotControllerV2 else (ctrl._try_get_angles() if kind=='joint' else ctrl._try_get_coords())
    assert value==(Q if kind=='joint' else POSE)


def test_v2_full_program_has_two_joint_transfers_and_four_vertical_tcp_moves():
    ctrl,backend=make(DirectRobotControllerV2)
    def solve(target,**kwargs):backend.joint_pose=list(target);return Q
    ctrl.select_preferred_ik=solve
    grasp=SimpleNamespace(pregrasp_pose_mm_deg=[-100,0,240,-175,-5,0],grasp_pose_mm_deg=[-100,0,160,-175,-5,0],lift_pose_mm_deg=[-100,0,220,-175,-5,0])
    place=SimpleNamespace(planner_mode='hybrid',pre_place_pose_mm_deg=POSE,place_pose_mm_deg=[100,80,180,-175,-5,20],retreat_pose_mm_deg=[100,80,260,-175,-5,20])
    result=ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    assert [c[0] for c in backend.calls]==['joint','tcp','gripper','tcp','joint','tcp','gripper','tcp']
    assert all(c[2]==0 for c in backend.calls if c[0]=='tcp')
    assert result['ik_validated'] and not result['path_validated'] and not result['outcome_verified']


def ik_controller(*, raw=Q, samples=None, pre_error=32, post_error=32, legacy=False):
    ctrl, backend = make(DirectRobotControllerV2)
    if legacy:
        backend.__class__ = type('MyCobot', (Backend,), {'__module__': 'pymycobot.mycobot'})
    backend.ik_calls = []
    backend.read_times = []
    backend.reads_at_dispatch = []
    queue = list(samples or [])
    def angles():
        backend.read_times.append(ctrl._feedback_clock())
        return queue.pop(0) if queue else list(backend.angles)
    def solve(target, seed):
        backend.ik_calls.append((list(target), list(seed)))
        return raw
    def error():
        return post_error if backend.ik_calls else pre_error
    send = backend.send_angles
    def dispatch(*args):
        backend.reads_at_dispatch = list(backend.read_times)
        return send(*args)
    backend.get_angles = angles
    backend.solve_inv_kinematics = solve
    backend.get_error_information = error
    backend.send_angles = dispatch
    events = []
    ctrl.event_sink = lambda stage, event, **kw: events.append((event, kw['data']))
    return ctrl, backend, events


@pytest.mark.parametrize('pre_error', [0, 32])
@pytest.mark.parametrize('legacy', [False, True])
def test_valid_ik_survives_post_error32_with_complete_attempt_diagnostics(pre_error, legacy):
    raw = [round(q * 100) for q in Q] if legacy else Q
    ctrl, backend, events = ik_controller(raw=raw, pre_error=pre_error, legacy=legacy)
    ctrl.move_hybrid_transfer(POSE, label='latched')
    assert ctrl.transfer_evidence[-1]['transfer_mode'] == 'preferred_ik'
    assert backend.calls == [('joint', Q)] and len(backend.ik_calls) == 2
    selected = next(data for event, data in events if event == 'preferred_ik_selected')
    for index, attempt in enumerate(selected['attempts']):
        assert attempt['raw_solution'] == raw and attempt['parsed_angles'] == Q
        assert attempt['pre_error'] == (pre_error if index == 0 else 32) and attempt['post_error'] == 32
        assert attempt['controller_error_32_ignored'] is True
        assert attempt['seed_name'] and len(attempt['seed_angles']) == 6


@pytest.mark.parametrize('raw', [None, [], [0] * 6, [1000] * 6, [True] * 6,
                                [float('nan')] * 6, [80.] * 6])
def test_invalid_ik_still_falls_back_and_persists_all_failed_seed_details(raw, tmp_path):
    ctrl, backend, events = ik_controller(raw=raw)
    ctrl.move_hybrid_transfer(POSE, label='invalid')
    record = ctrl.transfer_evidence[-1]
    assert record['transfer_mode'] == 'cartesian' and backend.calls == [('tcp', POSE, 0)]
    assert len(backend.ik_calls) == 2  # Standard reuses this transfer's failed current seed.
    failed = next(data for event, data in events
                  if event == 'ik_solver_failed' and data['solver_mode'] == 'preferred_ik')
    assert len(failed['attempts']) == 2
    assert failed['error_details']['attempts'] == failed['attempts']
    assert record['solver_failures'][0]['attempts'] == failed['attempts']
    for attempt in failed['attempts']:
        assert {'seed_name', 'seed_angles', 'raw_solution', 'parsed_angles',
                'pre_error', 'post_error'} <= attempt.keys()
        assert attempt['parsed_angles'] is None
        assert attempt['pre_error'] == attempt['post_error'] == 32
    import json
    from operator_console.events import atomic_json
    atomic_json(tmp_path / 'failure.json', {'events': events, 'record': record})
    saved = json.loads((tmp_path / 'failure.json').read_text(encoding='utf-8'))
    assert len(saved['record']['solver_failures'][0]['error_details']['attempts']) == 2


@pytest.mark.parametrize('misses', [0, 1, 2])
def test_preferred_ik_retries_only_fresh_seed_reads_then_dispatches_once(misses):
    ctrl, backend, _ = ik_controller(samples=[None] * misses + [Q], pre_error=0, post_error=0)
    ctrl._last_verified_angles = [99.] * 6
    ctrl.move_hybrid_transfer(POSE, label='fresh')
    assert backend.reads_at_dispatch == pytest.approx([.08 * i for i in range(misses + 1)])
    assert ctrl.transfer_evidence[-1]['transfer_mode'] == 'preferred_ik'
    assert backend.calls == [('joint', Q)]
    assert ctrl._last_verified_angles == [99.] * 6


def test_three_missing_seed_reads_are_not_repeated_by_standard_or_replaced_by_cache():
    ctrl, backend, events = ik_controller(samples=[None] * 3)
    ctrl._last_verified_angles = Q
    ctrl.move_hybrid_transfer(POSE, label='missing')
    assert backend.read_times == pytest.approx([0., .08, .16])
    assert backend.ik_calls == [] and backend.calls == [('tcp', POSE, 0)]
    assert ctrl.transfer_evidence[-1]['transfer_mode'] == 'cartesian'
    assert len([event for event, _ in events if event == 'ik_solver_failed']) == 2
    assert ctrl._last_verified_angles is Q


def test_standard_ik_also_preserves_valid_numeric_solution_with_error32():
    ctrl, backend, _ = ik_controller()
    assert ctrl.preflight_place_ik(POSE, label='standalone') == Q
    assert len(backend.ik_calls) == 1 and backend.calls == []


def test_solver_exception_is_reported_in_each_seed_attempt():
    ctrl, backend, events = ik_controller()
    def fail(*args, **kwargs):
        raise RuntimeError('serial query failed')
    backend.solve_inv_kinematics = fail
    ctrl.move_hybrid_transfer(POSE, label='failure')
    failed = next(data for event, data in events
                  if event == 'ik_solver_failed' and data['solver_mode'] == 'preferred_ik')
    assert len(failed['attempts']) == 2
    assert all('serial query failed' in attempt['error'] for attempt in failed['attempts'])
    assert all(attempt['pre_error'] == 32 and attempt['post_error'] == 32 for attempt in failed['attempts'])
    assert backend.calls == [('tcp', POSE, 0)]


def test_cancellation_between_seed_reads_prevents_any_motion():
    ctrl, backend, _ = ik_controller(samples=[None, Q])
    def checkpoint():
        if ctrl._feedback_clock() >= .08:
            raise RobotError('operator cancelled')
    ctrl.checkpoint = checkpoint
    with pytest.raises(RobotError, match='operator cancelled'):
        ctrl.move_hybrid_transfer(POSE, label='cancelled')
    assert backend.calls == [] and backend.ik_calls == []
