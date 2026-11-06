"""Direct controller contract: deterministic feedback, no SDK/hardware connections."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from operator_console.direct_control_v2 import ArrivalSettings, DirectRobotControllerV2
from v0_mvp.robot import RobotError
from test_engineering_v2 import engineering


@pytest.mark.parametrize('speed', [1, 15, 20, 100])
def test_saved_start_speed_passes_both_home_baseline_and_v2(engineering, speed):
    from operator_console.doctor import validate_baseline
    current = engineering.read()
    engineering.save({'values': {'robot.start_angle_speed': speed}, 'expected_revision': current['revision']})
    cfg = engineering.service._base_config()
    validate_baseline(cfg)
    assert DirectRobotControllerV2(cfg)._speed('start_angle_speed') == speed


@pytest.mark.parametrize('speed', [0, 101, -1, True, False, 20., '20', None])
def test_home_start_speed_rejects_non_integer_or_out_of_range_values(speed):
    from pathlib import Path
    import yaml
    from operator_console.doctor import validate_baseline
    cfg = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/v0.yaml').read_text(encoding='utf-8'))
    cfg['robot']['start_angle_speed'] = speed
    with pytest.raises(ValueError, match='start_angle_speed must be an integer within 1..100'):
        validate_baseline(cfg)


def test_real_preview_initialization_capture_cleanup_uses_one_config_identity(tmp_path):
    from test_v2_console_integration import Harness
    from operator_console.runtime_v2 import _identity
    h = Harness(tmp_path)
    try:
        h.service.hardware_enabled = h.service.motion_enabled = True
        h.runtime._devices = {'camera': {'detected': True, 'count': 1}, 'robot': {'detected': True}}
        h.runtime._discover_devices = lambda: h.runtime._devices
        h.runtime._preflight = lambda *args, **kwargs: None
        h.runtime._accept_observation_consent = lambda payload: None
        h.runtime._empty_hand_confirmed = lambda: True
        h.runtime._holding = lambda: False
        cleanup = h.runtime._cleanup
        evidence = []
        def observe_cleanup(run):
            cleanup(run)
            if run.get('open_empty_evidence'):
                evidence.append(deepcopy(run['open_empty_evidence']))
        h.runtime._cleanup = observe_cleanup
        state = h.runtime.preview({'mode': 'real', 'move_to_start': True})
        h.run_id = state['run_id']
        state = h.wait('completed', 'failed')
        assert state['status'] == 'completed', state.get('error')
        run = h.runtime._runs[h.run_id]
        assert run['config_revision'] == _identity(run['config'])
        assert evidence and evidence[0]['config_revision'] == run['config_revision']
        assert run['controller'] is None
        assert len(h.backends) == 1 and h.backends[0].calls[0] == ('gripper', 0)
        assert ('capture', 1) in h.order
    finally:
        h.close()


def test_cleanup_without_config_revision_does_not_raise_or_invent_identity():
    from operator_console.runtime_v2 import ConsoleRuntimeV2
    runtime = object.__new__(ConsoleRuntimeV2)
    runtime._holding = lambda: False
    runtime._arm_state_marker = lambda: 'controlled'
    closed = []
    ctrl = SimpleNamespace(at_start=True, _gripper_open_confirmed=True,
                           gripper_holding_uncertain=False, close=lambda: closed.append(True))
    run = {'mode': 'real', 'controller': ctrl, 'start_verified': True,
           'status': 'running', 'cancelled': False}
    runtime._cleanup(run)
    assert closed == [True] and run['controller'] is None
    assert run['open_empty_evidence']['config_revision'] is None


CONFIG = {"robot": {"approach_strategy": "linear", "coordinate_speed": 15, "start_angle_speed": 15,
    "start_angles_deg": [0,0,0,-90,0,-45], "start_pose_mm_deg": [0,0,250,-180,0,0], "fallback_wait_s": 0,
    "gripper": {"speed": 80}, "pose_verify_stable_polls": 3, "motion_timeout_s": 10.0,
    "motion_poll_interval_s": 1.5}, "grasp": {"workspace_mm": {"x": [-280,280],"y":[-280,280],"z":[70,320]}}}


class Clock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class Backend:
    def __init__(self, clock, *, feedback=None, latency=0, dispatch_latency=0, fail_gripper_open=False):
        self.clock=clock;self.calls=[];self.coords=[0,0,250,-180,0,0];self.angles=[0,0,0,-90,0,-45]
        self.queue=list(feedback or []);self.latency=latency;self.dispatch_latency=dispatch_latency;self.reads=0
        self.fail_gripper_open=fail_gripper_open
    def solve_inv_kinematics(self,*a,**k): return None  # Exercise the Cartesian fallback branch.
    def send_coords(self,target,speed,mode):self.calls.append(('coords',list(target)));self.coords=list(target);self.clock.sleep(self.dispatch_latency)
    def send_angles(self,target,speed):self.calls.append(('angles',list(target)));self.angles=list(target);self.clock.sleep(self.dispatch_latency)
    def _feedback(self, fallback):
        self.reads+=1;self.clock.sleep(self.latency)
        if self.queue:
            result=self.queue.pop(0)
            if isinstance(result,Exception):raise result
            return result
        return list(fallback)
    def get_coords(self):return self._feedback(self.coords)
    def get_angles(self):return self._feedback(self.angles)
    def get_error_information(self):return 0
    def set_gripper_state(self,state,speed):
        self.calls.append(('gripper',state))
        if state==0 and self.fail_gripper_open:raise RuntimeError('open failed')


def plans(mode='cartesian'):
    grasp=SimpleNamespace(pregrasp_pose_mm_deg=[200,0,200,-180,0,0],grasp_pose_mm_deg=[200,0,160,-180,0,0],lift_pose_mm_deg=[200,0,220,-180,0,0])
    place=SimpleNamespace(planner_mode=mode,pre_place_pose_mm_deg=[150,-80,180,-180,0,0],place_pose_mm_deg=[150,-80,120,-180,0,0],retreat_pose_mm_deg=[150,-80,200,-180,0,0],place_joint_angles_deg=[10,20,30,-90,0,-45],retreat_joint_angles_deg=[0,10,20,-90,0,-45])
    return grasp,place


def controller(*,config=None,feedback=None,latency=0,dispatch_latency=0,checkpoint=None):
    clock=Clock();backend=Backend(clock,feedback=feedback,latency=latency,dispatch_latency=dispatch_latency);events=[]
    ctrl=DirectRobotControllerV2(config or CONFIG,backend=backend,clock_fn=clock,sleep_fn=clock.sleep,
        checkpoint=checkpoint,event_sink=lambda stage,event,**k:events.append((event,k['data'])))
    return ctrl,backend,clock,events


@pytest.mark.parametrize('kind',['cartesian','joint'])
@pytest.mark.parametrize('controller_error',[None,0])
@pytest.mark.parametrize('second_result',[1,None,0,-1,'exception'])
def test_serial_timeout_retries_only_after_known_clear_controller(kind,controller_error,second_result):
    ctrl,b,clock,events=controller();attempts=[]
    b._serial_port=SimpleNamespace(in_waiting=0)
    b.get_error_information=lambda:controller_error
    target=[100,0,180,-180,0,0] if kind=='cartesian' else [10,20,30,-90,0,-45]
    def send(*args):
        attempts.append(clock())
        if len(attempts)==1:return -1
        if second_result=='exception':raise OSError('retry transport failure')
        if kind=='cartesian':b.coords=list(target)
        else:b.angles=list(target)
        return second_result
    setattr(b,'send_coords' if kind=='cartesian' else 'send_angles',send)
    move=lambda:ctrl.move_pose(target) if kind=='cartesian' else ctrl.move_joint_target(target,label='joint')
    if controller_error==0 and second_result in (1,None,0):
        move()
        assert next(d for e,d in events if e=='movement_completed')['arrival_verified']
    else:
        with pytest.raises(RobotError,match='dispatch_failed'):move()
        assert not any(e=='movement_completed' for e,d in events)
    assert len(attempts)==(2 if controller_error==0 else 1)
    if len(attempts)==2:assert attempts[1]-attempts[0]>=3.0

@pytest.mark.parametrize('result',[False,-2.5,'exception'])
def test_serial_present_does_not_retry_other_failures(result):
    ctrl,b,clock,events=controller();attempts=[]
    b._serial_port=SimpleNamespace(in_waiting=0,reset_input_buffer=lambda:pytest.fail('empty input'))
    def send(*args):
        attempts.append(args)
        if result=='exception':raise OSError('controlled')
        return result
    b.send_coords=send
    with pytest.raises(RobotError,match='dispatch_failed'):ctrl.move_pose([100,0,180,-180,0,0])
    assert len(attempts)==1 and clock()>=1.5 and not any(e=='dispatch_retry' for e,d in events)

def test_cancel_during_serial_retry_delay_prevents_second_send():
    ctrl,b,clock,events=controller();attempts=[]
    b._serial_port=SimpleNamespace(in_waiting=0)
    b.send_coords=lambda *args:attempts.append(args) or -1
    ctrl.checkpoint=lambda:(_ for _ in ()).throw(RobotError('cancelled')) if clock()>=.2 else None
    with pytest.raises(RobotError,match='cancelled'):ctrl.move_pose([100,0,180,-180,0,0])
    assert len(attempts)==ctrl.command_count==1 and not any(e=='dispatch_retry' for e,d in events)

@pytest.mark.parametrize('kind',['cartesian','joint'])
@pytest.mark.parametrize('result',[None,0,1,True,2])
def test_sdk_send_acceptance_still_requires_arrival_feedback(kind,result):
    ctrl,b,clock,events=controller()
    target=[100,0,180,-180,0,0] if kind=='cartesian' else [10,20,30,-90,0,-45]
    def send(*args):
        b.calls.append(('sent',))
        if kind=='cartesian':b.coords=list(target)
        else:b.angles=list(target)
        return result
    setattr(b,'send_coords' if kind=='cartesian' else 'send_angles',send)
    ctrl.move_pose(target) if kind=='cartesian' else ctrl.move_joint_target(target,label='joint')
    complete=next(d for e,d in events if e=='movement_completed')
    assert len(b.calls)==1 and complete['send_result'] is result and complete['arrival_verified']
    assert b.reads==3 and clock()>=6

@pytest.mark.parametrize('kind',['cartesian','joint'])
@pytest.mark.parametrize('result',[False,-1,-2.5,'exception'])
def test_failed_sdk_send_has_one_attempt_no_wait_completion_or_downstream_commands(kind,result):
    ctrl,b,clock,events=controller();attempts=[];reads_at_send=[];times_at_send=[]
    def send(*args):
        attempts.append(args)
        reads_at_send.append(b.reads)
        times_at_send.append(clock())
        if result=='exception':raise OSError('controlled transport failure')
        return result
    setattr(b,'send_coords' if kind=='cartesian' else 'send_angles',send)
    with pytest.raises(RobotError,match='dispatch_failed'):
        if kind=='cartesian':ctrl.execute_pick_and_place(*plans(),geometry_passed=True,initialize=False)
        else:
            ctrl.move_joint_target([0,0,0,-90,0,-45],label='joint')
            ctrl.close_gripper()
    assert len(attempts)==ctrl.command_count==1 and b.calls==[] and b.reads==reads_at_send[0] and clock()>=times_at_send[0]+1.5
    assert [e for e,d in events].count('dispatch_failed')==1
    assert not any(e=='movement_completed' for e,d in events)
    failure=next(d for e,d in events if e=='dispatch_failed')
    assert failure['send_result']==(None if result=='exception' else result)

@pytest.mark.parametrize('metrics',[False,True])
@pytest.mark.parametrize('result',[None,0,1,False,-1,'exception'])
def test_adapter_shares_dispatch_contract_but_retains_fresh_joint_verification(metrics,result):
    from operator_console.robot_adapter import OperatorRobotController
    from test_stage2_hybrid_control import make, Q
    c,b=make(OperatorRobotController);c.cfg['collect_motion_metrics']=metrics
    events=[];c.event_sink=lambda stage,event,**k:events.append((event,k['data']))
    def send(target,*args,**kwargs):
        b.calls.append(('sent',list(target)))
        if result=='exception':raise OSError('controlled transport failure')
        if result is not False and not (isinstance(result,(int,float)) and result<0):b.angles=list(target)
        return result
    b.sync_send_angles=b.send_angles=send
    if result is False or result=='exception' or (isinstance(result,(int,float)) and result<0):
        with pytest.raises(RobotError,match='dispatch_failed'):c.move_joint_target(Q,label='adapter')
        assert not any(e=='joint_target_completed' for e,d in events)
        assert any(e=='dispatch_failed' for e,d in events)
    else:assert c.move_joint_target(Q,label='adapter')==Q
    assert len(b.calls)==1


def test_adapter_zero_does_not_bypass_joint_verification():
    from operator_console.robot_adapter import OperatorRobotController
    from test_stage2_hybrid_control import make, Q
    c,b=make(OperatorRobotController);b.sync_send_angles=lambda *args,**kwargs:0
    with pytest.raises(RobotError,match='joint target not reached'):c.move_joint_target(Q,label='unmoved')


def test_adapter_checkpoint_after_send_is_not_mislabeled_as_dispatch_failure():
    from operator_console.robot_adapter import OperatorRobotController
    from test_stage2_hybrid_control import make, Q
    c,b=make(OperatorRobotController);c.cfg['collect_motion_metrics']=True;events=[]
    c.event_sink=lambda stage,event,**kw:events.append(event)
    def checkpoint():
        if b.calls:raise RuntimeError('controlled cancellation after dispatch')
    c.checkpoint=checkpoint
    with pytest.raises(RuntimeError,match='controlled cancellation'):c.move_joint_target(Q,label='cancelled')
    assert len(b.calls)==1 and 'dispatch_failed' not in events and 'joint_target_completed' not in events


def test_before_capture_initialization_opens_then_verifies_start_without_ik():
    ctrl,backend,clock,events=controller()
    assert ctrl.initialize_for_pick()=={'control_completed':True,'start_verified':True,'arrival_verified':True}
    assert backend.calls==[('gripper',0),('angles',CONFIG['robot']['start_angles_deg'])]
    assert backend.reads==3 and ctrl.command_count==2
    assert events[-1][0]=='start_verified_before_capture'

@pytest.mark.parametrize('fault',['endpoint','workspace','holding','disabled','gripper'])
def test_before_capture_invalid_start_has_no_commands(fault):
    cfg=deepcopy(CONFIG)
    if fault=='endpoint':cfg['robot']['start_angles_deg']=None
    if fault=='workspace':cfg['robot']['start_angles_deg']=[1000]*6
    if fault=='disabled':cfg['robot']['move_to_start_before_pick']=False
    if fault=='gripper':cfg['robot']['gripper']['open_speed']=0
    ctrl,backend,_,_=controller(config=cfg)
    if fault=='holding':ctrl.gripper_holding_uncertain=True
    with pytest.raises(RobotError):ctrl.initialize_for_pick()
    assert backend.calls==[]


def test_before_capture_slow_send_keeps_wait_and_confirms_start():
    ctrl,b,clock,events=controller(dispatch_latency=1.1)
    assert ctrl.initialize_for_pick()['arrival_verified']
    assert len(b.calls)==2 and b.reads==3
    assert any(e=='start_verified_before_capture' and d['arrival_verified'] for e,d in events)

@pytest.mark.parametrize('kind',['cartesian','joint'])
def test_three_stable_samples_are_required_after_invalid_feedback(kind):
    target=[100,0,180,-180,0,0] if kind=='cartesian' else [0,0,0,-90,0,-45]
    ctrl,b,clock,events=controller(feedback=[None,target,target,target])
    actual=ctrl.move_pose(target) if kind=='cartesian' else ctrl.move_joint_target(target,label='joint')
    assert actual==target and len(b.calls)==1 and b.reads==4
    assert len([d for e,d in events if e=='feedback_sampled'])==3

@pytest.mark.parametrize('kind',['cartesian','joint'])
def test_slow_feedback_is_included_in_confirmation_deadline(kind):
    ctrl,b,clock,events=controller(latency=1.1)
    if kind=='cartesian':ctrl.move_pose([100,0,180,-180,0,0])
    else:ctrl.move_joint_target([0,0,0,-90,0,-45],label='joint')
    assert len(b.calls)==1 and b.reads==3
    assert clock()>=9 and any(e=='movement_completed' for e,_ in events)

def test_slow_dispatch_still_waits_after_send_and_confirms_arrival():
    ctrl,backend,clock,events=controller(dispatch_latency=1.1)
    ctrl.move_pose([100,0,180,-180,0,0])
    assert len(backend.calls)==1 and backend.reads==3 and clock.now==pytest.approx(7.1)

@pytest.mark.parametrize('mode',['cartesian','joint'])
def test_complete_pick_place_with_failed_ik_emits_release_for_both_modes(mode):
    ctrl,backend,clock,events=controller();grasp,place=plans(mode)
    result=ctrl.execute_pick_and_place(grasp,place,geometry_passed=True)
    assert result['control_completed'] and not result['outcome_verified']
    assert not result['ik_validated'] and not result['path_validated'] and not result['collision_validated_3d']
    assert [c[1] for c in backend.calls if c[0]=='gripper']==[0,1,0]
    assert len([e for e,_ in events if e=='object_released_at_place'])==1
    assert not ctrl.gripper_holding_uncertain
    assert backend.calls[0]==('gripper',0) and backend.calls[1][0]=='angles'


@pytest.mark.parametrize('fault',['geometry','workspace','nan','missing_start_endpoint','missing_joint_endpoint','hybrid','speed','mode','segment'])
def test_invalid_whole_task_has_zero_commands_including_initialization(fault):
    cfg=deepcopy(CONFIG);grasp,place=plans();passed=True
    if fault=='geometry':passed=False
    if fault=='workspace':place.retreat_pose_mm_deg[2]=500
    if fault=='nan':grasp.grasp_pose_mm_deg[0]=float('nan')
    if fault=='missing_start_endpoint':cfg['robot']['start_angles_deg']=None
    if fault=='missing_joint_endpoint':place.planner_mode='joint';place.place_pose_mm_deg=None
    if fault=='hybrid':place.planner_mode='unknown'
    if fault=='speed':cfg['robot']['coordinate_speed']=True
    if fault=='mode':place.place_mode=9
    if fault=='segment':cfg['robot'].update(approach_strategy='segmented',coarse_step_mm=0)
    ctrl,backend,_,_=controller(config=cfg)
    with pytest.raises(RobotError):ctrl.execute_pick_and_place(grasp,place,geometry_passed=passed)
    assert backend.calls==[] and ctrl.command_count==0


def test_wrong_arrival_feedback_blocks_close_and_release():
    ctrl,b,_,_=controller();b.get_coords=lambda:[0,0,70,0,0,0]
    with pytest.raises(RobotError,match='arrival_timeout'):
        ctrl.execute_pick_and_place(*plans(),geometry_passed=True,initialize=False)
    assert not any(c[0]=='gripper' for c in b.calls)

def test_failure_after_grip_retains_uncertain_holding_and_does_not_auto_open():
    ctrl,backend,_,_=controller();grasp,place=plans()
    send=backend.send_coords
    def failed(target,speed,mode):
        if target==grasp.lift_pose_mm_deg:raise RuntimeError('transport failed after gripping')
        send(target,speed,mode)
    backend.send_coords=failed
    with pytest.raises(RobotError):ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    assert ctrl.gripper_holding_uncertain
    assert [c for c in backend.calls if c[0]=='gripper']==[('gripper',1)]
    with pytest.raises(RobotError,match='gripper_holding_uncertain'):ctrl.execute_pick_and_place(grasp,place,geometry_passed=True)


def test_cancel_during_feedback_stops_remaining_segments():
    holder={}
    def checkpoint():
        if holder.get('backend') and holder['backend'].calls:raise RuntimeError('cancelled')
    ctrl,backend,_,_=controller(checkpoint=checkpoint);grasp,place=plans()
    holder['backend']=backend
    with pytest.raises(RuntimeError,match='cancelled'):ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    assert len(backend.calls)==1


def test_open_failure_does_not_emit_release_or_reset_holding():
    ctrl,backend,_,events=controller();backend.fail_gripper_open=True;grasp,place=plans('joint')
    with pytest.raises(RobotError):ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    assert ctrl.gripper_holding_uncertain and not any(e=='object_released_at_place' for e,_ in events)


def test_segmented_waypoints_keep_order_and_send_each_target_once():
    cfg=deepcopy(CONFIG);cfg['robot']['approach_strategy']='segmented'
    ctrl,backend,_,events=controller(config=cfg);grasp,place=plans()
    ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    started=[d['label'] for e,d in events if e=='movement_started']
    completed=[d['label'] for e,d in events if e=='movement_completed']
    assert len(started)>6 and started==completed and len(set(started))==len(started)
    assert not any(c[0]=='angles' for c in backend.calls)


@pytest.mark.parametrize('key',list(ArrivalSettings.__dataclass_fields__))
@pytest.mark.parametrize('value',[0,-1,True,float('nan'),float('inf')])
def test_arrival_settings_reject_invalid_values_without_clamping(key,value):
    with pytest.raises(ValueError):ArrivalSettings.from_config({}, {key:value})


def test_settings_are_frozen_and_joint_tolerance_is_explicit():
    cfg=deepcopy(CONFIG);ctrl,_,_,_=controller(config=cfg);cfg['robot']['pose_verify_stable_polls']=99
    assert ctrl.arrival.pose_verify_stable_polls==3
    assert ctrl.arrival.to_dict()['place_joint_verify_tolerance_deg']==5
    with pytest.raises(ValueError):ArrivalSettings.from_config({}, {'unknown':1})
    with pytest.raises(ValueError):ArrivalSettings(pose_verify_stable_polls=2.5)


def test_completion_returns_after_release_and_retreat_without_extra_gripper_wait():
    ctrl,backend,_,events=controller();grasp,place=plans()
    result=ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,return_to_start=True)
    assert result['at_start'] and result['holding'] is False and ctrl.at_start
    assert backend.calls[-1]==('angles',CONFIG['robot']['start_angles_deg'])
    assert [c[1] for c in backend.calls if c[0]=='gripper']==[0,1,0]
    labels=[d['label'] for e,d in events if e=='movement_completed']
    assert labels[-2:]==['place_retreat','completion_return_to_start']
    assert len([d for e,d in events if e=='feedback_sampled' and d['label']=='completion_return_to_start'])==3

def test_intermediate_task_defers_return_until_sequence_final_step():
    ctrl,backend,_,_=controller();grasp,place=plans()
    result=ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False,return_to_start=False)
    assert result['holding'] is False and result['at_start'] is False
    assert not any(c[0]=='angles' for c in backend.calls)


def test_completion_start_failure_never_claims_completed_or_at_start():
    ctrl,backend,_,events=controller();grasp,place=plans()
    backend.send_angles=lambda *args:(_ for _ in ()).throw(RuntimeError('controlled failure'))
    with pytest.raises(RobotError,match='completion_return_to_start_dispatch_failed'):
        ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False,return_to_start=True)
    assert ctrl.gripper_holding_uncertain is False and ctrl.at_start is False
    assert not any(e=='empty_hand_returned_to_start' for e,_ in events)


def test_intermediate_waypoints_use_zero_reads_and_endpoints_use_stable_polls():
    cfg=deepcopy(CONFIG);cfg['robot']['approach_strategy']='segmented'
    ctrl,backend,_,events=controller(config=cfg);grasp,place=plans()
    ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    for start in [d for e,d in events if e=='movement_started' and d['vertical_stage']]:
        samples=[d for e,d in events if e=='feedback_sampled' and d['label']==start['label']]
        assert len(samples)==(0 if start['intermediate'] else 3)
    assert all(d['arrival_verified'] is (not d['intermediate']) for e,d in events if e=='movement_completed')

def test_missing_feedback_stops_without_reusing_verified_cache():
    ctrl,b,clock,events=controller();cached=[0,0,200,-180,0,0];ctrl._last_verified_coords=cached
    b.get_coords=lambda:None
    with pytest.raises(RobotError,match='feedback_unavailable'):ctrl.move_pose([100,0,180,-180,0,0])
    assert ctrl._last_verified_coords is cached
    assert not any(e=='movement_completed' for e,d in events)

@pytest.mark.parametrize('strategy',['linear','segmented'])
def test_each_vertical_stage_has_only_one_endpoint_sample(strategy):
    cfg=deepcopy(CONFIG);cfg['robot'].update(approach_strategy=strategy,pose_verify_stable_polls=1)
    ctrl,backend,_,events=controller(config=cfg);grasp,place=plans()
    ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    for prefix in ['grasp_descent','grasp_lift','place_descent','place_retreat']:
        samples=[d for e,d in events if e=='feedback_sampled' and d['label'].startswith(prefix)]
        assert len(samples)==1


@pytest.mark.parametrize('policy,sdk_mode', [('segmented_mode0',0),('linear_mode1',1),('single_mode0',0)])
@pytest.mark.parametrize('step,counts', [(25.,[2,3,3,4]),(35.,[2,2,2,3]),(0.,[1,1,1,1])])
def test_vertical_modes_dispatch_all_four_stages_and_record_actual_sdk(policy,sdk_mode,step,counts):
    cfg=deepcopy(CONFIG);cfg['robot'].update(vertical_motion_mode=policy,vertical_segment_step_mm=step)
    c,b,clock,events=controller(config=cfg);sent=[];old=b.send_coords
    def send(p,s,m):sent.append((list(p),m));return old(p,s,m)
    b.send_coords=send
    grasp,place=plans();result=c.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    if policy!='segmented_mode0':counts=[1,1,1,1]
    records=[e for e in result['transfer_evidence'] if e.get('record_type')=='vertical_motion']
    assert len(records)==sum(counts) and all(e['sdk_mode']==sdk_mode and e['vertical_motion_mode']==policy and e['motion_completed'] for e in records)
    for stage,n in zip(('grasp_descent','grasp_lift','place_descent','place_retreat'),counts):
        assert sum(e['label'].startswith(stage) for e in records)==n
    for record in records:assert (record['target_pose_mm_deg'],sdk_mode) in sent
    assert sum(e=='pre_grasp_settled' and d['delay_s']==.35 for e,d in events)==1


@pytest.mark.parametrize('policy', ['segmented_mode0','linear_mode1','single_mode0'])
def test_each_mode_releases_on_bad_final_z_telemetry(policy):
    cfg=deepcopy(CONFIG);cfg['robot'].update(vertical_motion_mode=policy,vertical_segment_step_mm=0.)
    c,b,_,events=controller(config=cfg);g,p=plans();get=b.get_coords
    b.get_coords=lambda:[*g.grasp_pose_mm_deg[:2],g.grasp_pose_mm_deg[2]+9.25,*g.grasp_pose_mm_deg[3:]] if b.coords==g.grasp_pose_mm_deg else get()
    assert c.execute_pick_and_place(g,p,geometry_passed=True,initialize=False)['control_completed']
    assert ('gripper',1) in b.calls
    assert any(d.get('z_error_mm')==9.25 for e,d in events if e=='feedback_sampled')


@pytest.mark.parametrize('value', [True,-1,float('nan'),float('inf'),501])
def test_vertical_setting_rejects_invalid_step_before_dispatch(value):
    cfg=deepcopy(CONFIG);cfg['robot'].update(vertical_motion_mode='linear_mode1',vertical_segment_step_mm=value)
    c,b,_,_=controller(config=cfg)
    with pytest.raises(RobotError):c.execute_pick_and_place(*plans(),geometry_passed=True)
    assert b.calls==[]


@pytest.mark.parametrize('dx,dz,accepted',[(8.,0.,True),(8.001,0.,False),(0.,4.,True),(0.,4.001,False),(7.,4.,False)])
def test_grasp_endpoint_uses_configured_position_tolerance(dx,dz,accepted):
    c,b,clock,events=controller();target=[100,0,180,-180,0,0]
    b.get_coords=lambda:[100+dx,0,180+dz,-180,0,0]
    assert c.move_pose(target,grasp_endpoint=True)[0]==100+dx
    assert clock.now==pytest.approx(7.0)
    assert any(e=='movement_completed' for e,_ in events)

@pytest.mark.parametrize('policy',['segmented_mode0','linear_mode1','single_mode0'])
@pytest.mark.parametrize('sample',[None,RuntimeError('read failed'),[100,0,189.25,-180,0,0]])
def test_each_mode_blocks_grip_without_valid_arrival(policy,sample):
    cfg=deepcopy(CONFIG);cfg['robot'].update(vertical_motion_mode=policy,vertical_segment_step_mm=35.)
    c,b,_,events=controller(config=cfg)
    def read():
        if isinstance(sample,Exception):raise sample
        return sample
    b.get_coords=read
    with pytest.raises(RobotError,match='feedback_unavailable|arrival_timeout'):
        c.execute_pick_and_place(*plans(),geometry_passed=True,initialize=False)
    assert not any(x[0]=='gripper' for x in b.calls)

def test_intermediate_feedback_is_not_polled():
    c,b,clock,events=controller()
    b.get_coords=lambda:pytest.fail('intermediate coordinate query')
    b.get_angles=lambda:pytest.fail('intermediate joint query')
    c.move_pose([100,0,180,-180,0,0],vertical_stage='grasp_descent',waypoint_index=1,intermediate=True)
    assert clock()==pytest.approx(.35)

@pytest.mark.parametrize('kind',['cartesian','joint'])
def test_feedback_exception_cannot_claim_movement_completed(kind):
    c,b,clock,events=controller()
    c._feedback=lambda kind:(_ for _ in ()).throw(RobotError('controlled feedback error'))
    with pytest.raises(RobotError,match='controlled feedback error'):
        c.move_pose([100,0,180,-180,0,0]) if kind=='cartesian' else c.move_joint_target([0,0,0,-90,0,-45],label='joint')
    assert len(b.calls)==1 and not any(e=='movement_completed' for e,d in events)