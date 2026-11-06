"""Focused controller/sequence checks; synthetic data and injected clocks only."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
import json
import math
from pathlib import Path

import pytest
from test_direct_control_v2 import CONFIG, controller, plans
from v0_mvp.robot import RobotError


@pytest.mark.parametrize('state',[0,1])
@pytest.mark.parametrize('response',[-1,False,None,0,1])
def test_gripper_result_contract(state,response):
    ctrl,backend,_,events=controller()
    if state==0:ctrl.close_gripper()
    backend.set_gripper_state=lambda *a:response
    if response is False or response==-1:
        with pytest.raises(RobotError,match='gripper_dispatch_rejected'):
            ctrl.open_gripper() if state==0 else ctrl.close_gripper()
        assert ctrl.gripper_holding_uncertain
        assert not ctrl._gripper_open_confirmed
    else:
        ctrl.open_gripper() if state==0 else ctrl.close_gripper()
        assert ctrl._gripper_open_confirmed is (state==0)


@pytest.mark.parametrize('kind',['relative','self_relative','at_initial','on_top_of'])
def test_retry_masks_only_selected_object(kind):
    import cv2
    import numpy as np
    from operator_console.simulation import synthetic_scene
    from integration.relative_placement import plan_relative_place,project_robot_points_mm,_footprint
    root=Path(__file__).resolve().parents[1]
    fixture='relative_place_support_demo.json' if kind=='on_top_of' else 'relative_place_s3_demo.json'
    c=json.loads((root/'tests/data'/fixture).read_text(encoding='utf-8'))
    c['surface'].update(surface_tolerance_mm=25,use_local_surface_z=True)
    if kind!='on_top_of':c['objects'][1]['depth_m']=.46
    if kind=='self_relative':c['task'].update(destination_type=kind,reference_object=None)
    if kind=='at_initial':c['task']['spatial_relation']=kind
    ds,frame,d,p=synthetic_scene(c)
    options={}
    if kind=='self_relative':options['self_displacement_mm']=100
    if kind=='at_initial':
        poly,depth=_footprint(ds[0],frame,d,np.asarray(c['config']['calibration']['T_robot_from_camera']),c['config'],25)
        poly[:,1]+=100  # Original site is now vacant; the reference has moved away.
        options['initial_reference']={'class_name':'pen','footprint_robot_xy_mm':poly.tolist(),'xyz_mm':[0,100,0],'yaw_deg':0}
    def plan(ds,frame,d,p,**extra):
        return plan_relative_place(c['task'],ds,frame,deproject=d,project=p,config=c['config'],
            surface=c['surface'],gap_mm=None if kind=='self_relative' else 50,
            captured_at_unix_ms=1000,now_unix_ms=1000,**options,**extra)
    first=plan(ds,frame,d,p)
    x,y=first['destination_center_robot_xy_mm'];z=40
    c['objects'][1].update(center_uv=project_robot_points_mm([[x+25,y,z]],c['config'],frame.intrinsics)[0],depth_m=.46)
    ds,frame,d,p=synthetic_scene(c)
    retry=plan(ds,frame,d,p,locked_destination=first)
    assert retry['destination_center_robot_xy_mm']==first['destination_center_robot_xy_mm']
    assert retry['excluded_self_pixels']>0
    uv=project_robot_points_mm([[a,b,first['landing_surface_z_mm']] for a,b in first['clearance_envelope_robot_xy_mm']],c['config'],frame.intrinsics)
    mask=np.zeros(frame.depth_m.shape,dtype=np.uint8);cv2.fillConvexPoly(mask,np.rint(uv).astype(np.int32),1)
    own=np.zeros_like(mask);cv2.fillConvexPoly(own,np.asarray(ds[1].corners_px).astype(np.int32),1)
    rows,cols=np.nonzero(mask & (1-own));frame.depth_m[rows[len(rows)//2],cols[len(cols)//2]]=.3
    with pytest.raises(RuntimeError,match='destination_surface_not_clear'):
        plan(ds,frame,d,p,locked_destination=first)


@pytest.mark.parametrize('value',[None,[],-1,False,[0]*5,[0]*7,[False]*6,['0']*6,[math.nan]*6,[math.inf]*6,[99999]*6])
def test_invalid_feedback_is_not_arrival(value):
    ctrl,b,_,_=controller();b.get_coords=lambda:value
    assert ctrl._feedback('cartesian') is None


def test_feedback_queries_are_spaced_and_flush_only_idle_buffer():
    import threading
    ctrl,b,clock,_=controller();times=[];clears=[]
    b.lock=threading.Lock();b.thread_lock=True
    b._serial_port=SimpleNamespace(in_waiting=1,reset_input_buffer=lambda:clears.append(b.lock.locked()))
    b.get_coords=lambda:times.append(clock()) or [0,0,100,-175,-5,0]
    assert ctrl._feedback('cartesian')
    assert ctrl._feedback('cartesian')
    assert times[1]-times[0]>=1.0 and clears==[True,True]


@pytest.mark.parametrize('kind,value',[('joint',[0,136,0,0,0,0]),('cartesian',[351,0,100,0,0,0])])
def test_model_feedback_bounds(kind,value):
    ctrl,b,_,_=controller();setattr(b,'get_angles' if kind=='joint' else 'get_coords',lambda:value)
    assert ctrl._feedback(kind) is None


def bounded_controller(**kwargs):
    cfg=deepcopy(CONFIG)
    cfg['robot'].update(motion_poll_interval_s=1.5,motion_timeout_s=10,pose_verify_stable_polls=2)
    return controller(config=cfg,**kwargs)


@pytest.mark.parametrize('bad',[None,[],-1,False])
def test_arrival_recovers_bad_packets_without_resending_motion(bad):
    target=[100,0,150,-175,-5,0]
    ctrl,b,clock,events=bounded_controller(feedback=[bad,target,target]);reads=[]
    get=b.get_coords
    b.get_coords=lambda:reads.append(clock()) or get()
    assert ctrl.move_pose(target)==target
    assert len(b.calls)==1 and len(reads)==3
    assert all(y-x>=1.5 for x,y in zip(reads,reads[1:]))
    assert [d for e,d in events if e=='movement_completed'][-1]['arrival_verified']


def test_three_invalid_replies_stop_before_gripping():
    ctrl,b,_,_=bounded_controller()
    b.get_coords=lambda:None
    with pytest.raises(RobotError,match='feedback_unavailable'):
        ctrl.execute_pick_and_place(*plans(),geometry_passed=True,initialize=False)
    assert len(b.calls)==1 and not any(c[0]=='gripper' for c in b.calls)


def test_valid_but_wrong_position_times_out_without_resend():
    ctrl,b,_,_=bounded_controller();b.get_coords=lambda:[0,0,100,-175,-5,0]
    with pytest.raises(RobotError,match='arrival_timeout'):
        ctrl.move_pose([100,0,150,-175,-5,0])
    assert len(b.calls)==1


def test_cancel_during_retry_wait_prevents_next_query():
    ctrl,b,clock,_=bounded_controller(feedback=[None]);reads=[]
    b.get_coords=lambda:reads.append(clock()) or None
    ctrl.checkpoint=lambda:(_ for _ in ()).throw(RobotError('cancelled')) if reads and clock()>=reads[0]+1 else None
    with pytest.raises(RobotError,match='cancelled'):
        ctrl.move_pose([100,0,150,-175,-5,0])
    assert len(reads)==1


def test_intermediate_waypoint_does_not_poll():
    ctrl,b,_,events=bounded_controller()
    ctrl.move_pose([100,0,150,-175,-5,0],vertical_stage='grasp_descent',waypoint_index=1,intermediate=True)
    assert b.reads==0 and not [d for e,d in events if e=='movement_completed'][-1]['arrival_verified']


@pytest.mark.parametrize('code',[None,-1,False,1,6,16,32,33])
def test_unknown_or_fault_controller_blocks_both_retry_levels(code):
    ctrl,b,clock,_=bounded_controller();sends=[];diagnostics=[]
    b._serial_port=SimpleNamespace(in_waiting=0)
    b.send_coords=lambda *a:sends.append(clock()) or -1
    b.get_error_information=lambda:diagnostics.append(clock()) or code
    with pytest.raises(RobotError) as caught:
        ctrl.move_pose([100,0,150,-175,-5,0],label='pick_pregrasp')
    assert not caught.value.retry_empty_hand and len(sends)==1
    assert len(diagnostics)==(2 if code is None or code is False or code==-1 else 1)


def test_error_query_can_recover_and_resend_only_with_known_zero():
    ctrl,b,clock,_=bounded_controller();sends=[];codes=iter([None,0]);queries=[]
    b._serial_port=SimpleNamespace(in_waiting=0)
    def send(*args):
        sends.append(clock());b.coords=list(args[0]);return -1 if len(sends)==1 else 1
    b.send_coords=send;b.get_error_information=lambda:queries.append(clock()) or next(codes)
    ctrl.move_pose([100,0,150,-175,-5,0],label='pick_pregrasp')
    assert len(sends)==2 and sends[1]-queries[-1]>=1.5


def test_joint_transfer_reuses_arrival_without_extra_tcp_read():
    ctrl,b,_,_=bounded_controller();b.get_coords=lambda:pytest.fail('extra TCP read')
    ctrl.move_hybrid_transfer([100,0,150,-175,-5,0],label='transfer',taught_angles=[0,0,0,-90,0,-45])
    assert ctrl.transfer_evidence[-1]['arrival_verified']
    assert not ctrl.transfer_evidence[-1]['tcp_arrival_verified']
    assert b.reads==2


def test_start_and_pick_result_require_feedback():
    ctrl,b,_,events=bounded_controller()
    assert ctrl.initialize_for_pick()['arrival_verified'] and ctrl.at_start
    result=ctrl.execute_pick_and_place(*plans(),geometry_passed=True,initialize=False,return_to_start=True)
    assert result['arrival_verified'] and not result['outcome_verified'] and not result['path_validated']
    assert all(d['required_stable_polls']==2 for e,d in events if e=='feedback_sampled')


def test_start_failure_does_not_set_start_flag():
    ctrl,b,_,_=bounded_controller();b.get_angles=lambda:None
    with pytest.raises(RobotError,match='feedback_unavailable'):
        ctrl.initialize_for_pick()
    assert not ctrl.at_start


def sequence_harness(tasks=None):
    from test_task_sequence_v2 import Harness
    h=Harness(tasks=tasks);h.seq.step_recovery_enabled=True
    h.seq.initialize_before_observe=lambda *a,**k:{'control_completed':True,'start_verified':True,'empty_hand_confirmed':True}
    observe=h.observe
    def fresh(step):
        h.now+=1
        obs=observe(step)
        return replace(obs,instances=tuple(i|{'instance_id':obs.frame_id+':'+i['class_name']} for i in obs.instances))
    h.seq.observe=fresh
    prepare=h.prepare
    h.seq.prepare=lambda *a:prepare(*a)|{'result_expectation':{'frozen':True}}
    h.seq.check_result=lambda *a:{'passed':True}
    h.completion_options=[]
    def complete(**kwargs):
        h.completion_options.append(kwargs)
        return fresh(1)
    h.seq.observe_completion=complete
    h.begin()
    return h


@pytest.mark.parametrize('phase',['capture','prepare'])
def test_before_motion_recaptures_once_with_new_selection(phase):
    h=sequence_harness();calls=[]
    method=h.seq.observe if phase=='capture' else h.seq.prepare
    def transient(*a):
        calls.append(True)
        if len(calls)==1:raise RuntimeError('camera_capture_timeout' if phase=='capture' else 'unreliable_object_depth')
        return method(*a)
    if phase=='capture':h.seq.observe=transient
    else:h.seq.prepare=transient
    assert h.seq.advance()['status']=='running' and not h.commands
    s=h.seq.advance()
    assert len(h.commands)==1 and s['steps'][0]['perception_recaptures_before']==1
    assert h.commands[0]['prepared']['selected']['target'].startswith(h.captures and 'frame')
    assert s['steps'][0].get('retry_count',0)==0


def test_missing_target_recaptures_then_retains_manual_recovery():
    h=sequence_harness();h.objects=[i for i in h.objects if i['class_name']!='pen']
    assert h.seq.advance()['status']=='running'
    assert h.seq.advance()['status']=='waiting_step_recovery'
    assert len(h.captures)==2 and not h.commands


@pytest.mark.parametrize('reason',['gripper_area_occupied','destination_outside_workspace','destination_surface_not_clear'])
def test_confirmed_geometry_rejection_does_not_recapture(reason):
    h=sequence_harness()
    h.seq.prepare=lambda *a:(_ for _ in ()).throw(RuntimeError(reason))
    s=h.seq.advance()
    assert s['status']=='failed' and len(h.captures)==1 and not h.commands
    assert s['steps'][0].get('perception_recaptures_before',0)==0


@pytest.mark.parametrize('persistent',[False,True])
def test_result_recapture_does_not_repeat_motion(persistent):
    h=sequence_harness();checked=[]
    def check(*args):
        checked.append(True)
        if persistent or len(checked)==1:raise RuntimeError('result_target_depth_unavailable')
        return {'passed':True}
    h.seq.check_result=check
    h.seq.advance();s=h.seq.advance()
    assert s['status']==('failed' if persistent else 'completed')
    assert len(h.commands)==1 and len(checked)==2
    assert h.completion_options==[{}, {'force':True}]
    assert s['steps'][0].get('retry_count',0)==0


def test_valid_result_failure_uses_motion_budget_not_recapture():
    h=sequence_harness();h.seq.check_result=lambda *a:{'passed':False}
    h.seq.advance();s=h.seq.advance()
    assert s['steps'][0]['retry_count']==1
    assert s['steps'][0].get('perception_recaptures_result',0)==0
    assert len(h.completion_options)==1


def test_forced_completion_capture_bypasses_only_image_cache():
    from operator_console.runtime_v2 import ConsoleRuntimeV2
    from operator_console.task_sequence_v2 import ObservationV2
    runtime=object.__new__(ConsoleRuntimeV2);captures=[]
    runtime._save=lambda run:None;runtime._event=lambda *a,**k:None
    def observe(*a,**k):
        captures.append(True)
        return ObservationV2(str(len(captures)),1000+len(captures),(),display={'frame_id':str(len(captures))})
    runtime._observe=observe
    run={'cancelled':False,'config':{'robot':{}},'controller':SimpleNamespace(gripper_holding_uncertain=False,at_start=True,sleep=lambda *a:None)}
    state={'steps':[{'frame_id':'source'}],'step_id':1}
    a=runtime._capture_completion(run,state)
    assert runtime._capture_completion(run,state) is a
    b=runtime._capture_completion(run,state,force=True)
    assert b.frame_id!=a.frame_id and len(captures)==2


def test_late_valid_feedback_cannot_pass_deadline():
    ctrl,b,clock,_=bounded_controller()
    def slow():
        clock.sleep(11)
        return [100,0,150,-175,-5,0]
    b.get_coords=slow
    with pytest.raises(RobotError,match='arrival_timeout'):
        ctrl.move_pose([100,0,150,-175,-5,0])
    assert not ctrl._last_arrival_verified


def test_invalid_reply_resets_stable_streak():
    target=[100,0,150,-175,-5,0]
    ctrl,b,_,_=bounded_controller(feedback=[target,None,target,target])
    ctrl.move_pose(target)
    assert b.reads==4


def test_position_setting_changes_endpoint_gate():
    from operator_console.direct_control_v2 import ArrivalSettings
    ctrl,b,_,_=bounded_controller()
    ctrl.arrival=ArrivalSettings(pose_verify_position_tolerance_mm=5,motion_poll_interval_s=1.5,motion_timeout_s=5)
    b.get_coords=lambda:[110,0,150,-175,-5,0]
    with pytest.raises(RobotError,match='arrival_timeout'):
        ctrl.move_pose([100,0,150,-175,-5,0])


def test_poll_setting_store_rejects_fast_queries(tmp_path):
    from operator_console.arrival_settings_v2 import ArrivalSettingsStoreV2
    cfg=deepcopy(CONFIG);cfg['robot']['motion_poll_interval_s']=.5
    store=ArrivalSettingsStoreV2(tmp_path,config_loader=lambda:cfg)
    with pytest.raises(ValueError,match='at_least_1.0'):store.freeze()
    cfg['robot']['motion_poll_interval_s']=1.5
    assert store.freeze().settings.motion_poll_interval_s==1.5


def test_result_recapture_rejects_reused_frame():
    h=sequence_harness();h.seq.check_result=lambda *a:(_ for _ in ()).throw(RuntimeError('result_target_not_observed'))
    get=h.seq.observe_completion;cached=[]
    def stale(**kwargs):
        if not cached:cached.append(get())
        return cached[0]
    h.seq.observe_completion=stale
    h.seq.advance();s=h.seq.advance()
    assert s['status']=='failed' and 'not_new' in s['error']['reason'] and len(h.commands)==1


def test_nonfinal_result_recapture_does_not_repeat_start_or_motion():
    from test_task_sequence_v2 import task
    h=sequence_harness([task(),task()]);starts=[];check=[]
    init=h.seq.initialize_before_observe
    h.seq.initialize_before_observe=lambda *a,**k:starts.append(a[0]) or init(*a,**k)
    def transient(*a):
        check.append(True)
        if len(check)==1:raise RuntimeError('result_target_depth_unavailable')
        return {'passed':True}
    h.seq.check_result=transient
    h.seq.advance();s=h.seq.advance()
    assert s['status']=='running' and s['steps'][0]['status']=='completed'
    assert len(h.commands)==1 and starts==[1,2] and h.captures==[1,2,2]
