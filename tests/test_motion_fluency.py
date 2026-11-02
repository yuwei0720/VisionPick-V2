"""Fluency changes with synthetic clocks/devices and temporary persistence only."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from v0_mvp.robot import build_vertical_waypoints, RobotError
from operator_console.direct_control_v2 import DirectRobotControllerV2, ArrivalSettings
from test_direct_control_v2 import CONFIG, Clock, controller, plans


def test_today_yaml_fixed_waits_and_start_settle_are_preserved():
    from pathlib import Path
    import yaml
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/v0.yaml').read_text(encoding='utf-8'))
    waits={key:cfg['robot'][key] for key in
           ('vertical_endpoint_wait_s','transfer_wait_s','intermediate_wait_s','start_settle_s')}
    assert waits==dict(vertical_endpoint_wait_s=4.,transfer_wait_s=3.,intermediate_wait_s=.35,start_settle_s=1.5)
    # Isolate Start timing from the independently configured gripper delay.
    cfg['robot']['gripper'].update(open_wait_s=0.,wait_s=0.)
    c,b,clock,events=controller(config=cfg)
    c.initialize_for_pick()
    assert clock()==pytest.approx(4.5)
    completed=next(data for event,data in events if event=='movement_completed')
    assert completed['wait_s']==3. and completed['arrival_verified'] is False


@pytest.mark.parametrize('distance,count', [(0,0),(5,1),(25,1),(60,3),(80,4),(62.5,3),(100,4)])
@pytest.mark.parametrize('direction', [-1,1])
def test_uniform_vertical_partition_exact_endpoint_and_no_remainder(distance,count,direction):
    start=[100,0,180,-180,0,0];end=list(start);end[2]+=distance*direction
    points=build_vertical_waypoints({'coarse_step_mm':25.},start,end,fine_near_end=True)
    assert len(points)==count
    if not points:return
    assert points[-1]==end
    zs=[start[2]]+[p[2] for p in points]
    assert [abs(b-a) for a,b in zip(zs,zs[1:])]==pytest.approx([distance/count]*count)
    assert all(p[:2]+p[3:]==end[:2]+end[3:] for p in points)


def move_vertical(ctrl,start,end,label):
    for _,pose,name,options in ctrl._vertical_commands(start,end,label,0,True):
        ctrl.move_pose(pose,label=name,mode=options[0],stable_count=options[1],grasp_endpoint=options[2],
            vertical_stage=options[3],waypoint_index=options[4],intermediate=options[5])


def test_vertical_intermediates_have_no_feedback_or_error_reads():
    cfg=deepcopy(CONFIG);cfg['robot'].update(approach_strategy='segmented',coarse_step_mm=25.)
    c,b,clock,_=controller(config=cfg);sends=[];reads=[];errors=[]
    send=b.send_coords;get=b.get_coords
    b.send_coords=lambda *a:(sends.append(clock()),send(*a))[-1]
    b.get_coords=lambda:(reads.append(clock()),get())[-1]
    b.get_error_information=lambda:(errors.append(clock()),0)[-1]
    move_vertical(c,[100,0,240,-180,0,0],[100,0,160,-180,0,0],'grasp_descent')
    assert sends==pytest.approx([0,.35,.7,1.05])
    assert reads==pytest.approx([5.05]) and errors==[]
    c.move_pose([100,0,160,-180,0,0],label='different_stage')
    assert sends[-1]==pytest.approx(5.3) and reads[-1]==pytest.approx(8.3)


@pytest.mark.parametrize('z,accepted',[(4.,True),(4.001,False),(4.8,False),(5.,False),(26.6666667,False)])
def test_grasp_z_error_is_sampled_once_without_arrival_gate(z,accepted):
    c,b,clock,events=controller();target=[100,0,180,-180,0,0]
    b.get_coords=lambda:[100,0,180+z,-180,0,0]
    assert c.move_pose(target,grasp_endpoint=True)[2]==180+z
    assert len([d for e,d in events if e=='feedback_sampled'])==1


def test_label_alone_does_not_enable_vertical_cooldown_bypass():
    c,b,clock,_=controller();target=[100,0,180,-180,0,0]
    c.move_pose(target,label='grasp_descent[1]',stable_count=1)
    c.move_pose(target,label='grasp_descent[2]',stable_count=1)
    assert clock()==pytest.approx(6.25)


def test_failed_current_ik_is_queried_once_per_transfer_and_throttled():
    from test_stage2_hybrid_control import make, POSE
    c,b=make(DirectRobotControllerV2);queries=[]
    def record(name,result):
        def call(*a,**k):queries.append((name,c._clock()));return result
        return call
    b.get_angles=record('angles',list(b.angles))
    b.get_error_information=record('error',0)
    b.solve_inv_kinematics=record('ik',None)
    c.move_hybrid_transfer(POSE,label='one')
    before_dispatch=queries[:-1]  # Last error belongs to movement completion.
    assert [name for name,_ in queries].count('angles')==1
    assert [name for name,_ in queries].count('ik')==2
    assert all(t1-t0>=.04-1e-9 for (_,t0),(_,t1) in zip(before_dispatch,before_dispatch[1:]))
    assert c._ik_attempt_context is None
    c.move_hybrid_transfer(POSE,label='two')
    assert [name for name,_ in queries].count('angles')==2
    assert [name for name,_ in queries].count('ik')==4


def test_standard_ik_is_still_available_without_a_completed_current_attempt():
    from test_stage2_hybrid_control import make, POSE, Q
    c,b=make(DirectRobotControllerV2);calls=[]
    c.select_preferred_ik=lambda *a,**k:(_ for _ in ()).throw(RobotError('unavailable before seed'))
    b.solve_inv_kinematics=lambda *a,**k:(calls.append(a),Q)[-1]
    c.move_hybrid_transfer(POSE,label='standard')
    assert len(calls)==1 and c.transfer_evidence[-1]['transfer_mode']=='standard_ik'


def test_high_frequency_events_stay_in_memory_but_boundaries_and_errors_save():
    import threading
    from operator_console.runtime_v2 import ConsoleRuntimeV2
    rt=object.__new__(ConsoleRuntimeV2);rt._lock=threading.RLock();saved=[]
    rt._save=lambda run:saved.append(len(run['events']))
    run={'id':'r','mode':'offline','events':[]}
    for _ in range(100):rt._event(run,'motion','feedback_polled',data={'intermediate':False})
    for event in ('movement_started','movement_completed'):
        rt._event(run,'motion',event,data={'intermediate':True})
    assert len(run['events'])==102 and saved==[]
    rt._event(run,'motion','movement_completed',data={'intermediate':False})
    assert saved==[]
    rt._event(run,'motion','feedback_polled',status='error',data={'intermediate':True})
    rt._event(run,'motion','dispatch_failed',data={})
    rt._event(run,'flow','step_completed',data={})
    rt._event(run,'motion','movement_completed',data={'controller_error':33})
    assert saved==[104,105,106,107]


@pytest.mark.parametrize('outcome',['completed','failed','waiting_instance'])
def test_step_boundary_is_persisted_before_execute_and_outside_sequence_lock(outcome):
    from test_task_sequence_v2 import Harness,instance
    objects=[instance('p1','pen'),instance('p2','pen')] if outcome=='waiting_instance' else None
    h=Harness(objects=objects);durable=[]
    def persist():
        assert not h.seq._lock._is_owned()
        durable.append(h.seq.snapshot())
    h.seq.persist_boundary=persist
    old=h.seq.execute
    def execute(*a,**kw):
        assert durable[-1]['steps'][0]['status']=='executing'
        if outcome=='failed':raise RobotError('controlled failure')
        return old(*a,**kw)
    h.seq.execute=execute;h.begin();h.seq.advance()
    assert durable[-1]['status']==outcome
    if outcome=='waiting_instance':assert h.commands==[]


def test_holding_files_change_only_at_close_dispatch_and_release(tmp_path):
    import json,threading
    from operator_console.runtime_v2 import ConsoleRuntimeV2
    rt=object.__new__(ConsoleRuntimeV2);rt._lock=threading.RLock();rt._arm_motion_epoch=0
    rt.holding_path=tmp_path/'controller.json';saved=[]
    ctrl=SimpleNamespace(gripper_holding_uncertain=False,at_start=False)
    run={'id':'r','mode':'real','controller':ctrl,'events':[]};rt._runs={'r':run}
    rt._save=lambda run:saved.append(json.loads(rt.holding_path.read_text())['holding_uncertain'])
    def execute(*a,**kw):
        assert saved==[] and not rt.holding_path.exists()
        ctrl.gripper_holding_uncertain=True
        rt._event(run,'gripper','gripper_commanded',data={'gripper_state':1})
        assert saved==[True]
        ctrl.gripper_holding_uncertain=False
        rt._event(run,'place_release','object_released_at_place',data={})
        assert saved==[True,False]
        return {'control_completed':True}
    ctrl.execute_pick_and_place=execute
    rt._execute(run,{'grasp':None,'place':None},initialize=False,checkpoint=lambda:None)
    assert saved==[True,False,False]


@pytest.mark.parametrize('waiting,resets',[(0,0),(1,1),(5,1)])
def test_fresh_getter_and_gripper_clear_only_waiting_serial_bytes(waiting,resets):
    from v0_mvp.robot import MyCobotController
    port=SimpleNamespace(in_waiting=waiting,resets=0)
    def reset():port.in_waiting=0;port.resets+=1
    port.reset_input_buffer=reset
    backend=SimpleNamespace(_serial_port=port,get_coords=lambda:[100,0,179 if port.in_waiting else 160,-180,0,0],
        set_gripper_state=lambda *a:None)
    c=MyCobotController({'gripper':{'wait_s':0}},backend=backend,sleep_fn=lambda _:None)
    assert c._try_get_coords()[2]==160 and port.resets==resets
    port.in_waiting=waiting;c.open_gripper()
    assert port.resets==resets*2 and c._gripper_open_confirmed
    c.close_gripper();assert not c._gripper_open_confirmed


def test_explicit_getter_retry_clears_pending_reply_without_cache_fallback():
    from v0_mvp.robot import MyCobotController
    port=SimpleNamespace(in_waiting=1,resets=0);calls=[]
    def reset():port.in_waiting=0;port.resets+=1
    port.reset_input_buffer=reset
    def read():calls.append(1);port.in_waiting=1;return None
    c=MyCobotController({},backend=SimpleNamespace(_serial_port=port,get_coords=read),sleep_fn=lambda _:None)
    c._last_verified_coords=[100,0,160,-180,0,0]
    assert c._try_get_coords() is None and len(calls)==1 and port.resets==1
    assert c._try_get_coords(attempts=2) is None and len(calls)==3 and port.resets==3


@pytest.mark.parametrize('interval,accepted',[(.001,False),(.249999,False),(.25,True),(.5,True)])
def test_ui_poll_floor_rejects_without_overwriting_and_frozen_values_stay(tmp_path,interval,accepted):
    from operator_console.arrival_settings_v2 import ArrivalSettingsStoreV2
    cfg=deepcopy(CONFIG);cfg['robot']['motion_poll_interval_s']=.5
    store=ArrivalSettingsStoreV2(tmp_path,config_loader=lambda:cfg);frozen=store.freeze()
    if accepted:
        store.save({'motion_poll_interval_s':interval},expected_revision=frozen.revision)
        assert store.freeze().settings.motion_poll_interval_s==interval
    else:
        with pytest.raises(ValueError,match='at_least_0.25'):
            store.save({'motion_poll_interval_s':interval},expected_revision=frozen.revision)
        assert not store.path.exists()
    assert frozen.settings.motion_poll_interval_s==.5


@pytest.mark.parametrize('key',['coordinate_speed','place_joint_speed'])
def test_motion_fingerprint_tracks_each_transfer_speed(key,tmp_path):
    from operator_console.assets import AssetRegistry
    reg=AssetRegistry(tmp_path);cfg=deepcopy(CONFIG)
    def fingerprint(c):return next(x['asset_id'] for x in reg._configuration_assets(c) if x['kind']=='motion_safety_profile')
    before=fingerprint(cfg);cfg['robot'][key]=20
    assert fingerprint(cfg)!=before


def test_initialization_reuses_only_completed_open_and_empty_evidence():
    c,b,clock,_=controller();c.initialize_for_pick();before=len(b.calls)
    c.initialize_for_pick()
    assert [x[0] for x in b.calls[before:]]==['angles']
    c.close_gripper();before=len(b.calls)
    assert c.gripper_holding_uncertain
    c.gripper_holding_uncertain=False  # Synthetic operator empty-hand confirmation.
    c.initialize_for_pick()
    assert [x[0] for x in b.calls[before:]]==['gripper','angles']
    c.gripper_holding_uncertain=True
    with pytest.raises(RobotError,match='holding_uncertain'):c.initialize_for_pick()


@pytest.mark.parametrize('invalidate',[None,'marker','config','holding'])
def test_auto_observe_open_evidence_is_run_local_and_invalidated(tmp_path,invalidate):
    import json,threading
    from operator_console.runtime_v2 import ConsoleRuntimeV2
    rt=object.__new__(ConsoleRuntimeV2);rt._lock=threading.RLock();rt._arm_motion_epoch=0;rt._runs={}
    rt.service=SimpleNamespace(runs_root=tmp_path/'runs');rt.service.runs_root.mkdir()
    rt.engineering=SimpleNamespace(last_pose=None)
    rt.holding_path=tmp_path/'holding.json';rt.holding_path.write_text('{"holding_uncertain":false}')
    ctrl=SimpleNamespace(at_start=True,_gripper_open_confirmed=True,gripper_holding_uncertain=False,close=lambda:None)
    run={'id':'r','mode':'real','controller':ctrl,'start_verified':True,'config_revision':'cfg',
        'config':{},'arrival':SimpleNamespace(settings=ArrivalSettings())}
    rt._cleanup(run)
    assert 'open_empty_evidence' in run and run['controller'] is None
    if invalidate=='marker':rt.holding_path.write_text('{"holding_uncertain":false,"external":true}')
    if invalidate=='config':run['config_revision']='changed'
    if invalidate=='holding':rt.holding_path.write_text('{"holding_uncertain":true}')
    seen=[];new=SimpleNamespace(_gripper_open_confirmed=False,gripper_holding_uncertain=False)
    def initialize():
        seen.append(new._gripper_open_confirmed)
        return {'start_verified':True,'control_completed':True}
    new.initialize_for_pick=initialize
    rt.controller_factory=lambda *a:new;rt._require_devices=lambda:None
    rt._initialize(run,1,checkpoint=lambda:None)
    assert seen==[invalidate is None] and 'open_empty_evidence' not in run


def test_recovery_wait_and_active_cancellation_reach_persistence_callback():
    from test_task_sequence_v2 import Harness,instance
    h=Harness(objects=[instance('battery','battery')]);saved=[]
    h.seq.step_recovery_enabled=True
    h.seq.initialize_before_observe=lambda *a,**k:{'control_completed':True,'start_verified':True,'empty_hand_confirmed':True}
    h.seq.persist_boundary=lambda:saved.append(h.seq.snapshot()['status'])
    h.begin();h.seq.advance()
    assert saved[-1]=='waiting_step_recovery' and h.commands==[]
    h=Harness();saved=[]
    h.seq.persist_boundary=lambda:saved.append(h.seq.snapshot()['status'])
    def execute(*a,**k):h.seq.cancel();return {}
    h.seq.execute=execute;h.begin();h.seq.advance()
    assert saved[-1]=='cancelled' and h.commands==[]


def test_persistence_failure_prevents_dispatch_and_records_failed_step():
    from test_task_sequence_v2 import Harness
    h=Harness();saved=[]
    def persist():
        state=h.seq.snapshot();saved.append(state['status'])
        if state['steps'][0]['status']=='executing':raise OSError('controlled disk failure')
    h.seq.persist_boundary=persist;h.begin();h.seq.advance()
    assert h.commands==[] and saved[-1]=='failed'


def test_legacy_serial_buffer_methods_and_failed_close_invalidate_open_evidence():
    from v0_mvp.robot import MyCobotController
    calls=[];port=SimpleNamespace(inWaiting=lambda:1,flushInput=lambda:calls.append('reset'))
    b=SimpleNamespace(_serial_port=port,get_angles=lambda:[0]*6,set_gripper_state=lambda *a:None)
    c=MyCobotController({},backend=b,sleep_fn=lambda _:None)
    assert c._try_get_angles()==[0]*6 and calls==['reset']
    c.open_gripper();assert c._gripper_open_confirmed
    def fail(*a):raise OSError('controlled send failure')
    b.set_gripper_state=fail
    with pytest.raises(RobotError):c.close_gripper()
    assert c._gripper_open_confirmed is False


def test_failed_vertical_dispatch_does_not_keep_continuation_proof():
    c,b,clock,_=controller();target=[100,0,180,-180,0,0]
    c.move_pose(target,stable_count=1,vertical_stage='grasp_lift',waypoint_index=1,intermediate=True)
    send=b.send_coords
    def fail(*a):raise OSError('controlled send failure')
    b.send_coords=fail
    with pytest.raises(RobotError,match='dispatch_failed'):
        c.move_pose(target,stable_count=1,vertical_stage='grasp_lift',waypoint_index=2)
    assert c._vertical_continuation is None
    b.send_coords=send;before=clock()
    c.move_pose(target,stable_count=1,vertical_stage='grasp_lift',waypoint_index=2)
    assert clock()-before==pytest.approx(4.25)


@pytest.mark.parametrize('fault,expected',[
    ('pregrasp_send',False),('descent_send',False),('last_close_checkpoint',False),
    ('close_persistence',False),('close_send',True),('after_close',True),('release_send',True),('none',False)])
def test_real_holding_persistence_matches_actual_close_dispatch(tmp_path,fault,expected):
    import json,threading
    from operator_console.runtime_v2 import ConsoleRuntimeV2
    from operator_console.events import atomic_json
    rt=object.__new__(ConsoleRuntimeV2);rt._lock=threading.RLock();rt._arm_motion_epoch=0
    rt.holding_path=tmp_path/'controller.json';c,b,_,_=controller();g,p=plans()
    run={'id':'r','mode':'real','controller':c,'events':[]};rt._runs={'r':run};saved=[];settled=[]
    atomic_json(rt.holding_path,{'holding_uncertain':False})
    def persist(run):
        value=json.loads(rt.holding_path.read_text())['holding_uncertain']
        if fault=='close_persistence' and value:raise OSError('controlled persistence failure')
        atomic_json(tmp_path/'run.json',{'holding_uncertain':c.gripper_holding_uncertain});saved.append(value)
    rt._save=persist
    def sink(stage,event,**detail):
        rt._event(run,stage,event,**detail)
        if event=='pre_grasp_settled':settled.append(True)
    c.event_sink=sink
    def checkpoint():
        if fault=='last_close_checkpoint' and settled:raise RuntimeError('controlled cancel before close dispatch')
    send=b.send_coords;grip=b.set_gripper_state
    def pose(target,*args):
        if fault=='pregrasp_send' and target==g.pregrasp_pose_mm_deg:raise OSError('pregrasp')
        if fault=='descent_send' and target==g.grasp_pose_mm_deg:raise OSError('descent')
        if fault=='after_close' and target==g.lift_pose_mm_deg:raise OSError('lift')
        return send(target,*args)
    def gripper(state,speed):
        if state==1:
            assert c.gripper_holding_uncertain and json.loads(rt.holding_path.read_text())['holding_uncertain']
            if fault=='close_send':raise OSError('close dispatch failed')
        if state==0 and fault=='release_send':raise OSError('release failed')
        return grip(state,speed)
    b.send_coords=pose;b.set_gripper_state=gripper
    if fault=='none':rt._execute(run,{'grasp':g,'place':p},initialize=False,checkpoint=checkpoint)
    else:
        with pytest.raises((RobotError,RuntimeError,OSError)):
            rt._execute(run,{'grasp':g,'place':p},initialize=False,checkpoint=checkpoint)
    assert c.gripper_holding_uncertain is expected
    assert json.loads(rt.holding_path.read_text())['holding_uncertain'] is expected
    assert json.loads((tmp_path/'run.json').read_text())['holding_uncertain'] is expected
    if not expected and fault!='none':assert ('gripper',1) not in b.calls
