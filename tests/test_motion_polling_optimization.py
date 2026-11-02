"""Motion tuning boundaries using synthetic clocks; never opens a device."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import math

import pytest
import yaml

from operator_console.direct_control_v2 import ArrivalSettings, DirectRobotControllerV2
from operator_console.robot_adapter import OperatorRobotController, planned_vertical_waypoint_count
from operator_console.assets import AssetRegistry
from v0_mvp.grasp import build_top_down_plan, GraspError
from v0_mvp.robot import MyCobotController, RobotError
from test_direct_control_v2 import CONFIG, controller, plans, Clock
from test_stage2_hybrid_control import make, POSE, Q


def tuned():
    cfg = deepcopy(CONFIG)
    cfg['robot'].update(pose_verify_stable_polls=1, motion_poll_interval_s=.5,
        pose_verify_orientation_tolerance_deg=8, motion_timeout_s=2)
    return cfg


def test_default_settings_and_cross_movement_dispatch_feedback_spacing():
    assert ArrivalSettings().pose_verify_stable_polls == 1
    assert ArrivalSettings().motion_poll_interval_s == .5
    assert ArrivalSettings().pose_verify_orientation_tolerance_deg == 8
    ctrl, backend, clock, _ = controller(config=tuned())
    sends, reads = [], []
    send = backend.send_coords
    get = backend.get_coords
    backend.send_coords = lambda *a: (sends.append(clock.now), send(*a))[-1]
    backend.get_coords = lambda: (reads.append(clock.now), get())[-1]
    target = [100, 0, 180, -180, 0, 0]
    ctrl.move_pose(target); ctrl.move_pose(target)
    assert sends == pytest.approx([0, 3.25])
    assert reads == pytest.approx([3.0, 6.25])


def test_missing_sample_is_log_only_and_not_retried():
    ctrl,backend,clock,events=controller(config=tuned());times=[]
    backend.get_coords=lambda:(times.append(clock.now),None)[-1]
    assert ctrl.move_pose([100,0,180,-180,0,0]) is None
    assert times==pytest.approx([3.0])


@pytest.mark.parametrize('strategy', ['linear', 'segmented'])
@pytest.mark.parametrize('delta', [0, 5, 80])
def test_only_grasp_final_waypoint_has_strict_gate_including_zero_distance(strategy, delta):
    cfg = tuned(); cfg['robot']['approach_strategy'] = strategy
    ctrl, _, _, _ = controller(config=cfg)
    end = [100, 0, 140, -180, 0, 0]; start = list(end); start[2] += delta
    commands = ctrl._vertical_commands(start, end, 'grasp_descent', 1, True)
    assert commands and commands[-1][1] == end
    assert all(c[3][0:2] == (0, 1) for c in commands)
    assert [c[3][2] for c in commands] == [False]*(len(commands)-1) + [True]
    for label in ['grasp_lift', 'place_descent', 'place_retreat']:
        assert not any(c[3][2] for c in ctrl._vertical_commands(start,end,label,1,False))


@pytest.mark.parametrize('xyz,accepted', [([0,0,19], False), ([0,0,6], False),
    ([8.01,0,0], False), ([6,0,5], False), ([6,0,4], True), ([8,0,0], True)])
def test_grasp_final_errors_do_not_gate_fixed_wait(xyz,accepted):
    ctrl,backend,clock,events=controller(config=tuned());target=[100,0,180,-180,0,0]
    actual=[target[i]+xyz[i] if i<3 else target[i] for i in range(6)]
    backend.get_coords=lambda:actual
    assert ctrl.move_pose(target,grasp_endpoint=True)==actual
    assert clock.now==pytest.approx(4.)


def test_full_program_fixed_wait_then_settles_before_close():
    cfg=tuned();cfg['robot']['approach_strategy']='segmented'
    ctrl,backend,clock,events=controller(config=cfg);grasp,place=plans();get=backend.get_coords
    arrivals=[];closed=[]
    def read():
        actual=get()
        if backend.coords==grasp.grasp_pose_mm_deg:
            actual[2]+=19;arrivals.append(clock.now)
        return actual
    backend.get_coords=read;grip=backend.set_gripper_state
    def gripper(state,speed):
        if state==1:closed.append(clock.now)
        grip(state,speed)
    backend.set_gripper_state=gripper
    ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    assert len(arrivals)==1 and closed[0]-arrivals[0]==pytest.approx(.35)
    assert [d['z_error_mm'] for e,d in events if e=='feedback_sampled' and d.get('grasp_endpoint')]==[19]


def test_missing_final_feedback_does_not_block_gripper():
    ctrl,backend,_,_=controller(config=tuned());grasp,place=plans()
    backend.get_coords=lambda:None
    assert ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)['control_completed']
    assert ('gripper',1) in backend.calls


@pytest.mark.parametrize('cls',[DirectRobotControllerV2,OperatorRobotController])
@pytest.mark.parametrize('offset,angle,accepted',[(20.267,0,True),(35,15,True),(35.01,0,False),(0,15.01,False)])
def test_joint_tcp_gate_applies_only_to_legacy_controller(cls,offset,angle,accepted):
    ctrl,backend=make(cls);backend.joint_pose[0]+=offset;backend.joint_pose[5]+=angle
    if cls is DirectRobotControllerV2:
        backend.get_coords=lambda:pytest.fail('unexpected secondary TCP read')
        ctrl.move_hybrid_transfer(POSE,label='tcp_check',taught_angles=Q)
        assert ctrl.transfer_evidence[-1]['arrival_verified'] is False
    elif accepted:
        ctrl.move_hybrid_transfer(POSE,label='tcp_check',taught_angles=Q)
        assert ctrl.transfer_evidence[-1]['tcp_position_tolerance_mm']==35
    else:
        with pytest.raises(RobotError):ctrl.move_hybrid_transfer(POSE,label='tcp_check',taught_angles=Q)
    assert len(backend.calls)==1


@pytest.mark.parametrize('source',['dispatch','feedback'])
def test_legacy_cartesian_rejects_late_success_with_actual_deadline(source):
    clock=Clock();reads=[];target=[100,0,180,-180,0,0]
    def send(*a):
        if source=='dispatch':clock.sleep(1.1)
    def read():
        reads.append(clock.now)
        if source=='feedback':clock.sleep(1.1)
        return target
    backend=SimpleNamespace(send_coords=send,get_coords=read)
    ctrl=MyCobotController({'motion_timeout_s':1,'pose_verify_stable_polls':1},
        backend=backend,clock_fn=clock,sleep_fn=clock.sleep)
    with pytest.raises(RobotError,match='Cartesian motion timeout'):ctrl.move_pose(target)
    assert len(reads)==(0 if source=='dispatch' else 1)


@pytest.mark.parametrize('sync',[False,True])
@pytest.mark.parametrize('feedback',[None,[90]*6,[0,0,0,-90,0,-45]])
def test_legacy_start_requires_fresh_joint_arrival_even_after_sync_success(sync,feedback):
    clock=Clock();events=[]
    backend=SimpleNamespace(send_angles=lambda *a:None,get_angles=lambda:feedback)
    if sync:backend.sync_send_angles=lambda *a,**k:1
    cfg={'start_angles_deg':[0,0,0,-90,0,-45],'motion_timeout_s':1}
    ctrl=MyCobotController(cfg,backend=backend,clock_fn=clock,sleep_fn=clock.sleep,
        event_sink=lambda stage,event,**k:events.append(event))
    if feedback==cfg['start_angles_deg']:
        assert ctrl.move_start()==feedback
        assert 'robot_start_completed' in events
    else:
        ctrl._last_verified_angles=cfg['start_angles_deg']
        with pytest.raises(RobotError):ctrl.move_start()
        assert 'robot_start_completed' not in events


@pytest.mark.parametrize('target,expected',[('pen',153),('eraser',149),('battery',151),(None,151)])
def test_per_target_z_offsets_are_added_to_global_offset(target,expected):
    cfg={'workspace_mm':{'x':[-280,280],'y':[-280,280],'z':[70,320]},
        'grasp_z_offset_mm':1,'target_z_offsets_mm':{'pen':2,'eraser':-2}}
    assert build_top_down_plan([.1,0,.15],0,cfg,canonical_target=target).grasp_pose_mm_deg[2]==expected


def test_table_and_config_floor_take_highest_limit_before_workspace_check():
    cfg={'workspace_mm':{'x':[-280,280],'y':[-280,280],'z':[70,320]},
        'grasp_z_offset_mm':-20,'target_z_offsets_mm':{'pen':-5},'min_grasp_z_mm':115}
    plan=build_top_down_plan([.1,0,.12],0,cfg,canonical_target='pen',table_z_mm=112.7)
    assert plan.grasp_pose_mm_deg[2]==115 and plan.pregrasp_pose_mm_deg[2]==195
    with pytest.raises(GraspError):build_top_down_plan([.1,0,.12],0,cfg,table_z_mm=300)


@pytest.mark.parametrize('value',[True,float('nan'),float('inf'),'2'])
def test_bad_z_offset_and_floor_fail_before_building_plan(value):
    cfg={'workspace_mm':{'x':[-280,280],'y':[-280,280],'z':[70,320]}}
    for key in ['target_z_offsets_mm','min_grasp_z_mm']:
        bad=cfg|{key:({'pen':value} if key=='target_z_offsets_mm' else value)}
        with pytest.raises(GraspError):build_top_down_plan([.1,0,.15],0,bad,canonical_target='pen')


@pytest.mark.parametrize('key,value',[('target_z_offsets_mm',{'pen':2}),('min_grasp_z_mm',113),
    ('table_z_mm',112.7),('coarse_step_mm',30),('motion_poll_interval_s',.75)])
def test_motion_fingerprint_covers_new_settings(tmp_path,key,value):
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/v0.yaml').read_text(encoding='utf-8'))
    changed=deepcopy(cfg);changed['grasp' if key.endswith('_z_mm') or key=='target_z_offsets_mm' else 'robot'][key]=value
    registry=AssetRegistry(tmp_path)
    def digest(c):return next(x['asset_id'] for x in registry._configuration_assets(c) if x['kind']=='motion_safety_profile')
    assert digest(cfg)!=digest(changed)


def test_local_sdk_res_retries_transport_three_times_without_opening_serial():
    from pymycobot.mycobot import MyCobot
    writes=[]
    fake=SimpleNamespace(_write=lambda data:writes.append(data),_flatten=lambda data:data,_read=lambda genre:b'')
    assert MyCobot._res(fake,[1],True,0)==-1
    assert len(writes)==3


def test_waypoint_count_is_shared_and_does_not_construct_controller(monkeypatch):
    monkeypatch.setattr(MyCobotController,'__init__',lambda *a,**k:pytest.fail('constructed controller'))
    assert planned_vertical_waypoint_count({},[0,0,200,0,0,0],[0,0,120,0,0,0],fine_near_end=True)==4


@pytest.mark.parametrize('destination',['relative','named'])
def test_v2_geometry_threads_canonical_target_and_table_floor(tmp_path,monkeypatch,destination):
    from test_v2_console_integration import Harness, task
    from v0_mvp import grasp as grasp_module
    calls=[];original=grasp_module.build_top_down_plan
    def spy(point,yaw,cfg,**kwargs):
        plan=original(point,yaw,cfg,**kwargs)
        calls.append((kwargs,plan))
        return plan
    monkeypatch.setattr(grasp_module,'build_top_down_plan',spy)
    item=task()
    if destination=='named':item.update(destination_type='named',destination_name='右盒子',reference=None,relation=None)
    harness=Harness(tmp_path,tasks=[item])
    try:
        harness.cfg['grasp']['target_z_offsets_mm']={'eraser':2.0}
        harness.create();harness.confirm();state=harness.wait('completed','failed')
        assert state['status']=='completed',state
        assert calls and all(c[0].get('canonical_target') for c in calls)
        assert all(c[0].get('table_z_mm') is not None for c in calls)
        assert any(c[0]['canonical_target']=='eraser' for c in calls)
    finally:harness.close()


def test_cancellation_during_pregrasp_settle_prevents_close():
    ctrl,backend,clock,_=controller(config=tuned());grasp,place=plans()
    sleep=ctrl.sleep;settled=[]
    def wait(seconds):
        sleep(seconds)
        if seconds==.35:settled.append(True)
    ctrl.sleep=wait
    ctrl.checkpoint=lambda: (_ for _ in ()).throw(RobotError('cancelled')) if settled else None
    with pytest.raises(RobotError,match='cancelled'):
        ctrl.execute_pick_and_place(grasp,place,geometry_passed=True,initialize=False)
    assert ('gripper',1) not in backend.calls and not ctrl.gripper_holding_uncertain
