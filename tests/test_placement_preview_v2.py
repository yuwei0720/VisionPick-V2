"""Analytic projection, rotated gap, shared plan and version safety; no devices."""
from copy import deepcopy
from pathlib import Path
import json
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from integration.relative_placement import (plan_relative_place,project_robot_points_mm,
    placement_preview_geometry,resolve_place_orientation,_footprint,footprint_yaw_deg,
    polygon_distance_mm,clearance_envelope_obb,convex_polygons_intersect,_surface_clear,
    _center_advance_for_gap)
from integration.scene_grounding import SceneGroundingError
from operator_console.simulation import synthetic_scene
from operator_console.task_sequence_v2 import ObservationV2
from test_v2_console_integration import Harness,task

ROOT=Path(__file__).resolve().parents[1]


def case():return json.loads((ROOT/'tests/data/relative_place_s3_demo.json').read_text(encoding='utf-8'))


def plan(c,**kwargs):
    ds,frame,deproject,project=synthetic_scene(c)
    return plan_relative_place(c['task'],ds,frame,deproject=deproject,project=project,
        config=c['config'],surface=c['surface'],gap_mm=40.,captured_at_unix_ms=1000,now_unix_ms=1000,**kwargs)


@pytest.mark.parametrize('theta',[0,25,90,180])
def test_inverse_extrinsic_projection_against_analytic_camera_points(theta):
    a=math.radians(theta);r=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
    t=np.array([.13,-.22,.55]);transform=np.eye(4);transform[:3,:3]=r;transform[:3,3]=t
    camera=np.array([[.1,-.05,1.],[0,0,.5],[-.1,.2,2.]])
    robot=(camera@r.T+t)*1000
    cfg={'calibration':{'T_robot_from_camera':transform.tolist()}}
    actual=project_robot_points_mm(robot,cfg,{'fx':600.,'fy':620.,'cx':320.,'cy':240.})
    expected=np.column_stack((camera[:,0]/camera[:,2]*600+320,camera[:,1]/camera[:,2]*620+240))
    assert np.max(np.abs(actual-expected))<1e-10


@pytest.mark.parametrize('z',[0,-1])
def test_projection_rejects_points_on_or_behind_camera(z):
    cfg={'calibration':{'T_robot_from_camera':np.eye(4).tolist()}}
    with pytest.raises(ValueError,match='behind_camera'):project_robot_points_mm([[0,0,z]],cfg,dict(fx=600.,fy=600.,ppx=320.,ppy=240.))


@pytest.mark.parametrize('scenario,policies',[
    ('relative',['align_reference','keep_target','fixed_deg']),
    ('self_relative',['keep_target','fixed_deg']),
    ('swap',['match_initial','keep_target','fixed_deg']),
    ('named',['profile_pose','keep_target'])])
def test_all_scenario_policies_and_custom_override(scenario,policies):
    values={'align_reference':35.,'keep_target':-20.,'fixed_deg':72.,'match_initial':63.,'profile_pose':-45.}
    for policy in policies:
        cfg={'orientation':{'place_orientation_policy_'+scenario:policy,'fixed_place_yaw_deg':72.}}
        got,yaw=resolve_place_orientation(cfg,scenario,-20.,reference_yaw=35.,initial_yaw=63.,profile_yaw=-45.)
        assert got==policy and yaw==values[policy]
        assert resolve_place_orientation(cfg,scenario,-20.,reference_yaw=35.,initial_yaw=63.,profile_yaw=-45.,override={'policy':'fixed_deg','rz_deg':-137.})==('fixed_deg',-137.)


@pytest.mark.parametrize('relation',['left_of','right_of','in_front_of','behind','near'])
@pytest.mark.parametrize('policy,angle',[('keep_target',0.),('align_reference',-20.),('fixed_deg',72.)])
def test_rotation_precedes_alpha_and_envelope_for_relative(relation,policy,angle):
    c=case();c['task']['spatial_relation']=relation
    c['config']['orientation'].update(place_orientation_policy_relative=policy,fixed_place_yaw_deg=72.)
    result=plan(c,exact_gap=True)
    assert result['place_yaw_deg']==pytest.approx(angle,abs=1e-4)
    ds,frame,deproject,_=synthetic_scene(c);transform=np.asarray(c['config']['calibration']['T_robot_from_camera'])
    source,_=_footprint(ds[1],frame,deproject,transform,c['config'],2)
    reference,_=_footprint(ds[0],frame,deproject,transform,c['config'],2)
    rad=math.radians(angle-result['grasp_yaw_deg']);rotation=np.array([[math.cos(rad),-math.sin(rad)],[math.sin(rad),math.cos(rad)]])
    offsets=(source-source.mean(axis=0))@rotation.T
    actual=np.asarray(result['destination_footprint_robot_xy_mm']);center=np.asarray(result['destination_center_robot_xy_mm'])
    assert actual-center==pytest.approx(offsets,abs=1e-4)
    direction=np.asarray(result['direction_axis_robot_xy']);rc=reference.mean(axis=0)
    alpha=float((center-rc)@direction)
    assert center==pytest.approx(rc+direction*alpha,abs=1e-4)
    assert polygon_distance_mm(actual,reference)==pytest.approx(45.,abs=1e-6)
    assert polygon_distance_mm(offsets+rc+direction*(alpha-.001),reference)<45.
    assert result['center_distance_mm']==pytest.approx(alpha)
    assert result['projected_gap_mm']==pytest.approx(np.min(actual@direction)-np.max(reference@direction))
    assert result['nearest_edge_gap_mm']==pytest.approx(45.,abs=1e-6)
    envelope=np.asarray(result['clearance_envelope_robot_xy_mm'])
    edges=np.roll(actual,-1,axis=0)-actual
    axis=edges[np.argmax(np.linalg.norm(edges,axis=1))];axis/=np.linalg.norm(axis)
    axes=np.array([axis,[-axis[1],axis[0]]])
    assert (envelope@axes.T).min(axis=0)==pytest.approx((actual@axes.T).min(axis=0)-15)
    assert (envelope@axes.T).max(axis=0)==pytest.approx((actual@axes.T).max(axis=0)+15)


@pytest.mark.parametrize('policy,angle',[('keep_target',0),('fixed_deg',72)])
def test_self_relative_rotates_without_changing_center_displacement(policy,angle):
    c=case();c['task'].update(destination_type='self_relative',reference_object=None)
    c['config']['orientation'].update(place_orientation_policy_self_relative=policy,fixed_place_yaw_deg=72.)
    ds,frame,d,p=synthetic_scene(c)
    result=plan_relative_place(c['task'],ds,frame,deproject=d,project=p,config=c['config'],surface=c['surface'],
        gap_mm=None,self_displacement_mm=40.,captured_at_unix_ms=1000,now_unix_ms=1000)
    assert result['place_yaw_deg']==angle and result['planned_displacement_mm']==pytest.approx(40.)
    source,_=_footprint(ds[1],frame,d,np.asarray(c['config']['calibration']['T_robot_from_camera']),c['config'],2)
    rad=math.radians(angle);rot=np.array([[math.cos(rad),-math.sin(rad)],[math.sin(rad),math.cos(rad)]])
    assert np.asarray(result['destination_footprint_robot_xy_mm'])-result['destination_center_robot_xy_mm']==pytest.approx((source-source.mean(axis=0))@rot.T)


@pytest.mark.parametrize('policy,angle',[('match_initial',63.),('keep_target',0.),('fixed_deg',72.)])
def test_swap_uses_original_yaw_before_rotating_footprint(policy,angle):
    c=case();c['task']['spatial_relation']='at_initial';c['config']['orientation'].update(place_orientation_policy_swap=policy,fixed_place_yaw_deg=72.)
    anchor={'class_name':'pen','xyz_mm':[-110.,60.,0.],'footprint_robot_xy_mm':[[-130,50],[-90,50],[-90,70],[-130,70]],'yaw_deg':63.}
    result=plan(c,initial_reference=anchor)
    assert result['place_yaw_deg']==angle and result['destination_center_robot_xy_mm']==anchor['xyz_mm'][:2]
    poly=np.asarray(result['destination_footprint_robot_xy_mm']);edges=np.roll(poly,-1,axis=0)-poly
    axis=edges[np.argmax(np.linalg.norm(edges,axis=1))]
    assert abs(np.dot(axis/np.linalg.norm(axis),[math.cos(math.radians(angle)),math.sin(math.radians(angle))]))==pytest.approx(1.)


def preview(h):
    state=h.create();run=h.runtime._runs[h.run_id]
    obs=h.runtime._observe(run,1);run['preview']=obs.display
    h.runtime._refresh_placement_previews(run)
    return h.runtime.state(h.run_id),run,obs


def test_preview_and_execution_geometry_share_custom_angle_and_plan_version(tmp_path):
    h=Harness(tmp_path)
    try:
        state,run,obs=preview(h);version=state['pending']['version'];assert h.backends==[]
        assert state['placement_previews'][0]['feasible']
        status,new,_=h.action('placement_orientation',dict(version=version,step_id=1,policy='fixed_deg',rz_deg=72.))
        assert status==200 and new['pending']['version']==version+1
        overlay=new['placement_previews'][0];assert overlay['rz_deg']==72.
        selected={'target':obs.instances[1]['instance_id'],'reference':obs.instances[0]['instance_id']}
        prepared=h.runtime._plan_placement(run,h.tasks[0],obs,selected,orientation_override={'policy':'fixed_deg','rz_deg':72.})
        assert prepared['place'].place_pose_mm_deg[5]==72.
        assert overlay['polygon_mm']==pytest.approx(np.column_stack((prepared['geometry']['destination_footprint_robot_xy_mm'],np.zeros(4))))
        code,_,_=h.action('placement_orientation',dict(version=version,step_id=1,policy='keep_target',rz_deg=None));assert code==400
        code,_,_=h.action('confirm',{'version':version});assert code==400
        assert h.backends==[]
    finally:h.close()


def test_failed_candidate_is_diagnostic_only_not_a_placement_overlay(tmp_path):
    h=Harness(tmp_path)
    try:
        h.cfg['grasp']['workspace_mm']['y']=[-190,30]
        state,run,obs=preview(h);p=state['placement_previews'][0]
        assert not p['feasible'] and p['reason']
        assert 'polygon_px' not in p and 'envelope_px' not in p and 'center_mm' not in p
        assert h.backends==[]
    finally:h.close()


def test_semantic_only_preview_reports_unavailable_without_requiring_table_or_calibration(tmp_path):
    h=Harness(tmp_path)
    try:
        _,run,obs=preview(h)
        obs.payload['semantic_only']=True
        obs.payload['surface']={}
        obs.payload['config']['calibration']={}
        h.service.table_surface.require=lambda cfg:pytest.fail('semantic-only preview must not require geometry')
        h.runtime._refresh_placement_previews(run)
        assert run['placement_previews'][0]['reason']=='semantic_only_not_geometry_validated'
        assert run['placement_previews'][0]['feasible'] is False
        assert h.backends==[]
    finally:h.close()


def test_semantic_preview_uses_available_table_without_another_capture(tmp_path):
    h=Harness(tmp_path)
    try:
        _,run,obs=preview(h)
        obs.payload.update(semantic_only=True,surface={})
        before=list(h.order);h.runtime._refresh_placement_previews(run)
        p=run['placement_previews'][0]
        assert p['feasible'] and len(p['polygon_px'])==len(p['envelope_px'])==4
        assert p['authorizes_motion'] is False and h.order==before and h.backends==[]
    finally:h.close()


def test_outside_reference_with_inside_destination_has_consistent_preview_and_execution(tmp_path):
    h=Harness(tmp_path,tasks=[task('above')])
    try:
        h.case['objects'][0]['center_uv']=[572.,240.]
        h.cfg['grasp']['workspace_mm']['x']=[-250,200]
        state,run,obs=preview(h);ref=next(i for i in obs.instances if i['class_name']=='pen')
        assert not ref['eligible'] and ref['reference_eligible']
        assert state['placement_previews'][0]['feasible'],state['placement_previews']
        assert h.confirm()[0]==200
        assert h.wait('completed','failed','waiting_step_recovery')['status']=='completed'
    finally:h.close()


def test_outside_source_cannot_be_green_in_preview(tmp_path):
    h=Harness(tmp_path)
    try:
        h.cfg['grasp']['workspace_mm']['x']=[-250,90]
        state,_,_=preview(h)
        assert state['placement_previews'][0]['feasible'] is False
        assert state['placement_previews'][0]['reason']=='no_eligible_target_instance'
        assert h.backends==[]
    finally:h.close()


@pytest.mark.parametrize('policy,angle',[('profile_pose',45.),('keep_target',0.)])
def test_named_yaw_rotation_and_reject_stale_taught_joint_pair(tmp_path,policy,angle):
    t={'target':'eraser','destination_type':'named','destination_name':'右盒子','reference':None,'relation':None}
    h=Harness(tmp_path,tasks=[t])
    try:
        for key in ('pre_place_pose_mm_deg','place_pose_mm_deg','retreat_pose_mm_deg'):h.station[key][5]=45.
        h.station['pre_place_joint_angles_deg']=[1.]*6
        h.cfg['orientation']['place_orientation_policy_named']=policy
        state,run,obs=preview(h);p=state['placement_previews'][0]
        assert p['rz_deg']==angle and len(p['polygon_px'])==4
        actual=h.runtime._plan_placement(run,t,obs,{'target':obs.instances[1]['instance_id']})
        assert actual['place'].place_pose_mm_deg[5]==angle
        if policy=='keep_target':assert actual['place'].pre_place_joint_angles_deg is None
    finally:h.close()


def test_three_step_swap_forecast_preserves_initial_yaw_and_numbering(tmp_path):
    tasks=[task('near'),task('at_initial',target='pen',reference='eraser'),task('at_initial',target='eraser',reference='pen')]
    h=Harness(tmp_path,tasks=tasks)
    try:
        h.cfg['orientation'].update(place_orientation_policy_relative='align_reference',place_orientation_policy_swap='match_initial')
        state,run,obs=preview(h);ps=state['placement_previews']
        assert [p['step_id'] for p in ps]==[1,2,3]
        assert [p['rz_deg'] for p in ps]==pytest.approx([-20.,0.,-20.],abs=1e-4)
        assert all(len(p['polygon_px'])==4 for p in ps)
        assert ps[1]['predicted_scene'] and ps[2]['requires_fresh_observation']
        assert h.backends==[]
    finally:h.close()


def swap_fixture(tmp_path, *, ordinary=False, no_space=False):
    objects=[{'class_name':'battery','center_uv':[320.,240.],'size_px':[48.,30.],'angle_deg':0.,'confidence':.95},
        {'class_name':'pen','center_uv':[320.,330.],'size_px':[120.,12.],'angle_deg':20.,'confidence':.95}]
    tasks=[task('near',target='pen',reference='battery')]
    if not ordinary:tasks += [task('at_initial',target='battery',reference='pen'),task('at_initial',target='pen',reference='battery')]
    h=Harness(tmp_path,tasks=tasks,objects=objects)
    h.cfg['orientation'].update(place_orientation_policy_relative='align_reference',place_orientation_policy_swap='match_initial')
    if no_space:h.cfg['relative_placement']={'near':{'candidate_directions':['left'],'min_offset_mm':50,'max_offset_mm':50}}
    return h


@pytest.mark.parametrize('future_rz',[None,72.])
def test_swap_temporary_reserves_original_sites_and_future_grippers(tmp_path,future_rz):
    h=swap_fixture(tmp_path)
    try:
        state,run,obs=preview(h)
        if future_rz is not None:
            h.action('placement_orientation',dict(version=state['pending']['version'],step_id=2,policy='fixed_deg',rz_deg=future_rz))
        ps=h.runtime.state(h.run_id)['placement_previews']
        assert all(p['feasible'] for p in ps),ps
        selected={role:next(i['instance_id'] for i in obs.instances if i['class_name']==name)
            for role,name in [('target','pen'),('reference','battery')]}
        g=h.runtime._plan_placement(run,h.tasks[0],obs,selected)['geometry']
        assert ps[0]['center_mm'][:2]==pytest.approx(g['destination_center_robot_xy_mm'])
        assert len(g['swap_reserved_regions'])==6
        destination=np.asarray(g['destination_footprint_robot_xy_mm'])
        envelope=np.asarray(g['clearance_envelope_robot_xy_mm'])
        for region in g['swap_reserved_regions']:
            assert not convex_polygons_intersect(envelope if region['check_envelope'] else destination,np.asarray(region['polygon_mm']))
        assert np.linalg.norm(destination.mean(0)-np.array([0,-75]))>100
        assert h.backends==[]
    finally:h.close()


def test_swap_original_regions_are_full_gripper_envelopes(tmp_path):
    h=swap_fixture(tmp_path)
    try:
        state,run,obs=preview(h)
        assert state['placement_previews'][0]['feasible']
        selected={role:next(i['instance_id'] for i in obs.instances if i['class_name']==name)
            for role,name in [('target','pen'),('reference','battery')]}
        geom=h.runtime._plan_placement(run,h.tasks[0],obs,selected)['geometry']
        regions={r['label']:r for r in geom['swap_reserved_regions']}
        for name in ('pen','battery'):
            anchor=next(i for i in obs.instances if i['class_name']==name)
            original=np.asarray(anchor['footprint_robot_xy_mm'])
            expected=clearance_envelope_obb(original,h.case['surface']['gripper_padding_mm'],
                h.case['surface']['error_margin_mm'],h.cfg['grasp'].get('tcp_xy_offset_mm',[0,0]))
            assert np.asarray(regions[name+'_initial']['polygon_mm'])==pytest.approx(expected)
            assert regions[name+'_initial']['check_envelope'] is True
            # The reserved original region is wider than its object footprint.
            assert np.ptp(expected,axis=0).max() > np.ptp(original,axis=0).max()
    finally:h.close()


def test_failed_swap_preview_never_forecasts_from_rejected_temporary_site(tmp_path):
    h=swap_fixture(tmp_path,no_space=True)
    try:
        state,_,_=preview(h)
        first,second,third=state['placement_previews']
        assert first['feasible'] is False and first['reason']=='swap_temporary_area_reserved'
        assert all(p['reason']=='previous_step_geometry_failed' for p in (second,third))
        assert all('center_px' not in p and 'polygon_px' not in p for p in (first,second,third))
    finally:h.close()


def test_relative_xy_surface_and_workspace_check_centre_not_envelope():
    baseline=case()
    original=plan(baseline)
    center=np.asarray(original['destination_center_robot_xy_mm'])
    envelope=np.asarray(original['clearance_envelope_robot_xy_mm'])
    upper=float((center[1]+envelope[:,1].max())/2)
    assert center[1]<upper<envelope[:,1].max()
    # Independently constrain both work area boundaries. The TCP centre has
    # zero offset in this synthetic fixture and remains within either bound.
    for boundary in ('surface','workspace'):
        c=case()
        if boundary=='surface':c['surface']['bounds_mm']['y'][1]=upper
        else:c['config']['grasp']['workspace_mm']['y'][1]=upper
        result=plan(c)
        assert result['destination_center_robot_xy_mm']==pytest.approx(center)
        assert max(p[1] for p in result['clearance_envelope_robot_xy_mm'])>upper
    # A centre genuinely outside either boundary is still rejected.
    for boundary,reason in [('surface','destination_outside_surface'),('workspace','destination_outside_workspace')]:
        c=case()
        if boundary=='surface':c['surface']['bounds_mm']['y'][1]=float(center[1]-1.)
        else:c['config']['grasp']['workspace_mm']['y'][1]=float(center[1]-1.)
        with pytest.raises(SceneGroundingError,match=reason):plan(c)


def test_ordinary_near_can_overlap_its_vacated_source(tmp_path):
    h=swap_fixture(tmp_path,ordinary=True)
    try:
        state,run,obs=preview(h)
        p=state['placement_previews'][0]
        assert p['feasible'] and p['center_mm'][:2]==pytest.approx([0,-72.5],abs=1e-4)
        assert h.runtime._plan_placement(run,h.tasks[0],obs,{'target':obs.instances[1]['instance_id'],
            'reference':obs.instances[0]['instance_id']})['geometry']['swap_reserved_regions']==[]
    finally:h.close()


def test_swap_no_temporary_space_rejects_before_any_pick(tmp_path):
    h=swap_fixture(tmp_path,no_space=True)
    try:
        state,_,_=preview(h)
        assert state['placement_previews'][0]['reason']=='swap_temporary_area_reserved'
        assert h.confirm()[0]==200
        state=h.wait('failed')
        assert state['sequence']['error']['reason']=='swap_temporary_area_reserved'
        assert all(call[0] not in {'coords','gripper'} or call==('gripper',0) for call in h.backends[0].calls)
    finally:h.close()


@pytest.mark.parametrize('policy,sdk',[('segmented_mode0',0),('linear_mode1',1),('single_mode0',0)])
@pytest.mark.parametrize('step,count',[(25.,4),(0.,1)])
def test_adapter_vertical_dispatch_uses_same_policy_and_equal_partition(policy,sdk,step,count):
    from operator_console.robot_adapter import OperatorRobotController
    from test_direct_control_v2 import Clock
    from operator_console.simulation import FakeMyCobot
    clock=Clock();b=FakeMyCobot();c=OperatorRobotController({'vertical_motion_mode':policy,'vertical_segment_step_mm':step,'coordinate_speed':20,
        'motion_timeout_s':1.,'motion_poll_interval_s':.1,'pose_verify_stable_polls':1},backend=b,clock_fn=clock,sleep_fn=clock.sleep)
    c.move_vertical_segmented([100,0,240,-180,0,0],[100,0,160,-180,0,0],label='grasp_descend',fine_near_end=True)
    if policy!='segmented_mode0':count=1
    sends=[x for x in b.calls if x['method']=='send_coords'];assert len(sends)==count and all(x['mode']==sdk for x in sends)
    zs=[240]+[x['coords'][2] for x in sends];assert np.diff(zs)==pytest.approx([-80/count]*count)
    assert c.transfer_evidence[-1]['vertical_motion_mode']==policy and c.transfer_evidence[-1]['sdk_mode']==sdk


def test_top_orientation_rotates_the_landing_footprint_before_clearance():
    c=case();c['task'].update(reference_object='box',spatial_relation='on_top_of')
    c['objects'][0].update(class_name='box',size_px=[150,150],angle_deg=0,depth_m=.47)
    c['objects'][1]['depth_m']=.49
    c['config']['orientation'].update(place_orientation_policy_relative='fixed_deg',fixed_place_yaw_deg=72.)
    result=plan(c);assert result['place_yaw_deg']==72.
    poly=np.asarray(result['destination_footprint_robot_xy_mm']);edges=np.roll(poly,-1,axis=0)-poly;axis=edges[np.argmax(np.linalg.norm(edges,axis=1))]
    assert abs(np.dot(axis/np.linalg.norm(axis),[math.cos(math.radians(72)),math.sin(math.radians(72))]))==pytest.approx(1.)
    assert result['destination_center_robot_xy_mm']==pytest.approx([0,0])



def test_refreshed_scene_can_preview_after_local_answer_without_retake(tmp_path):
    h=Harness(tmp_path,tasks=[task(reference=None)])
    try:
        state=h.create();assert state['status']=='waiting_answer'
        code,_,_=h.action('refresh_scene',{'version':state['pending']['version']});assert code==200
        waiting=h.wait('waiting_answer');q=waiting['pending']['question']
        assert waiting['placement_previews']==[]
        code,new,_=h.action('answer',dict(version=q['version'],task_index=q['task_index'],slot=q['slot'],value='pen'))
        assert code==200 and new['status']=='waiting_confirmation' and len(new['placement_previews'])==1
        assert new['placement_previews'][0]['feasible'] and h.backends==[]
        assert h.order==[('capture',1)] and len(h.calls)==1
    finally:h.close()


def test_real_preview_uses_new_engineering_geometry_values_on_existing_frame(tmp_path):
    h=Harness(tmp_path)
    try:
        state,run,obs=preview(h);run['mode']='real'
        run['config']['grasp']['grasp_z_offset_mm']=12.
        selected={'target':obs.instances[1]['instance_id'],'reference':obs.instances[0]['instance_id']}
        result=h.runtime._plan_placement(run,h.tasks[0],obs,selected)
        assert result['grasp'].grasp_pose_mm_deg[2]==12.
        assert obs.payload['config']['grasp']['grasp_z_offset_mm']!=12.
    finally:h.close()


@pytest.mark.parametrize('name',['tests/test_placement_preview_v2.py','tests/test_placement_preview_v2.js','tests/placement_preview_v2_browser.cjs'])
def test_delivery_accepts_explicit_verification_files_only(tmp_path,name):
    from package_project import load_selection
    p=tmp_path/name;p.parent.mkdir(parents=True);p.write_text('fixture',encoding='utf-8')
    whitelist=tmp_path/'whitelist.json';whitelist.write_text(json.dumps({'schema_version':1,'files':[name],'verification_files':[name]}),encoding='utf-8')
    assert [n for n,_ in load_selection(tmp_path,whitelist)]==[name]


@pytest.mark.parametrize('name,optin',[('tests/unlisted.py',False),('tests/credentials/leak.py',True),('tests/.env.py',True),('../outside.py',True),('src/main.py',True)])
def test_delivery_does_not_allow_arbitrary_tests_or_credential_paths(tmp_path,name,optin):
    from package_project import load_selection
    if '..' not in name:
        p=tmp_path/name;p.parent.mkdir(parents=True);p.write_text('fixture',encoding='utf-8')
    data={'schema_version':1,'files':[name]}
    if optin:data['verification_files']=[name]
    whitelist=tmp_path/'whitelist.json';whitelist.write_text(json.dumps(data),encoding='utf-8')
    with pytest.raises(ValueError):load_selection(tmp_path,whitelist)


@pytest.mark.parametrize('angle',[0,30,40,53,90])
def test_parallel_long_obbs_have_analytic_nearest_distance_and_rotating_envelope(angle):
    # Independent parallel-rectangle oracle: for an outward vertical ray,
    # closest points lie on long sides until their finite ends become closest.
    rad=math.radians(angle);axis=np.array([math.cos(rad),math.sin(rad)])
    normal=np.array([-axis[1],axis[0]])
    def rectangle(length,width):
        return np.array([[-length/2,-width/2],[length/2,-width/2],
                         [length/2,width/2],[-length/2,width/2]])@np.array([axis,normal])
    ref,target=rectangle(140,10),rectangle(150,20);direction=np.array([0.,1.])
    alpha=_center_advance_for_gap(ref,target,direction,55.)
    # Distance between two axis-aligned boxes in their shared local frame.
    along=max(0.,alpha*abs(direction@axis)-145.)
    across=max(0.,alpha*abs(direction@normal)-15.)
    assert math.hypot(along,across)==pytest.approx(55.,abs=1e-6)
    landing=target+alpha*direction
    env=clearance_envelope_obb(landing,10.,5.,[0,0])
    assert abs(np.linalg.det(np.array([env[1]-env[0],env[3]-env[0]])))==pytest.approx(180*50)
    assert abs(np.dot((env[1]-env[0])/np.linalg.norm(env[1]-env[0]),axis))==pytest.approx(1.)


@pytest.mark.parametrize('offset',[[12,-7],[-12,7],[20,20],[0,0]])
def test_tcp_compensation_contains_both_offset_footprints(offset):
    import cv2
    a=math.radians(40);rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    poly=np.array([[-75,-10],[75,-10],[75,10],[-75,10]])@rot.T
    env=clearance_envelope_obb(poly,10,5,offset).astype(np.float32)
    for shift in [np.array(offset),-np.array(offset),np.zeros(2)]:
        assert all(cv2.pointPolygonTest(env,tuple(p),True)>=15-1e-5 for p in poly+shift)


def test_polygon_distance_handles_intersection_containment_touch_and_zero_gap():
    a=np.array([[-2,-1],[2,-1],[2,1],[-2,1]],dtype=float)
    for b in [a,a*.5,a+[4,0]]:
        assert polygon_distance_mm(a,b)==0 and convex_polygons_intersect(a,b)
    alpha=_center_advance_for_gap(a,a,np.array([1.,0.]),0.)
    assert alpha==pytest.approx(4.)
    assert not convex_polygons_intersect(a,a+[alpha+1e-5,0])
    assert polygon_distance_mm(a,a+[7,6])==pytest.approx(5.)


@pytest.mark.parametrize('intrusion',['obstacle','hole'])
def test_surface_mask_ignores_aabb_corner_but_rejects_inside_rotating_envelope(intrusion):
    import cv2
    c=case();ds,frame,d,p=synthetic_scene(c)
    a=math.radians(40);rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    poly=np.array([[-75,-10],[75,-10],[75,10],[-75,10]])@rot.T+[-110,60]
    env=clearance_envelope_obb(poly,10,5,[0,0])
    uv=np.asarray(project_robot_points_mm(np.column_stack((env,np.zeros(4))),c['config'],frame.intrinsics))
    mask=np.zeros(frame.depth_m.shape,np.uint8);cv2.fillConvexPoly(mask,np.rint(uv).astype(np.int32),1)
    # Find a pixel at least three pixels from the true edge in an AABB corner.
    x0,y0=np.ceil(uv.min(axis=0)+3).astype(int);x1,y1=np.floor(uv.max(axis=0)-3).astype(int)
    outside=next((y,x) for y in range(y0,y1) for x in range(x0,x1)
        if cv2.pointPolygonTest(uv.astype(np.float32),(float(x),float(y)),True)<-4)
    obstacle=np.array([[outside[1]-1,outside[0]-1],[outside[1]+1,outside[0]-1],
                       [outside[1]+1,outside[0]+1],[outside[1]-1,outside[0]+1]],dtype=float)
    assert not convex_polygons_intersect(uv,obstacle)
    frame.depth_m[:]=.5;bad=.46 if intrusion=='obstacle' else 0.
    frame.depth_m[outside]=bad
    args=(env,frame,d,p,np.asarray(c['config']['calibration']['T_robot_from_camera']),0.,2.,c['config']['depth'])
    checked,_=_surface_clear(*args);assert checked==int(mask.sum())
    inside=tuple(np.rint(uv.mean(axis=0)[::-1]).astype(int));frame.depth_m[inside]=bad
    with pytest.raises(SceneGroundingError,match='destination_surface_not_clear' if intrusion=='obstacle' else 'destination_depth_unavailable'):
        _surface_clear(*args)


@pytest.mark.parametrize('intrusion',['obstacle','hole'])
def test_named_station_applies_rotated_depth_mask(tmp_path,intrusion):
    t={'target':'eraser','destination_type':'named','destination_name':'右盒子','reference':None,'relation':None}
    h=Harness(tmp_path,tasks=[t])
    try:
        _,run,obs=preview(h)
        selected={'target':obs.instances[1]['instance_id']}
        prepared=h.runtime._plan_placement(run,t,obs,selected)
        center=prepared['geometry']['destination_center_robot_xy_mm']
        uv=project_robot_points_mm([[*center,0.]],h.cfg,obs.payload['frame'].intrinsics)[0]
        obs.payload['frame'].depth_m[int(round(uv[1])),int(round(uv[0]))]=.46 if intrusion=='obstacle' else 0.
        with pytest.raises(SceneGroundingError,match='destination_surface_not_clear' if intrusion=='obstacle' else 'destination_depth_unavailable'):
            h.runtime._plan_placement(run,t,obs,selected)
        assert h.backends==[]
    finally:h.close()
