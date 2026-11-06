"""V2 HTTP -> semantic/pending -> RGB-D geometry -> direct controller (controlled).

All roots are temporary; no provider/device calls or writes to original assets.
"""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import json
import shutil
import threading
import time
import yaml
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from operator_console.direct_control_v2 import DirectRobotControllerV2
from operator_console.runtime_v2 import ConsoleRuntimeV2, DEMO_TEXT
from operator_console.server import create_server
from operator_console.simulation import synthetic_scene
from operator_console.task_parser_v2 import validate_task_plan_v2_candidate

ROOT = Path(__file__).resolve().parents[1]


def task(relation='right_of', target='eraser', reference='pen'):
    return {'target':target,'destination_type':'relative','destination_name':None,'reference':reference,'relation':relation}


def move_fixture(h, run, prepared):
    """Render the commanded object endpoint into the next controlled RGB-D frame."""
    import cv2
    import numpy as np
    from integration.relative_placement import project_robot_points_mm
    step = run['sequence'].snapshot()
    target_id = step['steps'][step['step_id']-1]['selected']['target']
    source = next(i for i in run['observation'].instances if i['instance_id'] == target_id)
    obj = h.case['objects'][source['index']]
    geometry = prepared['geometry']
    z = geometry.get('expected_top_z_mm')
    if z is None:
        z = source['xyz_mm'][2]
    points = np.column_stack((geometry['destination_footprint_robot_xy_mm'], [z]*4))
    ctx = run['observation'].payload
    pixels = project_robot_points_mm(points, ctx['config'], ctx['frame'].intrinsics)
    center, size, angle = cv2.minAreaRect(np.asarray(pixels, dtype=np.float32))
    if size[0] < size[1]:
        size = (size[1], size[0]);angle += 90
    obj.update(center_uv=list(center), size_px=list(size), angle_deg=angle)
    inv = np.linalg.inv(np.asarray(ctx['config']['calibration']['T_robot_from_camera']))
    obj['depth_m'] = float((inv @ np.array([*geometry['destination_center_robot_xy_mm'], z, 1000.]))[2]/1000.)


class Backend:
    def __init__(self):
        self.coords=[0,0,150,-180,0,0]; self.angles=[0]*6; self.calls=[]
    def solve_inv_kinematics(self,*a,**k): return None  # Controlled unavailable-IK fallback
    def send_angles(self,a,s): self.angles=list(a); self.calls.append(('angles',list(a)))
    def send_coords(self,p,s,m): self.coords=list(p); self.calls.append(('coords',list(p)))
    def get_angles(self): return self.angles
    def get_coords(self): return self.coords
    def set_gripper_state(self,s,v): self.calls.append(('gripper',s))
    def get_error_information(self): return 0


class Harness:
    def __init__(self, root, *, tasks=None, objects=None, rotation=90):
        self.root=root; self.calls=[]; self.order=[]; self.backends=[]
        self.case=json.loads((ROOT/'tests/data/relative_place_s3_demo.json').read_text(encoding='utf-8'))
        if objects is not None: self.case['objects']=deepcopy(objects)
        self.cfg=deepcopy(self.case['config'])
        self.cfg['scene']['workspace_view_rotation_deg']=rotation
        self.cfg['robot']={'start_angles_deg':[0,0,0,-90,0,-45],'start_pose_mm_deg':[0,0,150,-180,0,0],
            'coordinate_speed':15,'start_angle_speed':15,'approach_strategy':'linear','fallback_wait_s':0,
            'gripper':{'speed':80},'motion_poll_interval_s':1.5,'motion_timeout_s':10}
        for name in ['tools.json','world_entities.json','tests/data/relative_place_s3_demo.json']:
            dst=root/name; dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/name,dst)
        self.station={'profile_id':'station1','profile_key':'right','display_name':'右盒子','aliases':['收納區'],
            'version':1,'effective_status':'active','planner_mode':'cartesian',
            'pre_place_pose_mm_deg':[100,80,100,-180,0,0],'place_pose_mm_deg':[100,80,20,-180,0,0],
            'retreat_pose_mm_deg':[100,80,120,-180,0,0]}
        self.service=SimpleNamespace(root=root,runs_root=root/'runs',_base_config=lambda:deepcopy(self.cfg),
            _lock=threading.RLock(),_worker=None,_teach_robot=None,_active_robot=None,
            hardware_enabled=False,motion_enabled=False,confirmation_timeout=180,
            place_profiles=SimpleNamespace(status=lambda:{'profiles':[deepcopy(self.station)],'default_profile_id':'station1'}),
            table_surface=SimpleNamespace(status=lambda cfg:{'ready':True,'z_mm':0},require=lambda cfg:{'z_mm':0}))
        config_path=root/'config/v0.yaml';config_path.parent.mkdir(parents=True,exist_ok=True)
        config_path.write_text(yaml.safe_dump(self.cfg),encoding='utf-8');self.service.config=str(config_path)
        self.tasks=tasks or [task()]
        def parse(text,**options):
            self.calls.append((text,options));return validate_task_plan_v2_candidate({'tasks':deepcopy(self.tasks)},**options)
        def scene(step,cfg,mode):
            self.order.append(('capture',step));ds,f,d,p=synthetic_scene(self.case)
            return ds,f,d,p,deepcopy(self.cfg),deepcopy(self.case['surface'])
        def controller(cfg,settings,mode,sink,checkpoint):
            backend=Backend();self.backends.append(backend)
            from test_direct_control_v2 import Clock
            clock=Clock()
            ctrl=DirectRobotControllerV2(cfg,backend=backend,settings=settings,event_sink=sink,checkpoint=checkpoint,
                clock_fn=clock,sleep_fn=clock.sleep)
            initialize=ctrl.initialize_for_pick
            def init():
                self.order.append(('start',len([x for x in self.order if x[0]=='start'])+1));return initialize()
            ctrl.initialize_for_pick=init
            return ctrl
        self.runtime=ConsoleRuntimeV2(self.service,semantic_parser=parse,scene_factory=scene,controller_factory=controller)
        execute = self.runtime._execute
        def move_scene(run, prepared, **kwargs):
            result = execute(run, prepared, **kwargs)
            move_fixture(self, run, prepared)
            return result
        self.runtime._execute = move_scene
        self.server=create_server(self.service,port=0,v2_runtime=self.runtime)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url=f'http://127.0.0.1:{self.server.server_address[1]}'
        _,b,_=self.http('/api/v2/bootstrap');self.token=b['session']
    def http(self,path,payload=None,*,headers=None):
        hs={} if payload is None else {'Content-Type':'application/json','X-VisionPick-Token':getattr(self,'token','')}
        hs.update(headers or {})
        req=Request(self.url+path,headers=hs,data=None if payload is None else json.dumps(payload).encode())
        try: response=urlopen(req,timeout=5)
        except HTTPError as exc:response=exc
        with response:
            raw=response.read();content=response.headers.get('Content-Type','')
            return response.status,json.loads(raw) if content.startswith('application/json') else raw,response.headers
    def create(self):
        code,state,_=self.http('/api/v2/runs',{'text':'受控整份指令','semantic_source':'api','mode':'offline'})
        assert code==201;self.run_id=state['run_id'];return self.wait('waiting_confirmation','waiting_answer','blocked','failed')
    def wait(self,*statuses):
        deadline=time.monotonic()+max(10, 5*len(self.tasks)+5)
        while time.monotonic()<deadline:
            state=self.runtime.state(self.run_id)
            if state['status'] in statuses and not state['worker_busy']:return state
            time.sleep(.02)  # Avoid starving the worker with large long-plan snapshots.
        raise AssertionError(self.runtime.state(self.run_id))
    def action(self,name,payload=None):return self.http(f'/api/v2/runs/{self.run_id}/{name}',payload or {})
    def confirm(self):
        state=self.runtime.state(self.run_id)
        return self.action('confirm',{'version':state['pending']['version']})
    def close(self):
        self.runtime.shutdown();self.server.shutdown();self.server.server_close();self.thread.join(2)


@pytest.fixture
def factory(tmp_path):
    made=[]
    def build(**kw):
        h=Harness(tmp_path/str(len(made)),**kw);made.append(h);return h
    yield build
    for h in made:h.close()


def test_stage5_preview_scene_is_injected_without_another_capture_or_provider_call(factory):
    h=factory();preview=h.runtime.preview({'mode':'offline'});h.run_id=preview['run_id']
    h.wait('completed');assert h.calls==[]
    captured=list(h.order);s=h.create()
    assert h.calls[0][1]['visible_objects']==['pen','eraser']
    assert s['semantic_scene']['frame_id'] is not None and h.order==captured
    h.confirm();assert h.wait('completed','failed')['status']=='completed'
    final=h.runtime.state(h.run_id)['final_preview']['frame_id']
    assert h.runtime._visible_scene is None
    assert final!=s['semantic_scene']['frame_id']


@pytest.mark.parametrize('mismatch',['age','mode','configuration'])
def test_stage5_stale_or_different_scene_is_not_presented_as_current(factory,mismatch):
    from operator_console.runtime_v2 import _identity
    h=factory();scene={'objects':['battery'],'frame_id':'old','mode':'offline',
        'captured_at_unix_ms':time.time()*1000,'config_revision':_identity(h.cfg)}
    if mismatch=='age':scene['captured_at_unix_ms']-=60000
    elif mismatch=='mode':scene['mode']='real'
    else:scene['config_revision']='different'
    h.runtime._visible_scene=scene;h.create()
    assert h.calls[0][1]['visible_objects'] is None


def test_stage5_gap_reaches_native_geometry_and_tcp_destination(factory):
    h=factory(tasks=[task()|{'gap_mm':80.}]);h.create();h.confirm()
    s=h.wait('completed','failed');assert s['status']=='completed',s.get('error')
    p=next((h.root/'runs/v2'/h.run_id).glob('*.geometry.json'))
    g=json.loads(p.read_text(encoding='utf-8'))
    assert g['requested_gap_mm']==80. and g['planned_gap_mm']==85.
    assert any(c[0]=='coords' and c[1][:2]==g['candidate_tcp_poses_mm_deg']['place'][:2] for c in h.backends[0].calls)


@pytest.mark.parametrize('count',[1,2,3,4,5])
def test_http_full_loop_one_confirmation_start_before_every_capture(factory,count):
    h=factory(tasks=[task() for _ in range(count)]);s=h.create();assert h.confirm()[0]==200
    s=h.wait('completed','failed');assert s['status']=='completed',s
    assert h.order==[item for n in range(1,count+1) for item in [('start',n),('capture',n)]]+[('capture',count)]
    assert len(h.calls)==1 and len(s['sequence']['steps'])==count
    assert all(not row['result'][key] for row in s['sequence']['steps'] for key in ('outcome_verified','ik_validated','path_validated','collision_validated_3d'))
    assert len(list((h.root/'runs/v2'/h.run_id).glob('*.geometry.json')))==count
    assert h.confirm()[0]==400


def test_named_relative_near_mixed_three_tasks_share_one_loop_and_confirmation(factory):
    named=task()|{'destination_type':'named','destination_name':'右盒子','reference':None,'relation':None}
    h=factory(tasks=[task(),named,task('near')]);h.create();assert h.confirm()[0]==200
    s=h.wait('completed','failed');assert s['status']=='completed',s
    assert len(h.calls)==1 and h.order==[item for n in range(1,4) for item in [('start',n),('capture',n)]]+[('capture',3)]
    assert sum(e['event']=='plan_confirmed' for e in s['pending']['history'])==1


@pytest.mark.parametrize('relation',['left_of','right_of','in_front_of','behind','above','below','near','on_top_of'])
def test_native_geometry_relation_consumed_without_legacy_ik(factory,relation):
    h=factory(tasks=[task(relation)]);h.create();h.confirm();s=h.wait('completed','failed')
    assert s['status']=='completed',s
    assert any(c[0]=='coords' for c in h.backends[0].calls)


def test_second_task_missing_blocks_all_initialization_then_local_answer_no_api(factory):
    h=factory(tasks=[task(),task()|{'target':None}]);s=h.create();assert s['status']=='waiting_answer'
    assert h.confirm()[0]==400 and h.order==[]
    q=s['pending']['question']
    assert h.action('answer',{'value':'橡皮擦','version':q['version'],'task_index':q['task_index'],'slot':q['slot']})[0]==200
    assert len(h.calls)==1;h.confirm();assert h.wait('completed','failed')['status']=='completed'


def test_over_twenty_tasks_entire_plan_blocked_zero_sensor_and_commands(factory):
    h=factory(tasks=[task()]*21);assert h.create()['status']=='blocked'
    assert h.confirm()[0]==400 and h.order==[] and h.backends==[]


def duplicates(h):
    objs=deepcopy(h.case['objects']);extra=deepcopy(objs[1]);extra['center_uv']=[160,340];objs.append(extra);return objs


def test_manual_same_class_selection_has_bound_frame_and_preserves_all_objects(factory):
    h=factory();h.case['objects']=duplicates(h);h.create();h.confirm();s=h.wait('waiting_instance')
    q=s['sequence']['question'];assert q['role']=='target' and len(q['choices'])==2
    fields={k:q[k] for k in ('run_id','step_id','frame_id','plan_version','role')}
    assert h.action('select',fields|{'instance_id':q['choices'][0]['instance_id'],'frame_id':'old'})[0]==400
    assert not any(c[0]=='coords' for c in h.backends[0].calls)
    assert h.action('select',fields|{'instance_id':q['choices'][0]['instance_id']})[0]==200
    assert h.wait('completed','failed')['status']=='completed'
    geometry=json.loads(next((h.root/'runs/v2'/h.run_id).glob('*.geometry.json')).read_text())
    assert geometry['target_index']==1


def test_target_reference_same_class_stays_two_distinct_roles(factory):
    h=factory(tasks=[task('right_of','eraser','eraser')]);h.case['objects']=duplicates(h);h.create();h.confirm()
    s=h.wait('waiting_instance');q=s['sequence']['question'];assert q['role']=='target'
    fields={k:q[k] for k in ('run_id','step_id','frame_id','plan_version','role')}
    chosen=q['choices'][0]['instance_id'];h.action('select',fields|{'instance_id':chosen})
    s=h.wait('waiting_instance');q=s['sequence']['question'];assert q['role']=='reference'
    assert next(c for c in q['choices'] if c['instance_id']==chosen)['eligible'] is False
    h.action('cancel');assert h.wait('cancelled')['sequence']['steps'][0]['status']=='pending'


def test_refresh_rejects_old_frame_after_reverifying_start(factory):
    h=factory();h.case['objects']=duplicates(h);h.create();h.confirm();q=h.wait('waiting_instance')['sequence']['question']
    assert h.action('refresh')[0]==200;s=h.wait('waiting_instance')
    assert s['sequence']['question']['frame_id']!=q['frame_id']
    assert h.order==[('start',1),('capture',1),('start',2),('capture',1)]
    fields={k:q[k] for k in ('run_id','step_id','frame_id','plan_version','role')}
    assert h.action('select',fields|{'instance_id':q['choices'][0]['instance_id']})[0]==400


def test_mutation_token_origin_protection_and_legacy_page_retired(factory):
    h=factory()
    assert h.http('/api/v2/runs',{},headers={'X-VisionPick-Token':'wrong'})[0]==403
    assert h.http('/api/v2/settings',{},headers={'Origin':'https://external.invalid'})[0]==403
    code,html,_=h.http('/');assert code==200 and html==(ROOT/'src/operator_console/static/v2/index.html').read_bytes()
    assert h.http('/legacy/')[0] == 404
    code,html,headers=h.http('/v2/');assert code==200 and b'v2/app.js' in html and "script-src 'self'" in headers['Content-Security-Policy']
    assert h.http('/v2/dashboard.css')[0]==200
    assert h.http('/v2/app.js')[0]==200


def test_settings_restart_and_changed_config_requires_new_plan(factory):
    h=factory();_,s,_=h.http('/api/v2/settings');assert h.http('/api/v2/settings',{'overrides':{'motion_timeout_s':2},'expected_revision':s['revision']})[0]==200
    again=ConsoleRuntimeV2(h.service);assert again.settings.read()['values']['motion_timeout_s']==2
    h.create();h.cfg['grasp']['grasp_z_offset_mm']+=1
    assert h.confirm()[0]==400 and h.order==[]


def test_unknown_explicit_station_never_falls_back_or_moves(factory):
    named=task()|{'destination_type':'named','destination_name':'未知站','reference':None,'relation':None}
    h=factory(tasks=[named]);s=h.create();assert s['status']=='waiting_answer' and h.order==[]
    q=s['pending']['question'];assert q['slot']=='destination_name'
    assert h.action('answer',{'value':'收納區','version':q['version'],'task_index':1,'slot':'destination_name'})[0]==200
    h.confirm();assert h.wait('completed','failed')['status']=='completed'
    assert h.calls[0][1]['stations']==[{'name':'右盒子','aliases':['收納區']}]


def test_named_occupied_destination_no_pick_after_initialization(factory):
    named=task()|{'destination_type':'named','destination_name':'右盒子','reference':None,'relation':None}
    h=factory(tasks=[named]);h.station['place_pose_mm_deg'][:2]=[0,0];h.create();h.confirm()
    s=h.wait('failed');assert s['sequence']['error']['reason']=='named_landing_occupied'
    assert not any(c[0]=='coords' for c in h.backends[0].calls)


@pytest.mark.parametrize('source',['robot_teaching','robot_pose_read','visual_manual_selection','manual_entry'])
@pytest.mark.parametrize('defect',['depth_hole','occupied'])
def test_named_station_source_controls_identical_preview_and_execution_checks(factory,source,defect):
    named=task()|{'destination_type':'named','destination_name':'右盒子','reference':None,'relation':None}
    h=factory(tasks=[named]);h.station.update(placement_source={'type':source},clear_path_acknowledged=True)
    if defect=='occupied':
        for key in ('place_pose_mm_deg','pre_place_pose_mm_deg','retreat_pose_mm_deg'):h.station[key][:2]=[0,0]
    else:
        scene=h.runtime.scene_factory
        def holes(*args):
            result=scene(*args);result[1].depth_m[90:210,380:510]=0
            return result
        h.runtime.scene_factory=holes
    h.create();run=h.runtime._runs[h.run_id];h.runtime._observe(run,1);h.runtime._refresh_placement_previews(run)
    overlay=h.runtime.state(h.run_id)['placement_previews'][0]
    taught=source in {'robot_teaching','robot_pose_read'}
    assert overlay['feasible'] is taught,overlay
    expected='destination_depth_unavailable' if defect=='depth_hole' else 'named_landing_occupied'
    if not taught:assert overlay['reason']==expected
    assert h.confirm()[0]==200
    state=h.wait('completed','failed')
    assert state['status']==('completed' if taught and defect!='depth_hole' else 'failed'),state['sequence']
    if taught:
        path=next((h.root/'runs/v2'/h.run_id).glob('*.geometry.json'))
        geometry=json.loads(path.read_text(encoding='utf-8'))
        assert geometry['destination_check_policy']=='operator_taught'
        assert not geometry['destination_collision_checked'] and not geometry['destination_surface_checked']
        assert geometry['source_occlusion_checked'] and geometry['checked_surface_pixels']==0
        assert geometry['local_surface_z_mm'] is None
        assert any(call==('gripper',1) for call in h.backends[0].calls)
    else:
        assert state['sequence']['error']['reason']==expected
        assert not any(call[0]=='coords' or call==('gripper',1) for call in h.backends[0].calls)


@pytest.mark.parametrize('source,ack',[('robot_teaching',False),('robot_pose_read',None),('visual_manual_selection',True),(None,True),('unknown',True)])
def test_only_confirmed_robot_taught_sources_bypass_named_collision_checks(factory,source,ack):
    from operator_console.place_catalog_v2 import PlaceCatalogV2
    h=factory();h.station.update(clear_path_acknowledged=ack)
    if source is not None:h.station['placement_source']={'type':source}
    place=PlaceCatalogV2(h.service.place_profiles.status()).direct_place('右盒子')
    assert place.destination_check_policy=='rgbd_envelope'


@pytest.mark.parametrize('source',['robot_teaching','visual_manual_selection'])
@pytest.mark.parametrize('axis,value',[(0,400.),(2,400.)])
def test_named_station_both_methods_keep_tcp_workspace_limits(factory,source,axis,value):
    named=task()|{'destination_type':'named','destination_name':'右盒子','reference':None,'relation':None}
    h=factory(tasks=[named]);h.station.update(placement_source={'type':source},clear_path_acknowledged=True)
    h.station['place_pose_mm_deg'][axis]=value
    h.create();h.confirm();state=h.wait('failed')
    assert state['sequence']['error']['reason']=='named_tcp_outside_workspace'
    assert not any(call[0]=='coords' or call==('gripper',1) for call in h.backends[0].calls)


@pytest.mark.parametrize('source',['robot_teaching','robot_pose_read','visual_manual_selection'])
def test_named_station_keeps_source_occlusion_check(factory,source):
    named=task()|{'destination_type':'named','destination_name':'右盒子','reference':None,'relation':None}
    h=factory(tasks=[named]);h.station.update(placement_source={'type':source},clear_path_acknowledged=True)
    h.case['objects'][1]['center_uv']=[320.,240.]
    h.case['objects'][0]['depth_m']=.45
    h.create();h.confirm();state=h.wait('failed')
    assert state['sequence']['error']['reason']=='target_occluded_by_stacked_object'
    assert not any(call[0]=='coords' or call==('gripper',1) for call in h.backends[0].calls)


@pytest.mark.parametrize('a,b',[('pen','battery'),('battery','pen')])
def test_swap_avoids_its_original_site_and_completes_three_moves(factory,a,b):
    import math
    import numpy as np
    objects=[{'class_name':name,'center_uv':[320.,330.] if name==a else [320.,240.],
        'size_px':[120.,12.] if name=='pen' else [48.,30.],
        'angle_deg':20. if name=='pen' else 0.,'confidence':.95} for name in (a,b)]
    tasks=[task('near',target=a,reference=b),task('at_initial',target=b,reference=a),task('at_initial',target=a,reference=b)]
    h=factory(tasks=tasks,objects=objects)
    h.cfg['orientation'].update(place_orientation_policy_relative='align_reference',place_orientation_policy_swap='match_initial')
    original={o['class_name']:list(o['center_uv']) for o in h.case['objects']}
    execute=h.runtime._execute;moves=[]
    def move_scene(run,prepared,**kwargs):
        result=execute(run,prepared,**kwargs)
        g=prepared['geometry'];step=run['sequence'].snapshot()['step_id'];x,y=g['destination_center_robot_xy_mm']
        moves.append((step,x,y,g.get('swap_reserved_regions',[])))
        for obj in h.case['objects']:
            if obj['class_name']==tasks[step-1]['target']:
                obj['center_uv']=[320+1.2*x,240-1.2*y]
                poly=np.asarray(g['destination_footprint_robot_xy_mm']);edges=np.roll(poly,-1,axis=0)-poly
                edge=edges[np.argmax(np.linalg.norm(edges,axis=1))]
                obj['angle_deg']=math.degrees(math.atan2(-edge[1],edge[0]))
        return result
    h.runtime._execute=move_scene
    h.create();run=h.runtime._runs[h.run_id]
    h.runtime._observe(run,1);h.runtime._refresh_placement_previews(run)
    state=h.runtime.state(h.run_id)
    assert len(state['placement_previews'])==3 and all(p['feasible'] for p in state['placement_previews'])
    predicted=state['placement_previews'][0]['center_mm'][:2]
    assert h.confirm()[0]==200
    state=h.wait('completed','failed','waiting_step_recovery')
    assert state['status']=='completed',state['sequence']
    assert len(moves)==3 and len(moves[0][3])==6 and moves[0][1:3]==pytest.approx(predicted)
    final={o['class_name']:o['center_uv'] for o in h.case['objects']}
    assert final[a]==pytest.approx(original[b],abs=1e-4) and final[b]==pytest.approx(original[a],abs=1e-4)


def test_local_outcome_separate_from_control_completion(factory):
    h=factory();h.create();h.confirm();h.wait('completed')
    assert h.action('outcome',{'outcome':'success'})[0]==200
    s=h.runtime.state(h.run_id);assert not s['outcome_verified'] and s['manual_outcome']['outcome']=='success'


def test_real_mode_disabled_keeps_semantics_and_blocks_motion(factory):
    h=factory();code,s,_=h.http('/api/v2/runs',{'text':'真實指令','semantic_source':'api','mode':'real'})
    assert code==201;h.run_id=s['run_id'];h.wait('waiting_confirmation')
    assert h.confirm()[1]['error']=='hardware_not_detected'
    assert len(h.calls)==1 and h.order==h.backends==[]


def test_cancelled_plan_restart_is_read_only_never_reexecutes(factory):
    h=factory();h.create();h.action('cancel');assert h.wait('cancelled')['status']=='cancelled'
    again=ConsoleRuntimeV2(h.service);assert again.state(h.run_id)['read_only']
    with pytest.raises(ValueError,match='not_resumable'):again.action(h.run_id,'confirm',{'version':1})
    assert h.order==[]


def test_offline_demo_no_provider_and_no_original_data_writes(factory):
    h=factory();code,s,_=h.http('/api/v2/runs',{'text':DEMO_TEXT,'mode':'offline','semantic_source':'demo'})
    assert code==201;h.run_id=s['run_id'];h.wait('waiting_confirmation');h.confirm();assert h.wait('completed','failed')['status']=='completed'
    assert h.calls==[] and not h.runtime.holding_path.exists()


def test_failure_stops_later_steps_and_retains_holding_uncertainty(factory):
    h=factory(tasks=[task(),task()]);original=h.runtime.controller_factory;controllers=[]
    def factory_controller(*args):
        ctrl=original(*args);controllers.append(ctrl);send=ctrl.mc.send_coords
        def fail_after_close(*a):
            if ctrl.gripper_holding_uncertain:raise RuntimeError('controlled transfer failure')
            return send(*a)
        ctrl.mc.send_coords=fail_after_close;return ctrl
    h.runtime.controller_factory=factory_controller;h.create();h.confirm();s=h.wait('failed')
    assert h.order==[('start',1),('capture',1)]
    assert s['sequence']['steps'][1]['status']=='pending'
    assert h.backends[0].calls[-1]==('gripper',1) and controllers[0].gripper_holding_uncertain


def native_real(h, monkeypatch):
    import v0_mvp.camera as camera_module
    import v0_mvp.detector as detector_module
    import operator_console.runtime_v2 as runtime_module
    import numpy as np
    h.service.hardware_enabled=h.service.motion_enabled=True
    h.runtime.acknowledge_empty({'confirm':'EMPTY_GRIPPER'})
    h.service.device_status=lambda:{'camera':{'detected':True,'count':1},'robot':{'detected':True}}
    h.cfg['calibration'].update(ready_for_motion=True,rmse_mm=1,max_rmse_mm=15)
    h.cfg['robot']['port']='controlled-not-a-physical-port'
    h.cfg['camera']={'warmup_frames':2};h.cfg['model']['weights']='controlled.pt'
    h.runtime.scene_factory=h.runtime.controller_factory=None
    native_order=[]
    class Camera:
        def __init__(self,*a):self.step=0;self.scene=None
        def connect(self):native_order.append('camera_connected')
        def warmup(self,n):native_order.append(('flush',n))
        def capture(self):
            self.step+=1;native_order.append(('capture',self.step))
            case=deepcopy(h.case)
            if self.step>1:case['objects'][1]['depth_m']=.49
            self.scene=synthetic_scene(case);return self.scene[1]
        def deproject(self,u,v,z,intr):return np.array([(u-intr.ppx)*z/intr.fx,(v-intr.ppy)*z/intr.fy,z])
        def disconnect(self):native_order.append('camera_closed')
    cameras=[]
    def camera_factory(*a):c=Camera();cameras.append(c);return c
    class Detector:
        def __init__(self,*a,**k):pass
        def detect(self,image):return cameras[-1].scene[0],None
    class Lease:
        def __init__(self,port):pass
        def acquire(self):native_order.append('lease_acquired');return self
        def release(self):native_order.append('lease_released')
    def connect(ctrl):
        # Native routing still uses a fake backend: fixed physical waits must
        # advance the same synthetic clock as controller telemetry.
        from test_direct_control_v2 import Clock
        clock=Clock();ctrl._clock=ctrl._feedback_clock=ctrl._motion_clock=clock;ctrl.sleep=clock.sleep
        ctrl.mc=Backend();h.backends.append(ctrl.mc)
        send=ctrl.mc.send_angles
        def start(*a):native_order.append('start_command');return send(*a)
        ctrl.mc.send_angles=start
    monkeypatch.setattr(camera_module,'RealSenseCamera',camera_factory)
    monkeypatch.setattr(detector_module,'YOLO11OBBDetector',Detector)
    monkeypatch.setattr(runtime_module,'RobotLease',Lease)
    monkeypatch.setattr(DirectRobotControllerV2,'connect',connect)
    return native_order


def create_real(h):
    code,s,_=h.http('/api/v2/runs',{'text':'受控原生路由','semantic_source':'api','mode':'real'})
    assert code==201;h.run_id=s['run_id'];return h.wait('waiting_confirmation','waiting_answer','failed')


def test_native_camera_detector_route_flushes_after_start_and_remeasures_height(factory,monkeypatch):
    h=factory(tasks=[task(),task()]);order=native_real(h,monkeypatch);create_real(h);assert h.confirm()[0]==200
    s=h.wait('completed','failed');assert s['status']=='completed',s
    assert [x for x in order if x=='start_command' or isinstance(x,tuple)]==[
        'start_command',('flush',2),('capture',1),  # Explicit controlled empty-hand acknowledgement.
        'start_command',('flush',2),('capture',1),'start_command',('flush',2),('capture',2),'start_command',('flush',2),('capture',3)]
    sources=[json.loads(p.read_text())['source_grasp_plan']['grasp_pose_mm_deg'][2] for p in
        sorted((h.root/'runs/v2'/h.run_id).glob('*.geometry.json'),key=lambda p:p.stat().st_mtime_ns)]
    assert sources==pytest.approx([10,20],abs=.001)
    assert order[-2:]==['camera_closed','lease_released']
    assert json.loads(h.runtime.holding_path.read_text())['holding_uncertain'] is False


@pytest.mark.parametrize('fault',['start_angles','calibration','persisted_holding'])
def test_native_preflight_failure_has_json_error_and_zero_robot_connections(factory,monkeypatch,fault):
    h=factory();order=native_real(h,monkeypatch)
    if fault=='start_angles':h.cfg['robot']['start_angles_deg']=None
    if fault=='calibration':h.cfg['calibration']['ready_for_motion']=False
    if fault=='persisted_holding':
        h.runtime.holding_path.parent.mkdir(parents=True,exist_ok=True)
        h.runtime.holding_path.write_text(json.dumps({'holding_uncertain':True}))
    result=create_real(h)
    assert result['status']=='failed' and result['error'] and h.backends==[]
    assert 'lease_acquired' not in order and 'start_command' not in order


def test_native_waiting_selection_blocks_legacy_mutations_and_cancel_releases(factory,monkeypatch):
    h=factory();h.case['objects']=duplicates(h);order=native_real(h,monkeypatch);create_real(h);h.confirm();h.wait('waiting_instance')
    assert h.http('/api/preflight',{})[0]==404 and order.count('lease_acquired')-order.count('lease_released')==1
    h.action('cancel');h.wait('cancelled');assert 'lease_released' in order


def test_persisted_uncertain_gripper_needs_explicit_idle_operator_recovery(factory):
    h=factory();h.runtime.holding_path.parent.mkdir(parents=True,exist_ok=True)
    h.runtime.holding_path.write_text(json.dumps({'holding_uncertain':True}))
    assert h.http('/api/v2/recover-empty',{'confirm':'wrong'})[0]==400
    assert h.http('/api/v2/bootstrap')[1]['holding_uncertain'] is True
    assert h.http('/api/v2/recover-empty',{'confirm':'EMPTY_GRIPPER'})[0]==200
    assert ConsoleRuntimeV2(h.service)._holding() is False


def test_selection_during_worker_final_save_does_not_consume_choice(factory):
    h=factory();h.case['objects']=duplicates(h);h.create();h.confirm();s=h.wait('waiting_instance')
    q=s['sequence']['question'];real_thread=h.runtime._thread
    h.runtime._thread=SimpleNamespace(is_alive=lambda:True)
    try:
        fields={k:q[k] for k in ('run_id','step_id','frame_id','plan_version','role')}
        assert h.action('select',fields|{'instance_id':q['choices'][0]['instance_id']})[0]==400
        assert h.runtime.state(h.run_id)['sequence']['question']==q
        assert h.runtime.state(h.run_id)['sequence']['selected']=={}
    finally:h.runtime._thread=real_thread


def test_shutdown_rejects_new_plan_and_never_reexecutes(factory):
    h=factory();h.create();h.runtime.shutdown()
    assert h.runtime.state(h.run_id)['status']=='cancelled'
    assert h.http('/api/v2/runs',{'text':DEMO_TEXT,'semantic_source':'demo','mode':'offline'})[0]==400
    assert h.order==[]


def test_supplied_zero_frame_timestamp_is_not_replaced_by_fresh_clock(factory):
    h=factory();scene=h.runtime.scene_factory
    def stale(*args):
        result=list(scene(*args));frame=result[1]
        result[1]=SimpleNamespace(color_bgr=frame.color_bgr,depth_m=frame.depth_m,
            intrinsics=frame.intrinsics,captured_at_unix_ms=0)
        return result
    h.runtime.scene_factory=stale;h.create();h.confirm();s=h.wait('failed')
    assert s['sequence']['error']['reason']=='scene_snapshot_stale'
    assert not any(c[0]=='coords' for c in h.backends[0].calls)


def browser_check(h, *, width=1280, use_button=False, overlap=False):
    import subprocess
    import os
    packages=Path.home()/'.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules'
    if not (packages/'playwright').is_dir():
        pytest.skip('Browser integration requires the bundled Playwright package')
    script=r'''
const {chromium}=require('playwright');
const fs=require('fs');const assert=require('assert');const input=JSON.parse(process.argv[2]);
(async()=>{
 const browser=await chromium.launch({headless:true,channel:'msedge'});
 try{
  const page=await browser.newPage({viewport:{width:input.width,height:1000}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  page.on('console',m=>{if(m.type()==='error')errors.push(m.text()+' '+m.location().url);});
  await page.goto(input.url+'/v2/');await page.waitForLoadState('networkidle');
  assert.strictEqual(await page.locator('#settings-fields input').count(),6);
   await page.click('#engineering-open');await page.click('#tab-motion');await page.locator('#environment-details summary').click();await page.selectOption('#semantic-source','api');await page.click('#engineering-close');await page.fill('#instruction','受控整份指令');
  await page.click('#parse');await page.waitForFunction(()=>!document.querySelector('#confirm').disabled);
  await page.click('#confirm');await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('請選擇當步物件'));
  await page.waitForFunction(()=>!document.querySelector('#scene').hidden);
  const b=await (await page.request.get(input.url+'/api/v2/bootstrap')).json();
  const state=await (await page.request.get(input.url+'/api/v2/runs/'+b.active_run)).json();
  const q=state.sequence.question,d=state.sequence.display;assert.strictEqual(q.role,'target');
  const expected=q.choices[0].instance_id;
  await page.locator('#scene').scrollIntoViewIfNeeded();
  const box=await page.locator('#scene').boundingBox();
  const r=d.rotation_deg,raw=[450,340];
  const rotated=r===0?raw:r===90?[raw[1],d.width-raw[0]]:r===180?[d.width-raw[0],d.height-raw[1]]:[d.height-raw[1],raw[0]];
  const w=r===90||r===270?d.height:d.width,h=r===90||r===270?d.width:d.height;
  if(!input.use_button||input.overlap){
   await page.locator('#scene').click({position:{x:rotated[0]*box.width/w,y:rotated[1]*box.height/h}});
   if(input.overlap){await page.waitForFunction(()=>document.querySelector('#error').textContent.includes('框重疊'));}
  }
  if(input.use_button||input.overlap)await page.locator('#choices button').first().click();
  await page.waitForFunction(()=>document.querySelector('#status').textContent.includes('動作完成'),{},{timeout:10000});
  const finished=await (await page.request.get(input.url+'/api/v2/runs/'+b.active_run)).json();
  assert.strictEqual(finished.sequence.steps[0].selected.target,expected);
  assert.strictEqual(finished.outcome_verified,false);assert.strictEqual(finished.events.filter(e=>e.event==='object_released_at_place').length,1);
  await page.screenshot({path:input.output,fullPage:true});
  assert.deepStrictEqual(errors,[]);
  console.log(JSON.stringify({browser:'Edge headless / Playwright',rotation:r,width:input.width,instance:expected,console_errors:errors.length}));
 }finally{await browser.close();}
})().catch(e=>{console.error(e.stack);process.exitCode=1;});
'''
    code=h.root/'browser_check.cjs';code.write_text(script,encoding='utf-8')
    result=subprocess.run(['node',str(code),json.dumps({'url':h.url,'width':width,'use_button':use_button,
        'overlap':overlap,'output':str(h.root/'browser.png')})],capture_output=True,text=True,encoding='utf-8',timeout=45,
        env={**os.environ,'NODE_PATH':str(packages)})
    assert result.returncode==0,result.stdout+'\n'+result.stderr
    (h.root/'browser_result.json').write_text(result.stdout,encoding='utf-8')


@pytest.mark.parametrize('rotation',[0,90,180,270])
@pytest.mark.parametrize('width',[900,1280])
def test_browser_rotated_scaled_canvas_selects_exact_instance(factory,rotation,width):
    h=factory(rotation=rotation);h.case['objects']=duplicates(h);browser_check(h,width=width)
    assert len(h.calls)==1 and h.order==[('start',1),('capture',1),('capture',1)]


def test_v2_software_stop_latches_cancel_before_transport_and_keeps_gripper(factory):
    h=factory();h.case['objects']=duplicates(h);h.create();h.confirm();h.wait('waiting_instance')
    backend=h.backends[0]
    backend.stop=lambda:backend.calls.append(('stop',))
    before=list(backend.calls)
    assert h.action('stop')[0]==200
    state=h.wait('cancelled')
    assert backend.calls==before+[('stop',)]
    assert any(e['event']=='software_stop_result' and e['sent'] for e in state['events'])
    assert not h.runtime.owns_resources()


def test_connection_status_never_labels_controlled_backend_as_physical(factory):
    h=factory();h.create();h.confirm();s=h.wait('completed')
    assert s['connections']=={'robot':'simulated','camera':'simulated'}
    run=h.runtime._runs[h.run_id];run['mode']='real';run['cleanup_error']=['camera:OSError']
    assert h.runtime.state(h.run_id)['connections']=={'robot':'unknown','camera':'unknown'}


def test_preview_verifies_start_before_capture_and_publishes_measured_detection_data(factory):
    h=factory();code,state,_=h.http('/api/v2/preview',{'mode':'offline'})
    assert code==201;h.run_id=state['run_id'];state=h.wait('completed')
    assert h.calls==[] and h.order==[('start',1),('capture',1)]
    assert len(state['preview']['instances'])==2
    assert all(len(i['xyz_mm'])==3 and 0<=i['confidence']<=1 for i in state['preview']['instances'])
    assert not any(c[0]=='coords' for c in h.backends[0].calls)
    assert not h.runtime.owns_resources()


def test_preview_cannot_replace_a_plan_waiting_for_confirmation(factory):
    h=factory();h.create()
    assert h.http('/api/v2/preview',{'mode':'offline'})[0]==400
    assert h.order==[] and h.calls


def test_bootstrap_labels_include_world_reference_objects_without_grasp_admission(factory):
    h=factory();_,boot,_=h.http('/api/v2/bootstrap')
    assert boot['object_labels']['remote_car']=='遙控車'
    assert boot['object_labels']['eraser']=='橡皮擦'
    assert 'remote_car' not in __import__('object_catalog').graspable_classes()


def test_browser_number_button_selects_exact_instance(factory):
    h=factory();h.case['objects']=duplicates(h);browser_check(h,use_button=True)


def test_browser_overlapping_frames_require_number_choice(factory):
    h=factory();h.case['objects']=duplicates(h);h.case['objects'][-1]['center_uv']=[450,340]
    browser_check(h,overlap=True)


if __name__=='__main__':
    import sys
    root=Path(sys.argv[1]);root.mkdir(parents=True,exist_ok=True)
    h=Harness(root)
    print(h.url,flush=True)
    try:
        while True:time.sleep(1)
    finally:h.close()



def test_native_parse_publishes_geometry_preview_before_confirmation(factory,monkeypatch):
    h=factory();native_real(h,monkeypatch);s=create_real(h)
    assert s['status']=='waiting_confirmation'
    assert all(i['xyz_mm'] is not None for i in s['preview']['instances'])
    p=s['placement_previews'][0]
    assert p['feasible'] and len(p['polygon_px'])==len(p['envelope_px'])==4
    assert p['authorizes_motion'] is False and len(h.calls)==1
    assert not any(e['event']=='object_released_at_place' for e in s['events'])


def test_completed_run_captures_new_scene_once_after_existing_return(factory):
    h=factory();h.create();h.confirm();s=h.wait('completed','failed')
    assert s['status']=='completed' and s['completion_capture']['status']=='captured'
    final=s['final_preview']['frame_id'];last=s['sequence']['steps'][-1]['frame_id']
    assert final!=last and s['preview']['frame_id']==final
    assert s['sequence']['display']['frame_id']==last
    assert h.runtime._visible_scene is None
    assert h.order==[('start',1),('capture',1),('capture',1)]
    assert sum(e['event']=='empty_hand_returned_to_start' for e in s['events'])==1
    run=h.runtime._runs[h.run_id];before=list(h.order)
    h.runtime._capture_completion(run,s['sequence']);assert h.order==before
    saved=json.loads((h.root/'runs/v2'/h.run_id/'state.json').read_text(encoding='utf-8'))
    assert saved['final_preview']['frame_id']==final


def test_completion_capture_failure_retains_last_frame_without_repeating_motion(factory):
    h=factory();h.create();capture=h.runtime._observe
    def observe(run,step,**kwargs):
        if kwargs.get('semantic_only'):raise RuntimeError('controlled_final_capture_failure')
        return capture(run,step,**kwargs)
    h.runtime._observe=observe;h.confirm();s=h.wait('completed','failed')
    assert s['status']=='failed' and s['pending']['status']=='failed' and s['final_preview'] is None
    assert s['completion_capture']['status']=='failed'
    assert s['completion_capture']['reason']=='controlled_final_capture_failure'
    assert s['preview']['frame_id']==s['sequence']['display']['frame_id']==s['sequence']['steps'][0]['frame_id']
    assert h.order==[('start',1),('capture',1)]
    assert sum(e['event']=='empty_hand_returned_to_start' for e in s['events'])==1
    assert not s['outcome_verified'] and not s['sequence']['outcome_verified']



def test_stop_during_completion_settle_prevents_new_capture(factory):
    h=factory();h.create();execute=h.runtime._execute
    def stop_after_return(run,prepared,**kwargs):
        result=execute(run,prepared,**kwargs)
        run['controller'].sleep=lambda duration:run.update(cancelled=True)
        return result
    h.runtime._execute=stop_after_return;h.confirm();s=h.wait('completed','cancelled','failed')
    assert s['completion_capture']['status']=='cancelled' and s['final_preview'] is None
    assert h.order==[('start',1),('capture',1)]


@pytest.mark.parametrize('count',[1,2])
@pytest.mark.parametrize('second_succeeds',[False,True])
def test_partial_self_move_retries_once_at_frozen_endpoint_and_reuses_captures(factory,count,second_succeeds):
    move=task()|{'destination_type':'self_relative','reference':None,'displacement_mm':40}
    h=factory(tasks=[move]+[task() for _ in range(count-1)])
    execute=h.runtime._execute;attempts=[]
    def partial(run,prepared,**kwargs):
        step=run['sequence'].snapshot()['step_id']
        before=list(h.case['objects'][1]['center_uv'])
        result=execute(run,prepared,**kwargs)
        if step==1:
            attempts.append(deepcopy(prepared['geometry']))
            if len(attempts)==1:
                h.case['objects'][1]['center_uv']=[a+.375*(b-a) for a,b in zip(before,h.case['objects'][1]['center_uv'])]
            elif not second_succeeds:
                h.case['objects'][1]['center_uv']=before
        return result
    h.runtime._execute=partial;h.create();h.confirm();s=h.wait('completed','failed')
    assert s['status']==('completed' if second_succeeds else 'failed'),s['sequence']
    assert len(attempts)==2 and attempts[0]['destination_center_robot_xy_mm']==attempts[1]['destination_center_robot_xy_mm']
    assert attempts[0]['source_grasp_plan']!=attempts[1]['source_grasp_plan']
    first=s['sequence']['steps'][0]
    assert first['retry_count']==1 and first['local_result_check']['passed'] is second_succeeds
    assert len(h.calls)==1 and not s['outcome_verified']
    assert len([x for x in h.order if x[0]=='capture'])==(3 if count==1 else 4 if second_succeeds else 3)
    if second_succeeds:
        assert s['final_preview']['frame_id']==s['completion_capture']['frame_id']
        assert s['pending']['status']=='completed'


def test_same_class_retry_keeps_bystander_and_does_not_ask_for_selection_again(factory):
    move=task()|{'destination_type':'self_relative','reference':None,'displacement_mm':40}
    h=factory(tasks=[move]);h.case['objects']=duplicates(h)
    bystander=deepcopy(h.case['objects'][2]);execute=h.runtime._execute;attempts=[]
    def miss_once(run,prepared,**kwargs):
        before=list(h.case['objects'][1]['center_uv']);attempts.append(prepared['frame_id'])
        result=execute(run,prepared,**kwargs)
        if len(attempts)==1:h.case['objects'][1]['center_uv']=before
        return result
    h.runtime._execute=miss_once;h.create();h.confirm();s=h.wait('waiting_instance')
    q=s['sequence']['question'];chosen=next(i for i in q['choices'] if i['instance_id'].endswith(':1'))
    h.action('select',{k:q[k] for k in ('run_id','step_id','frame_id','plan_version','role')}|{'instance_id':chosen['instance_id']})
    s=h.wait('completed','failed','waiting_instance')
    assert s['status']=='completed',s['sequence']
    assert len(attempts)==2 and attempts[0]!=attempts[1]
    assert h.case['objects'][2]==bystander
    assert sum(e['event']=='instance_selected' for e in s['sequence']['history'])==1


def fail_motion_once(h, stage, *, error_code=0, fail_recovery=False):
    factory=h.runtime.controller_factory;failures=[];active=[]
    execute=h.runtime._execute
    def executing(run,prepared,**kwargs):
        active[:]=[run,prepared]
        return execute(run,prepared,**kwargs)
    h.runtime._execute=executing
    def controller(*args):
        ctrl=factory(*args);move=ctrl._move
        def fail(kind,target,**kwargs):
            label=kwargs['label']
            trigger=(not failures and label.startswith(stage) or
                     fail_recovery and failures and label=='initialization_before_capture')
            if not trigger:return move(kind,target,**kwargs)
            if label.startswith('place_retreat'):
                move_fixture(h,*active)  # Release already occurred before the failing retreat.
            failures.append(label);key='send_angles' if kind=='joint' else 'send_coords'
            send=getattr(ctrl.mc,key);ctrl.mc.get_error_information=lambda:error_code
            setattr(ctrl.mc,key,lambda *a:False)
            try:return move(kind,target,**kwargs)
            finally:setattr(ctrl.mc,key,send)
        ctrl._move=fail
        return ctrl
    h.runtime.controller_factory=controller
    return failures


@pytest.mark.parametrize('stage,code,status,retries',[
    ('pick_pregrasp',0,'completed',1),
    ('grasp_descent',32,'completed',1),
    ('grasp_lift',0,'failed',0),
    ('place_retreat',0,'completed',0),
    ('completion_return_to_start',0,'failed',0),
    ('pick_pregrasp',16,'failed',0),
])
def test_motion_recovery_respects_gripper_phase_and_collision_stop(factory,stage,code,status,retries):
    h=factory();failures=fail_motion_once(h,stage,error_code=code)
    h.create();h.confirm();s=h.wait('completed','failed')
    assert s['status']==status,s['sequence']
    assert len(failures)==1 and s['sequence']['steps'][0].get('retry_count',0)==retries
    assert len(h.calls)==1
    if stage=='grasp_lift':
        assert not any(e['event']=='object_released_at_place' for e in s['events'])
        assert len([x for x in h.order if x[0]=='start'])==1
    if stage=='place_retreat':
        assert s['sequence']['steps'][0]['local_result_check']['passed']
        assert sum(e['event']=='object_released_at_place' for e in s['events'])==1


def test_recovery_start_failure_stops_without_second_recovery_or_capture(factory):
    h=factory();failures=fail_motion_once(h,'pick_pregrasp',fail_recovery=True)
    h.create();h.confirm();s=h.wait('failed')
    assert failures==['pick_pregrasp','initialization_before_capture']
    assert h.order==[('start',1),('capture',1),('start',2)]
    assert not any(e['event']=='object_released_at_place' for e in s['events'])


def test_motion_failure_and_bad_result_share_one_retry_budget(factory):
    h=factory();fail_motion_once(h,'pick_pregrasp');execute=h.runtime._execute;attempts=[]
    def no_move(run,prepared,**kwargs):
        attempts.append(prepared['frame_id']);before=deepcopy(h.case['objects'])
        result=execute(run,prepared,**kwargs);h.case['objects']=before
        return result
    h.runtime._execute=no_move;h.create();h.confirm();s=h.wait('failed')
    assert len(attempts)==2 and s['sequence']['steps'][0]['retry_count']==1
    assert 'step_retry_exhausted' in s['sequence']['error']['reason']
    assert sum(e['event']=='step_retry_scheduled' for e in s['sequence']['history'])==1


@pytest.mark.parametrize('outcome',['disappeared','stationary'])
def test_named_local_rule_accepts_disappearance_but_retries_stationary_target(factory,outcome):
    named=task()|{'destination_type':'named','destination_name':'右盒子','reference':None,'relation':None}
    h=factory(tasks=[named]);execute=h.runtime._execute;attempts=[]
    def result_scene(run,prepared,**kwargs):
        before=deepcopy(h.case['objects']);attempts.append(prepared['frame_id'])
        result=execute(run,prepared,**kwargs)
        h.case['objects']=[o for o in before if o['class_name']!='eraser'] if outcome=='disappeared' else before
        return result
    h.runtime._execute=result_scene;h.create();h.confirm();s=h.wait('completed','failed')
    assert s['status']==('completed' if outcome=='disappeared' else 'failed')
    assert len(attempts)==(1 if outcome=='disappeared' else 2)
    if outcome=='disappeared':
        assert s['sequence']['steps'][0]['local_result_check']['basis']=='named_target_not_observed'


def test_short_self_displacement_still_accepts_no_motion_within_tolerance(factory):
    move=task()|{'destination_type':'self_relative','reference':None,'displacement_mm':10}
    h=factory(tasks=[move]);execute=h.runtime._execute
    def no_move(run,prepared,**kwargs):
        before=deepcopy(h.case['objects']);result=execute(run,prepared,**kwargs);h.case['objects']=before
        return result
    h.runtime._execute=no_move;h.create();h.confirm();s=h.wait('completed','failed')
    assert s['status']=='completed',s['sequence']
    step=s['sequence']['steps'][0]
    assert step['local_result_check']['destination_error_mm']==pytest.approx(10)
    assert step.get('retry_count',0)==0


def test_stack_uses_original_measured_thickness_after_height_failure(factory):
    h=factory(tasks=[task('on_top_of',reference='marker')])
    support=json.loads((ROOT/'tests/data/relative_place_support_demo.json').read_text(encoding='utf-8'))
    support['objects'][0]['class_name']='marker'
    support['surface']['support_surfaces']['marker']=support['surface']['support_surfaces'].pop('box')
    h.case['objects']=support['objects'];h.case['surface']=support['surface']
    execute=h.runtime._execute;expected=[]
    def low_once(run,prepared,**kwargs):
        expected.append(prepared['result_expectation']['expected_top_z_mm'])
        before=list(h.case['objects'][1]['center_uv'])
        result=execute(run,prepared,**kwargs)
        if len(expected)==1:
            h.case['objects'][1].update(center_uv=before,depth_m=.42)  # Wrong site/height; retry must retain the original thickness.
        return result
    h.runtime._execute=low_once;h.create();h.confirm();s=h.wait('completed','failed')
    assert s['status']=='completed',s['sequence']
    step=s['sequence']['steps'][0]
    assert expected==pytest.approx([40,40],abs=1e-3)
    assert step['retry_count']==1 and step['result_expectation']['expected_top_z_mm']==pytest.approx(40,abs=1e-3)
    assert step['local_result_check']['vertical_error_mm']<1e-3


def test_swap_retry_keeps_initial_snapshot_and_does_not_repeat_completed_prefix(factory):
    tasks=[task('near'),task('at_initial',target='pen',reference='eraser'),task('at_initial')]
    h=factory(tasks=tasks);execute=h.runtime._execute;moves=[]
    def miss_second_once(run,prepared,**kwargs):
        step=run['sequence'].snapshot()['step_id'];before=deepcopy(h.case['objects'])
        moves.append((step,deepcopy(prepared['geometry']['destination_center_robot_xy_mm'])))
        result=execute(run,prepared,**kwargs)
        if [i for i,_ in moves].count(2)==1 and step==2:h.case['objects']=before
        return result
    h.runtime._execute=miss_second_once;h.create();h.confirm();s=h.wait('completed','failed')
    assert s['status']=='completed',s['sequence']
    assert [i for i,_ in moves]==[1,2,2,3]
    assert moves[1][1]==moves[2][1]
    assert [step.get('retry_count',0) for step in s['sequence']['steps']]==[0,1,0]
    assert sum(e['event']=='initial_pose_snapshot' for e in s['sequence']['history'])==1


@pytest.mark.parametrize('defect',['missing','invalid_depth'])
def test_unobservable_relative_result_stops_without_automatic_regrasp(factory,defect):
    h=factory();execute=h.runtime._execute;scene=h.runtime.scene_factory;done=[]
    def observed(*args):
        value=scene(*args)
        if done and defect=='invalid_depth':value[1].depth_m[:]=0
        return value
    def finish(run,prepared,**kwargs):
        result=execute(run,prepared,**kwargs);done.append(True)
        if defect=='missing':h.case['objects']=[o for o in h.case['objects'] if o['class_name']!='eraser']
        return result
    h.runtime._execute=finish;h.runtime.scene_factory=observed
    h.create();h.confirm();s=h.wait('failed')
    assert len(done)==1 and s['sequence']['steps'][0].get('retry_count',0)==0
    assert s['pending']['status']=='failed' and not s['outcome_verified']


def test_cancel_after_retry_is_scheduled_prevents_second_execution(factory):
    h=factory();execute=h.runtime._execute;advance=h.runtime._advance
    def no_move(run,prepared,**kwargs):
        before=deepcopy(h.case['objects']);result=execute(run,prepared,**kwargs);h.case['objects']=before
        return result
    def cancel_on_retry(run):
        seq=run['sequence'];retry=seq._retry_current_step
        def cancel(*args):
            retry(*args);run['cancelled']=True;seq.cancel()
        seq._retry_current_step=cancel
        return advance(run)
    h.runtime._execute=no_move;h.runtime._advance=cancel_on_retry
    h.create();h.confirm();s=h.wait('cancelled')
    assert sum(e['event']=='object_released_at_place' for e in s['events'])==1
    assert s['pending']['status']=='cancelled'
