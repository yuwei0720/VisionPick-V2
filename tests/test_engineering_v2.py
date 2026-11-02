"""Engineering writes touch only isolated copies; device/provider calls are forbidden."""
from copy import deepcopy
import json
from pathlib import Path
import shutil
from types import SimpleNamespace

import pytest
import yaml

from operator_console.engineering_v2 import FIELDS, _patch_yaml
from operator_console.service_v2 import ConsoleServiceV2 as ConsoleService
from operator_console.runtime_v2 import ConsoleRuntimeV2

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def engineering(tmp_path):
    for name in ['config/v0.yaml','tools.json','world_entities.json','model_policy.json','task_schema_v2.json']:
        dst = tmp_path/name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/name, dst)
    service = ConsoleService(tmp_path, hardware_enabled=False, motion_enabled=False)
    # Controlled station assets; tests never depend on a user's deployment data.
    manager = service.place_profiles
    for name in ['01','02','03','咖啡盒子']:
        candidate = manager.create_candidate({'label':name,'reference_object':'table','spatial_relation':'on_top_of',
            'place_pose_mm_deg':[80,80,120,-175,-5,25],'approach_clearance_mm':60,'retreat_clearance_mm':80,
            'planner_mode':'hybrid','preferred_ik_enabled':True,'preferred_ik_reference_angles_deg':[8,-7,-73,-8,9,0],
            'placement_source':{'type':'robot_pose_read','record':{'controlled_test':True}}})['profile']
        manager.activate(candidate['profile_id'],confirm='ACTIVATE_PLACE_PROFILE',clear_path_acknowledged=True)
        if name == '咖啡盒子': manager.set_default(candidate['profile_id'])
    catalog = manager._load()
    for row in catalog['profiles']:
        if row['display_name'] != '咖啡盒子': row['motion_safety_asset_id'] = 'controlled_stale_fingerprint'
    manager._save(catalog)
    service.table_surface.save(service._base_config(),z_mm=0,confirmed=True,method='manual_measurement')
    service.assets.register_current()
    # Bind the ready station after establishing the controlled table fingerprint.
    catalog = manager._load()
    for row in catalog['profiles']:
        if row['display_name'] == '咖啡盒子': row.update(manager._dependencies())
    manager._save(catalog)
    runtime = ConsoleRuntimeV2(service)
    yield runtime.engineering
    runtime.shutdown()
    service.shutdown()


@pytest.mark.parametrize('mode',['segmented_mode0','linear_mode1','single_mode0'])
def test_all_three_vertical_modes_are_saved_and_exposed(engineering,mode):
    result=save(engineering,{'robot.vertical_motion_mode':mode})
    assert result['values']['robot.vertical_motion_mode']==mode


def save(e, values):
    return e.save({'values': values, 'expected_revision': e.read()['revision']})


@pytest.mark.parametrize('key', list(FIELDS))
def test_tuning_preserves_ready_station_taught_values_and_invalid_stations(engineering, key):
    e = engineering
    before = e.read()
    old_catalog = json.loads(e.service.place_profiles.path.read_text(encoding='utf-8'))
    value = before['values'][key]
    value = [v + .25 for v in value] if isinstance(value, list) else value + 1
    result = save(e, {key: value})
    assert result['values'][key] == value
    after = e.service.place_profiles.status()
    assert after['default_ready']
    assert {r['display_name']: r['effective_status'] for r in after['profiles']} == {
        '01':'needs_revalidation','02':'needs_revalidation','03':'needs_revalidation','咖啡盒子':'active'}
    new_catalog = json.loads(e.service.place_profiles.path.read_text(encoding='utf-8'))
    for old, new in zip(old_catalog['profiles'], new_catalog['profiles']):
        assert {k:v for k,v in old.items() if k != 'motion_safety_asset_id'} == {
            k:v for k,v in new.items() if k != 'motion_safety_asset_id'}
    snapshot = e.service.assets.current_snapshot()
    motion = next(a['asset_id'] for a in snapshot['assets'] if a['kind']=='motion_safety_profile')
    ready = next(p for p in after['profiles'] if p['display_name']=='咖啡盒子')
    assert ready['motion_safety_asset_id'] == motion
    assert e.service.assets._load_catalog()['active_snapshot_id'] == snapshot['snapshot_id']
    assert result['events'][-1]['action'] == 'settings_saved'


@pytest.mark.parametrize('key,value', [
    ('robot.coordinate_speed',0),('robot.coordinate_speed',101),('robot.coordinate_speed',1.5),
    ('robot.coordinate_speed',True),('grasp.rx_deg',181),('grasp.grasp_z_offset_mm',-31),
    ('robot.gripper.wait_s',-1),('robot.motion_timeout_s',float('nan')),
    ('robot.motion_timeout_s',float('inf')),('scene.sequence_gap_mm',201),
    ('grasp.tcp_xy_offset_mm',[1]),('grasp.tcp_xy_offset_mm',[1,2,3]),
    ('grasp.tcp_xy_offset_mm',[1,'x']),('grasp.tcp_xy_offset_mm',3),
    ('calibration.ready_for_motion',True),
])
def test_invalid_values_write_nothing(engineering,key,value):
    e=engineering
    originals={p:p.read_bytes() if p.exists() else None for p in e._paths()}
    with pytest.raises((ValueError,TypeError)):
        save(e,{key:value})
    assert all((p.read_bytes() if p.exists() else None)==v for p,v in originals.items())


def test_stale_revision_and_active_worker_reject_before_writes(engineering):
    e=engineering
    before=e.read()
    save(e,{'grasp.grasp_z_offset_mm':2})
    with pytest.raises(ValueError,match='stale_engineering'):
        e.save({'values':{'grasp.grasp_z_offset_mm':3},'expected_revision':before['revision']})
    e.runtime._thread=SimpleNamespace(is_alive=lambda:True,join=lambda **k:None)
    with pytest.raises(ValueError,match='locked_while_running'):
        save(e,{'grasp.grasp_z_offset_mm':3})
    e.runtime._thread=None


def test_quarantined_transport_blocks_tuning(engineering):
    e=engineering
    e.service._quarantined_leases.append(object())
    with pytest.raises(ValueError,match='robot_ownership_quarantined'):
        save(e,{'robot.gripper.wait_s':2})
    e.service._quarantined_leases.clear()


def test_sync_failure_restores_all_original_files(engineering,monkeypatch):
    e=engineering
    originals={p:p.read_bytes() if p.exists() else None for p in e._paths()}
    def fail(): raise OSError('injected_registry_write_failure')
    monkeypatch.setattr(e.service.assets,'register_current',fail)
    with pytest.raises(OSError,match='injected_registry'):
        save(e,{'grasp.tcp_xy_offset_mm':[-5,-5]})
    assert all((p.read_bytes() if p.exists() else None)==v for p,v in originals.items())
    assert e.read()['profiles'][-1]['effective_status']=='active'


def test_comment_preserving_yaml_patch_and_only_selected_values_change(engineering):
    text=Path(engineering.service.config).read_text(encoding='utf-8')
    changed=_patch_yaml(text,'grasp.tcp_xy_offset_mm',[-3,-4])
    changed=_patch_yaml(changed,'scene.sequence_gap_mm',60)
    before=yaml.safe_load(text); after=yaml.safe_load(changed)
    before['grasp']['tcp_xy_offset_mm']=[-3,-4]
    before.setdefault('scene',{})['sequence_gap_mm']=60
    assert before==after
    assert all(line in changed for line in text.splitlines() if line.lstrip().startswith('#'))


def test_no_hardware_opt_in_blocks_pose_gripper_and_real_preview(engineering):
    e=engineering
    with pytest.raises(RuntimeError,match='hardware_disabled'): e.pose({})
    for action in ['open','close']:
        with pytest.raises(ValueError,match='real_motion_not_enabled'):
            e.gripper({'action':action,'confirm':action.upper()+'_GRIPPER'})
    with pytest.raises(ValueError,match='real_motion_not_enabled'):
        e.runtime.preview({'mode':'real'})


def test_station_teaching_requires_fresh_paired_feedback(engineering):
    e=engineering
    with pytest.raises(ValueError,match='fresh_pose_read_required'):
        e.station({'action':'teach','name':'新站點'})


def test_station_metadata_and_explicit_disable(engineering):
    e=engineering
    pid=e.service.place_profiles.status()['default_profile_id']
    result=e.station({'action':'metadata','profile_id':pid,'name':'咖啡盒子','aliases':['Coffee']})
    assert result['aliases']==['Coffee']
    e.station({'action':'disable','profile_id':pid,'confirm':'DISABLE_PLACE_PROFILE'})
    assert not e.service.place_profiles.status()['default_ready']


def test_calibration_change_cannot_be_hidden_by_fingerprint_sync(engineering):
    e=engineering
    previous=e.service.place_profiles._dependencies()
    text=Path(e.service.config).read_text(encoding='utf-8')
    # A source calibration edit, rather than an allowed engineering tuning key.
    cfg=yaml.safe_load(text);cfg['calibration']['rmse_mm']+=.01
    Path(e.service.config).write_text(yaml.safe_dump(cfg),encoding='utf-8')
    with pytest.raises(ValueError,match='calibration_changed'):
        e.service.place_profiles.sync_motion_fingerprint(previous,[])


def test_taught_update_keeps_family_and_default_without_guessed_pre_place_pair(engineering):
    import time
    from operator_console.place_catalog_v2 import PlaceCatalogV2
    e=engineering
    pid=e.service.place_profiles.status()['default_profile_id']
    original=e.service.place_profiles.get(pid)
    e.last_pose={'coords_mm_deg':original['place_pose_mm_deg'],
                 'angles_deg':[8,-7,-73,-8,9,0], 'read_at_unix_ms':time.time()*1000}
    candidate=e.station({'action':'teach','profile_id':pid,'name':original['display_name'],'aliases':['Coffee'],'clear_path_acknowledged':True})['profile']
    assert candidate['profile_key']==original['profile_key'] and candidate['version']==original['version']+1
    assert candidate['pre_place_joint_angles_deg'] is None
    e.station({'action':'activate','profile_id':candidate['profile_id'],
               'confirm':'ACTIVATE_PLACE_PROFILE','clear_path_acknowledged':True})
    status=PlaceCatalogV2(e.service.place_profiles.status()).status()
    assert status['default_ready'] and status['default_profile_id']==candidate['profile_id']
    assert not status['conflicts']


@pytest.mark.parametrize('fail_close',[False,True])
def test_manual_gripper_uses_exclusive_fake_transport_and_preserves_uncertainty(engineering,monkeypatch,fail_close):
    import operator_console.engineering_v2 as module
    e=engineering; calls=[]
    e.service.hardware_enabled=True;e.service.motion_enabled=True
    class Lease:
        def __init__(self,*a): pass
        def acquire(self): calls.append('acquire');return self
        def release(self): calls.append('release')
    class Controller:
        def __init__(self,*a,**k): self.mc=self
        def connect(self): calls.append('connect')
        def open_gripper(self): calls.append('open')
        def close(self):
            calls.append('close')
            if fail_close: raise OSError('injected_close_failure')
    monkeypatch.setattr(module,'RobotLease',Lease);monkeypatch.setattr(module,'DirectRobotControllerV2',Controller)
    if fail_close:
        with pytest.raises(OSError,match='injected_close_failure'):
            e.gripper({'action':'open','confirm':'OPEN_GRIPPER'})
        assert calls==['acquire','connect','open','close'] and len(e.service._quarantined_leases)==1
        e.service._quarantined_leases.clear()
    else:
        e.gripper({'action':'open','confirm':'OPEN_GRIPPER'})
        assert calls==['acquire','connect','open','close','release']
    assert e.runtime._holding()


def test_empty_hand_ack_cannot_race_quarantined_robot_work(engineering):
    e=engineering
    e.service._quarantined_leases.append(object())
    with pytest.raises(ValueError,match='robot_ownership_quarantined'):
        e.runtime.acknowledge_empty({'confirm':'EMPTY_GRIPPER'})
    e.service._quarantined_leases.clear()


def test_engineering_http_token_origin_and_valid_save_contract(engineering):
    import threading
    from urllib.request import Request,urlopen
    from urllib.error import HTTPError
    from operator_console.server import create_server
    e=engineering;server=create_server(e.service,port=0,v2_runtime=e.runtime)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base=f'http://127.0.0.1:{server.server_address[1]}'
    def http(path,payload=None,token=None,origin=None):
        headers={} if payload is None else {'Content-Type':'application/json','X-VisionPick-Token':token or 'wrong'}
        if origin:headers['Origin']=origin
        request=Request(base+path,headers=headers,data=None if payload is None else json.dumps(payload).encode())
        try:response=urlopen(request,timeout=5)
        except HTTPError as exc:response=exc
        with response:return response.status,json.loads(response.read())
    try:
        _,boot=http('/api/v2/bootstrap');token=boot['session']
        _,data=http('/api/v2/engineering')
        payload={'values':{'grasp.grasp_z_offset_mm':2},'expected_revision':data['revision']}
        assert http('/api/v2/engineering/settings',payload)[0]==403
        assert http('/api/v2/engineering/settings',payload,token,'https://external.invalid')[0]==403
        code,saved=http('/api/v2/engineering/settings',payload,token)
        assert code==200 and saved['values']['grasp.grasp_z_offset_mm']==2
        assert http('/api/v2/bootstrap')[1]['catalog']['default_ready']
    finally:
        server.shutdown();server.server_close();thread.join(3)


@pytest.mark.parametrize('key',list(__import__('operator_console.engineering_v2',fromlist=['ENUM_FIELDS']).ENUM_FIELDS))
def test_enum_settings_sync_only_previously_active_station(engineering,key):
    from operator_console.engineering_v2 import ENUM_FIELDS
    e=engineering;before=e.read();old=json.loads(e.service.place_profiles.path.read_text(encoding='utf-8'))
    choices=ENUM_FIELDS[key][1];value=next(c for c in choices if c!=before['values'][key])
    result=save(e,{key:value});assert result['values'][key]==value and len(result['synced_profile_ids'])==1
    assert e.service.place_profiles.status()['default_ready']
    new=json.loads(e.service.place_profiles.path.read_text(encoding='utf-8'))
    for a,b in zip(old['profiles'],new['profiles']):
        assert {k:v for k,v in a.items() if k!='motion_safety_asset_id'}=={k:v for k,v in b.items() if k!='motion_safety_asset_id'}


@pytest.mark.parametrize('key,value',[
    ('robot.vertical_motion_mode','bad'),('robot.vertical_motion_mode',True),
    ('robot.vertical_segment_step_mm',-1),('robot.vertical_segment_step_mm',float('nan')),
    ('orientation.fixed_place_yaw_deg',181),('orientation.fixed_place_yaw_deg',True),
    ('orientation.place_orientation_policy_named','fixed_deg'),
    ('orientation.place_orientation_policy_self_relative','align_reference'),
    ('orientation.place_orientation_policy_swap','profile_pose')])
def test_new_engineering_invalid_values_do_not_write(engineering,key,value):
    e=engineering;paths=e._paths();before={p:p.read_bytes() if p.exists() else None for p in paths}
    with pytest.raises(ValueError):save(e,{key:value})
    assert before=={p:p.read_bytes() if p.exists() else None for p in paths}


def test_engineering_yaw_save_refreshes_waiting_plan_and_preview_without_hardware(engineering):
    from operator_console.task_pending_v2 import PendingTaskPlanV2
    from operator_console.simulation import synthetic_scene
    from operator_console.task_sequence_v2 import ObservationV2
    from operator_console.runtime_v2 import _identity
    import time
    e=engineering;rt=e.runtime;cfg=e.service._base_config();case=json.loads((ROOT/'tests/data/relative_place_s3_demo.json').read_text(encoding='utf-8'))
    ds,frame,d,p=synthetic_scene(case);fid='f'*32
    instances=tuple({'instance_id':fid+':'+str(i),'class_name':obj.class_name,'index':i} for i,obj in enumerate(ds))
    obs=ObservationV2(fid,time.time()*1000,instances,payload={'config':case['config'],'surface':case['surface'],'detections':ds,'frame':frame,'deproject':d,'project':p})
    pending=PendingTaskPlanV2({'tasks':[{'target':'eraser','destination_type':'relative','destination_name':None,'reference':'pen','relation':'right_of'}]},original_text='test',context_options={'tools':{'eraser':{},'pen':{}}})
    run={'id':'a'*32,'config':cfg,'config_revision':_identity(cfg),'catalog':rt._catalog(),'arrival':rt.settings.freeze(),
        'pending':pending,'observation':obs,'mode':'offline'}
    rt._runs[run['id']]=run;rt._active=run['id'];rt._refresh_placement_previews(run)
    before=pending.snapshot()['version'];assert run['placement_previews'][0]['rz_deg']!=72.
    result=save(e,{'orientation.place_orientation_policy_relative':'fixed_deg','orientation.fixed_place_yaw_deg':72.})
    assert pending.snapshot()['version']==before+1 and run['placement_previews'][0]['rz_deg']==72.
    assert run['config_revision']==_identity(e.service._base_config()) and result['synced_profile_ids']
    assert 'controller' not in run
    # Dispose this manually constructed plan before the fixture shutdown.
    rt._runs.clear();rt._active=None
