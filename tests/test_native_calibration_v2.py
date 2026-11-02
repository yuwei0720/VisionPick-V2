"""Frozen RGB-D and transport doubles; never connect devices or providers."""
from copy import deepcopy
import importlib
import json
from pathlib import Path
import shutil
import threading
import time
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np
import pytest
import yaml

from operator_console.runtime_v2 import ConsoleRuntimeV2
from operator_console.service_v2 import ConsoleServiceV2
from operator_console.server import create_server

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def native(tmp_path, monkeypatch):
    for name in ['config/v0.yaml', 'tools.json', 'world_entities.json', 'model_policy.json', 'task_schema_v2.json']:
        dest = tmp_path/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/name, dest)
    path = tmp_path/'config/v0.yaml'
    cfg = yaml.safe_load(path.read_text(encoding='utf-8'))
    cfg['calibration']['T_robot_from_camera'] = [[1,0,0,0],[0,1,0,0],[0,0,1,-.3],[0,0,0,1]]
    cfg['grasp']['workspace_mm'] = {'x': [-500,500], 'y': [-500,500], 'z': [-100,600]}
    path.write_text(yaml.safe_dump(cfg), encoding='utf-8')
    service = ConsoleServiceV2(tmp_path)
    runtime = ConsoleRuntimeV2(service)
    intr = SimpleNamespace(width=80, height=60, ppx=40, ppy=30, fx=100, fy=100, model='none', coeffs=[0]*5)
    # Spatially varying depth exposes incorrect rotation rather than hiding it
    # behind a uniform plane. The selected ROI still stays flat within 25 mm.
    depth = np.full((60,80), .4, np.float32)
    depth[10:30, 20:40] = .42
    frame = SimpleNamespace(color_bgr=np.zeros((60,80,3), np.uint8), depth_m=depth, intrinsics=intr)
    capture = service.place_capture.persist(frame, cfg)
    monkeypatch.setattr(service.place_capture, 'capture', lambda: service.place_capture.persist(frame, service._base_config()))
    yield SimpleNamespace(runtime=runtime, service=service, frame=frame, capture=capture, root=tmp_path, initial_catalog=service.place_profiles.path.read_bytes())
    runtime.shutdown()


def roi(n, **extra):
    return {'capture_id': n.capture['capture_id'], 'roi_xyxy': [20,10,40,30], 'rotation_deg': 0, **extra}


def teach(n, name='示教收納區', **extra):
    n.runtime.engineering.last_pose = {'coords_mm_deg': [80,60,120,-172.,-7.,34.],
        'angles_deg': [10,-20,-60,-15,8,42], 'read_at_unix_ms': time.time()*1000}
    return n.runtime.engineering.station({'action':'teach','name':name,'aliases':['收納'],
        'clear_path_acknowledged':True, **extra})['profile']


@pytest.mark.parametrize('rotation,rect', [(0,[20,10,40,30]),(90,[10,40,30,60]),
                                         (180,[40,30,60,50]),(270,[30,20,50,40])])
@pytest.mark.parametrize('reverse', [False,True])
def test_rotated_display_edges_resolve_same_raw_depth_region(native, rotation, rect, reverse):
    if reverse: rect = rect[2:] + rect[:2]
    result = native.runtime.roi_capture(roi(native, rotation_deg=rotation, roi_xyxy=rect))
    assert result['selected_roi_px'] == [20,10,39,29]
    assert result['depth_valid_ratio'] == 1
    assert result['depth_valid_pixels'] == 400
    assert result['robot_surface_xyz_mm'][2] == pytest.approx(120, abs=.001)
    assert result['place_pose_mm_deg'][3:] == [-175.,-5.,-45.]
    assert not native.service.place_profiles.status()['profiles']


@pytest.mark.parametrize('extra', [dict(rotation_deg=45),dict(rotation_deg=True),
    dict(roi_xyxy=[-1,0,20,20]),dict(roi_xyxy=[0,0,81,60]),dict(roi_xyxy=[10,10,10,20]),
    dict(roi_xyxy=[False,0,20,20]),dict(roi_xyxy=[0,0,float('nan'),20]),dict(rz_deg=181),dict(save='yes')])
def test_invalid_roi_rejected_before_catalog_write(native, extra):
    with pytest.raises((ValueError,RuntimeError)):
        native.runtime.roi_capture(roi(native, **extra))
    assert native.service.place_profiles.path.read_bytes() == native.initial_catalog


def test_depth_holes_report_ratio_and_empty_depth_cannot_activate(native):
    path = native.service.place_capture._path(native.capture['capture_id'])/'depth_m.npy'
    depth = np.load(path);depth[10:20,20:40] = 0;np.save(path,depth)
    assert native.runtime.roi_capture(roi(native))['depth_valid_ratio'] == .5
    depth[:] = 0;np.save(path,depth)
    with pytest.raises(ValueError,match='depth pixel'):
        native.runtime.roi_capture(roi(native, save=True, name='不成立',clear_path_acknowledged=True))
    assert native.service.place_profiles.path.read_bytes() == native.initial_catalog


def test_roi_save_update_default_and_registry_are_immediately_ready(native):
    first = native.runtime.roi_capture(roi(native, save=True, name='ROI 盒子', aliases=['視覺盒'],
                                          clear_path_acknowledged=True, rz_deg=17))
    pid = first['profile']['profile_id']
    native.runtime.engineering.station({'action':'default','profile_id':pid})
    second = native.runtime.roi_capture(roi(native, save=True, profile_id=pid, name='ROI 新名稱',
                                            clear_path_acknowledged=True, rz_deg=24))
    profile = second['profile']
    assert profile['profile_key'] == first['profile']['profile_key']
    assert profile['version'] == 2 and profile['place_pose_mm_deg'][3:] == [-175.,-5.,24.]
    assert native.runtime._catalog().direct_place('ROI 新名稱').destination_check_policy == 'rgbd_envelope'
    assert second['catalog']['default_ready']
    assert second['catalog']['default_profile_id'] == profile['profile_id']
    assert native.service.assets.path.is_file()


def test_teaching_preserves_real_orientation_and_paired_place_joints(native):
    profile = teach(native)
    assert profile['effective_status'] == 'active'
    assert profile['place_pose_mm_deg'][3:] == [-172.,-7.,34.]
    assert profile['place_joint_angles_deg'] == [10,-20,-60,-15,8,42]
    assert profile['pre_place_joint_angles_deg'] is None
    assert native.runtime._catalog().direct_place('示教收納區').destination_check_policy == 'operator_taught'
    assert native.service.assets.catalog()['snapshots']


def test_alias_conflict_rolls_back_station_and_registry(native):
    teach(native)
    before = native.service.place_profiles.path.read_bytes(), native.service.assets.path.read_bytes()
    with pytest.raises(ValueError,match='conflict'):
        native.runtime.roi_capture(roi(native,save=True,name='第二站',aliases=['收納'],clear_path_acknowledged=True))
    assert before == (native.service.place_profiles.path.read_bytes(), native.service.assets.path.read_bytes())


def test_table_recalibration_updates_motion_fingerprint_and_keeps_active_station(native):
    profile = teach(native)
    old = profile['motion_safety_asset_id']
    result = native.runtime.table_calibrate({**roi(native), 'confirmed':True})
    assert result['z_mm'] == pytest.approx(120, abs=.001)
    assert result['synced_profile_ids'] == [profile['profile_id']]
    current = native.service.place_profiles.get(profile['profile_id'])
    assert current['effective_status'] == 'active' and current['motion_safety_asset_id'] != old
    assert current['place_pose_mm_deg'] == profile['place_pose_mm_deg']


def test_table_and_station_changes_roll_back_when_registry_write_fails(native, monkeypatch):
    profile = teach(native)
    paths = [native.service.place_profiles.path,native.service.assets.path,native.service.table_surface.path]
    before = {p:p.read_bytes() if p.exists() else None for p in paths}
    def fail(): raise OSError('injected_registry_failure')
    monkeypatch.setattr(native.service.assets,'register_current',fail)
    with pytest.raises(OSError,match='injected_registry'):
        native.runtime.table_calibrate({**roi(native),'confirmed':True})
    assert all((p.read_bytes() if p.exists() else None) == data for p,data in before.items())
    assert native.service.place_profiles.get(profile['profile_id'])['effective_status'] == 'active'


def test_changed_calibration_rejects_old_rgbd(native):
    path = Path(native.service.config);cfg = native.service._base_config()
    cfg['calibration']['T_robot_from_camera'][0][3] += .001
    path.write_text(yaml.safe_dump(cfg),encoding='utf-8')
    with pytest.raises(ValueError,match='configuration_changed'):
        native.runtime.roi_capture(roi(native))


def test_explicit_config_is_the_registry_motion_fingerprint_source(native):
    cfg=native.service._base_config();cfg['grasp']['tcp_xy_offset_mm']=[7,9]
    alternate=native.root/'config/custom.yaml';alternate.write_text(yaml.safe_dump(cfg),encoding='utf-8')
    service=ConsoleServiceV2(native.root,config='config/custom.yaml')
    before=native.service.place_profiles._dependencies()
    after=service.place_profiles._dependencies()
    assert before['motion_safety_asset_id'] != after['motion_safety_asset_id']
    snapshot=service.assets.current_snapshot()
    config_asset=next(a for a in snapshot['assets'] if a['kind']=='runtime_config')
    assert config_asset['path']=='config/custom.yaml'


def test_current_capture_rejects_simulated_expired_or_moved_frames(native):
    from operator_console.runtime_v2 import _identity
    native.service.hardware_enabled = True
    native.runtime._calibration_frame = {'frame':native.frame,'synthetic':True,'at':time.monotonic(),
                                         'config_revision':_identity(native.service._base_config())}
    with pytest.raises(ValueError,match='real_rgbd'): native.runtime.capture_rgbd({'source':'current'})
    native.runtime._calibration_frame.update(synthetic=False,at=time.monotonic()-61)
    with pytest.raises(ValueError,match='expired'): native.runtime.capture_rgbd({'source':'current'})
    native.runtime._calibration_frame['at'] = time.monotonic()
    assert native.runtime.capture_rgbd({'source':'current'})['frame_url'].startswith('/api/v2/')


def test_native_pose_read_only_lease_and_teaching_session(native,monkeypatch):
    calls = []
    class Controller:
        def __init__(self,*args,**kwargs): self.mc=self
        def connect(self): calls.append('connect')
        def get_coords(self): calls.append('coords');return [80,60,120,-172,-7,34]
        def _try_get_angles(self): calls.append('angles');return [10,-20,-60,-15,8,42]
        def close(self): calls.append('close')
        def enter_free_mode(self): calls.append('free')
        def exit_free_mode(self): calls.append('focus')
    class Lease:
        def __init__(self,*args): pass
        def acquire(self): calls.append('lease');return self
        def release(self): calls.append('release')
    monkeypatch.setattr('operator_console.runtime_v2.DirectRobotControllerV2',Controller)
    monkeypatch.setattr('operator_console.runtime_v2.RobotLease',Lease)
    native.service.hardware_enabled = native.service.motion_enabled = True
    record = native.runtime.engineering.pose({})
    assert record['motion_command_sent'] is False and calls == ['lease','connect','coords','angles','close','release']
    native.runtime.acknowledge_empty({'confirm':'EMPTY_GRIPPER'})
    native.runtime.teaching({'action':'enter','confirm':'ENTER_TEACH_MODE'})
    record = native.runtime.engineering.pose({})
    assert record['teaching_mode_active']
    with pytest.raises(ValueError,match='teaching_session_active'):
        native.runtime.preview({'mode':'offline'})
    native.runtime.teaching({'action':'exit','confirm':'EXIT_TEACH_MODE'})
    assert calls[-3:] == ['focus','close','release']
    assert teach(native)['effective_status'] == 'active'


@pytest.mark.parametrize('operation,stage',[('pose','construct'),('pose','connect'),
    ('teaching','construct'),('teaching','connect'),('teaching','free')])
def test_maintenance_failures_release_unused_lease_but_retain_uncertain_torque(native,monkeypatch,operation,stage):
    calls=[]
    class Controller:
        def __init__(self,*args,**kwargs):
            if stage=='construct': raise RuntimeError('controlled_construct')
            self.mc=self
        def connect(self):
            if stage=='connect': raise RuntimeError('controlled_connect')
        def enter_free_mode(self): raise RuntimeError('controlled_free')
        def exit_free_mode(self): calls.append('focus')
        def close(self): calls.append('close')
    class Lease:
        def __init__(self,*args): pass
        def acquire(self): calls.append('lease');return self
        def release(self): calls.append('release')
    monkeypatch.setattr('operator_console.runtime_v2.DirectRobotControllerV2',Controller)
    monkeypatch.setattr('operator_console.runtime_v2.RobotLease',Lease)
    native.service.hardware_enabled=native.service.motion_enabled=True
    native.runtime.acknowledge_empty({'confirm':'EMPTY_GRIPPER'})
    with pytest.raises(RuntimeError,match='controlled_'+stage):
        if operation=='pose': native.runtime.read_pose({})
        else: native.runtime.teaching({'action':'enter','confirm':'ENTER_TEACH_MODE'})
    if stage=='free':
        assert calls==['lease'] and native.runtime._teaching_controller is not None
        native.runtime.teaching({'action':'exit','confirm':'EXIT_TEACH_MODE'})
        assert calls==['lease','focus','close','release']
    else:
        assert calls==(['lease','release'] if stage=='construct' else ['lease','close','release'])
        assert getattr(native.runtime,'_teaching_controller',None) is None


def test_station_management_lifecycle_and_default_safety(native):
    profile=teach(native);pid=profile['profile_id'];engineering=native.runtime.engineering
    engineering.station({'action':'default','profile_id':pid})
    engineering.station({'action':'metadata','profile_id':pid,'name':'新示教名稱','aliases':['新別名']})
    assert native.runtime._catalog().resolve_answer('新別名')['profile_id']==pid
    engineering.station({'action':'disable','profile_id':pid,'confirm':'DISABLE_PLACE_PROFILE'})
    assert native.runtime._catalog().status()['default_ready'] is False
    engineering.station({'action':'revalidate','profile_id':pid,'confirm':'ACTIVATE_PLACE_PROFILE','clear_path_acknowledged':True})
    assert native.runtime._catalog().resolve_name('新示教名稱')['effective_status']=='active'
    engineering.station({'action':'delete','profile_id':pid,'confirm':'DELETE_PLACE_PROFILE'})
    assert not native.runtime._catalog().status()['stations']


def test_native_home_requires_no_camera_and_uses_only_v2_controller(native,monkeypatch):
    calls=[];native.service.hardware_enabled=native.service.motion_enabled=True
    monkeypatch.setattr(native.service,'device_status',lambda:{'robot':{'detected':True},'camera':{'detected':False,'count':0}})
    class Controller:
        gripper_holding_uncertain=False
        at_start=False
        def initialize_for_pick(self):
            calls.append('start');self.at_start=True
            return {'start_verified':True,'control_completed':True}
        def close(self): calls.append('close')
    native.runtime.controller_factory=lambda *args:Controller()
    result=native.runtime.home({'confirm':'RETURN_HOME'})
    native.runtime._thread.join(5)
    assert native.runtime.state(result['run_id'])['status']=='completed'
    assert calls==['start','close'] and native.runtime._empty_hand_confirmed()


def test_native_routes_guard_tokens_and_retire_all_v1_routes(native):
    server = create_server(native.service,port=0,v2_runtime=native.runtime)
    thread = threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base = f'http://127.0.0.1:{server.server_address[1]}'
    def http(path,payload=None,token=None,origin=None):
        headers = {'Content-Type':'application/json','X-VisionPick-Token':token or ''}
        if origin: headers['Origin']=origin
        request = Request(base+path,headers=headers,data=json.dumps(payload).encode() if payload is not None else None)
        try: response = urlopen(request,timeout=5)
        except HTTPError as exc: response=exc
        with response:
            data=response.read()
            return response.status,json.loads(data) if response.headers['Content-Type'].startswith('application/json') else data
    try:
        token = http('/api/v2/bootstrap')[1]['session']
        for old in ['/legacy/','/api/bootstrap','/api/place-profiles','/api/robot/home','/app.js']:
            assert http(old)[0] == 404
        for old in ['/api/runs','/api/place-captures','/api/robot/home','/api/preflight']:
            assert http(old,{},token)[0] == 404
        assert http('/api/v2/places/roi_capture',roi(native))[0] == 403
        assert http('/api/v2/places/roi_capture',roi(native),token,'https://untrusted.invalid')[0] == 403
        assert http('/api/v2/places/roi_capture',roi(native),token)[0] == 200
        assert http(native.capture['frame_url'])[0] == 200
        assert http('/api/v2/table-surface/calibrate',{**roi(native),'confirmed':True},token)[0] == 200
    finally:
        server.shutdown();server.server_close();thread.join(5)


def test_production_entrypoint_does_not_load_v1_business_modules():
    import subprocess,sys
    result = subprocess.run([sys.executable,'-c',
        "import sys;sys.path.insert(0,'src');import operator_console.cli,operator_console.runtime_v2;"
        "bad=[m for m in sys.modules if m in {'AI_MVP','operator_console.runtime','operator_console.real_sequence','operator_console.task_plan','operator_console.destination_resolver'}];"
        "assert not bad,bad;print('zero V1 imports')"],cwd=ROOT,capture_output=True,text=True)
    assert result.returncode == 0,result.stderr


def test_browser_native_tabs_rotated_roi_teaching_and_table(native,monkeypatch):
    import os
    import subprocess
    import cv2
    native.service.hardware_enabled = True
    # Exercise a full 640x480 camera surface, preserving the same depth region.
    color = np.full((480,640,3), (238,235,230), dtype=np.uint8)
    for x in range(0,640,80): cv2.line(color,(x,0),(x,479),(215,210,205),1)
    for y in range(0,480,80): cv2.line(color,(0,y),(639,y),(215,210,205),1)
    cv2.rectangle(color,(160,80),(319,239),(170,130,90),-1)
    cv2.putText(color,'CONTROLLED RGB-D / NO HARDWARE',(30,440),cv2.FONT_HERSHEY_SIMPLEX,.6,(75,65,55),1)
    frame = SimpleNamespace(color_bgr=color, depth_m=np.repeat(np.repeat(native.frame.depth_m,8,axis=0),8,axis=1),
        intrinsics=SimpleNamespace(width=640,height=480,ppx=320,ppy=240,fx=800,fy=800,model='none',coeffs=[0]*5))
    monkeypatch.setattr(native.service.place_capture,'capture',lambda:
        native.service.place_capture.persist(frame,native.service._base_config(),source='controlled_browser_fixture'))
    # The HTTP endpoints are real. Only camera capture and paired feedback are
    # controlled doubles; motion authorization remains disabled throughout.
    monkeypatch.setattr(native.runtime,'read_pose',lambda payload:{
        'coords_mm_deg':[80,60,120,-172,-7,34], 'angles_deg':[10,-20,-60,-15,8,42],
        'read_at_unix_ms':time.time()*1000, 'motion_command_sent':False, 'gripper_command_sent':False})
    monkeypatch.setattr(native.service,'device_status',lambda:{
        'robot':{'detected':True,'configured_port':'CONTROLLED'},
        'camera':{'detected':True,'count':1},'ai':{'provider':'controlled'}})
    server = create_server(native.service,port=0,v2_runtime=native.runtime)
    thread = threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    out = native.root/'browser';out.mkdir()
    packages = Path.home()/'.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules'
    args = {'url':f'http://127.0.0.1:{server.server_address[1]}','output':str(out),'scale':8}
    try:
        result = subprocess.run(['node',str(ROOT/'tests/native_calibration_v2_browser.cjs'),json.dumps(args)],
            capture_output=True,text=True,encoding='utf-8',timeout=90,
            env={**os.environ,'NODE_PATH':str(packages)})
        if os.getenv('VISIONPICK_BROWSER_OUTPUT'):
            shutil.copytree(out,Path(os.environ['VISIONPICK_BROWSER_OUTPUT']),dirs_exist_ok=True)
        assert result.returncode == 0,result.stdout+result.stderr
        report = json.loads((out/'browser-report.json').read_text(encoding='utf-8'))
        assert report['errors'] == [] and len(report['screens']) == 8
        if os.getenv('VISIONPICK_BROWSER_OUTPUT'):
            shutil.copytree(out,Path(os.environ['VISIONPICK_BROWSER_OUTPUT']),dirs_exist_ok=True)
    finally:
        server.shutdown();server.server_close();thread.join(5)
