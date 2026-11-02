"""Q1-Q4 regression at HTTP and local state boundaries; no physical devices/API."""
from copy import deepcopy
import hashlib
import json
import time
from pathlib import Path

import pytest

from test_v2_console_integration import factory, task, create_real
from operator_console.task_pending_v2 import PendingTaskPlanV2, PendingTaskError
from operator_console.task_parser_v2 import SYSTEM_PROMPT


def devices(camera=True, robot=True, count=None):
    return {'camera': {'detected': camera, 'count': int(camera) if count is None else count},
            'robot': {'detected': robot}}


@pytest.mark.parametrize('status',[devices(False,False),devices(False,True),devices(True,False),devices(True,True,0)])
def test_q1_offline_real_semantics_local_answer_but_zero_lease(factory, status):
    h=factory(tasks=[task() | {'target':None}])
    h.service.device_status=lambda:deepcopy(status)
    code,s,_=h.http('/api/v2/runs',{'text':'受控離線語意','semantic_source':'api','mode':'real'})
    assert code==201;h.run_id=s['run_id'];s=h.wait('waiting_answer')
    q=s['pending']['question']
    assert h.action('answer',{'value':'eraser','version':q['version'],'slot':q['slot'],'task_index':q['task_index']})[0]==200
    assert h.wait('waiting_confirmation')['status']=='waiting_confirmation'
    code,result,_=h.confirm()
    assert code==400 and result['error']=='hardware_not_detected'
    assert len(h.calls)==1 and h.order==h.backends==[]
    assert h.runtime._runs[h.run_id].get('lease') is None


def test_q1_historical_prompt_hash_is_not_reused_for_v3():
    from operator_console.task_parser_v2 import PROMPT_VERSION
    assert PROMPT_VERSION=='3.0.0'
    assert hashlib.sha256(SYSTEM_PROMPT.encode()).hexdigest()!='842aaf2353852250bc08076a8188d3f13d896b46a844b1e26cfde83a5a4a609e'


@pytest.mark.parametrize('fault',['missing','stale','configuration','after_move'])
def test_q2_real_parse_auto_observes_invalid_scene_before_parser(factory, fault):
    from operator_console.runtime_v2 import _identity
    h=factory();h.service.hardware_enabled=True
    h.service.device_status=lambda:devices(True,False)
    h.runtime._visible_scene={'mode':'real','config_revision':_identity(h.cfg),'captured_at_unix_ms':time.time()*1000,'objects':['battery'],'frame_id':'old'} if fault in {'stale','configuration'} else None
    if fault=='stale':h.runtime._visible_scene['captured_at_unix_ms']=0
    if fault=='configuration':h.runtime._visible_scene['config_revision']='old'
    if fault=='after_move':
        h.create();h.confirm();completed=h.wait('completed')
        assert h.runtime._visible_scene is None and completed['final_preview'] is not None
        h.calls.clear();h.order.clear();h.backends.clear()
    create_real(h)
    assert h.calls[0][1]['visible_objects']==['pen','eraser']
    assert h.order==[('capture',1)] and h.backends==[]
    assert h.runtime.state(h.run_id)['semantic_scene']['frame_id']


def test_q2_real_fresh_cache_is_reused_and_offline_hardware_clears_it(factory):
    h=factory();h.service.hardware_enabled=True;h.service.device_status=lambda:devices(True,False)
    code,s,_=h.http('/api/v2/preview',{'mode':'real'});assert code==201
    h.run_id=s['run_id'];h.wait('completed');create_real(h)
    assert h.order==[('capture',1)] and h.calls[0][1]['visible_objects']==['pen','eraser']
    h.action('cancel');h.service.device_status=lambda:devices(False,False);create_real(h)
    assert h.calls[-1][1]['visible_objects'] is None and len(h.order)==1


def test_q2_auto_start_requires_explicit_empty_hand_ack(factory):
    h=factory();h.service.hardware_enabled=h.service.motion_enabled=True
    h.service.device_status=lambda:devices()
    h.cfg['calibration'].update(ready_for_motion=True,rmse_mm=1,max_rmse_mm=15)
    assert h.runtime.acknowledge_empty({'confirm':'EMPTY_GRIPPER'})['holding_uncertain'] is False
    create_real(h)
    assert h.order==[('start',1),('capture',1)] and len(h.backends)==1
    assert h.runtime._runs[h.run_id].get('controller') is None


def test_q2_camera_only_without_calibration_or_table_never_initializes_arm(factory, monkeypatch):
    from test_v2_console_integration import native_real
    h=factory();order=native_real(h,monkeypatch)
    h.service.motion_enabled=False;h.service.device_status=lambda:devices(True,False)
    h.cfg['calibration']={}
    h.service.table_surface.require=lambda cfg:pytest.fail('semantic detection must not require table geometry')
    create_real(h)
    assert h.calls[0][1]['visible_objects']==['pen','eraser']
    assert h.backends==[] and 'lease_acquired' not in order and 'camera_closed' in order


def test_q2_empty_camera_scene_is_not_replaced_with_none(factory):
    h=factory();h.tasks=[];h.case['objects']=[]
    h.service.hardware_enabled=True;h.service.device_status=lambda:devices(True,False)
    code,s,_=h.http('/api/v2/runs',{'text':'收文具','semantic_source':'api','mode':'real'})
    assert code==201;h.run_id=s['run_id'];s=h.wait('blocked')
    assert h.calls[0][1]['visible_objects']==[] and s['semantic_scene']['visible_objects']==[]
    assert s['pending']['issues'][0]['code']=='no_transfer_tasks' and h.backends==[]


def test_q2_capture_failure_never_falls_back_to_catalog_or_calls_provider(factory):
    h=factory();h.service.hardware_enabled=True;h.service.device_status=lambda:devices(True,False)
    def fail(*args):raise RuntimeError('controlled_capture_failure')
    h.runtime.scene_factory=fail
    code,s,_=h.http('/api/v2/runs',{'text':'收文具','semantic_source':'api','mode':'real'})
    assert code==201;h.run_id=s['run_id'];s=h.wait('failed')
    assert s['error']['reason']=='controlled_capture_failure' and h.calls==h.backends==[]


ROOT=Path(__file__).resolve().parents[1]
TOOLS=json.loads((ROOT/'tools.json').read_text(encoding='utf-8'))
WORLD=json.loads((ROOT/'world_entities.json').read_text(encoding='utf-8'))
OPTIONS={'tools':TOOLS,'world_entities':WORLD,'stations':[{'name':'右盒子','aliases':[]}]}


@pytest.mark.parametrize('reference',list(WORLD['entities']))
def test_q3_environment_reference_requires_answer_in_both_offline_and_real(reference):
    candidate={'tasks':[task(reference=reference)]}
    offline=PendingTaskPlanV2(candidate,original_text='受控指令',context_options=OPTIONS)
    assert offline.snapshot()['status']=='waiting_answer'
    assert offline.snapshot()['question']['code']=='unsupported_visual_reference'
    real=PendingTaskPlanV2(candidate,original_text='受控指令',context_options=OPTIONS,
        visual_perception_required=True,default_station='右盒子')
    s=real.snapshot();q=s['question']
    assert s['status']=='waiting_answer' and q['code']=='unsupported_visual_reference'
    assert len(q['choices'])==7 and set(q['choices'])==set(TOOLS)
    aliases=WORLD['entities'][reference].get('aliases') or [reference]
    assert aliases[0] in q['message'] and '系統相機僅支援定位 7 種桌面工具' in q['message']
    with pytest.raises(PendingTaskError):real.confirm(version=1)
    s=real.answer('__default_station__',version=1,task_index=1,slot='reference')
    assert s['status']=='waiting_confirmation'
    assert s['plan']['tasks'][0]['destination_name']=='右盒子' and s['plan']['tasks'][0]['reference'] is None


@pytest.mark.parametrize('slot',['reference','target'])
@pytest.mark.parametrize('visible',[None,[],['pen'],['pen','eraser','box']])
def test_q3_choices_only_yolo_tools_and_current_scene(slot,visible):
    candidate=task()|{slot:None}
    p=PendingTaskPlanV2({'tasks':[candidate]},original_text='受控指令',context_options=OPTIONS|{'visible_objects':visible})
    q=p.snapshot()['question'];expected=set(TOOLS) if visible is None else set(visible)&set(TOOLS)
    assert set(q['choices'])==expected
    if not expected:assert q['refresh_scene'] and '放入視野並重新取像' in q['message']


def test_q3_unknown_and_disabled_station_keep_original_name_and_enabled_choices(factory):
    h=factory(tasks=[task()|{'destination_type':'named','destination_name':'門口','reference':None,'relation':None}])
    s=h.create();q=s['pending']['question']
    assert s['status']=='waiting_answer' and q['choices']==['右盒子']
    assert q['message']=='⚠️ 無法解析出支援的放置區「門口」，請從下方選擇已啟用的放置站點：'
    h.action('cancel');h.station['effective_status']='disabled';h.tasks[0]['destination_name']='右盒子'
    s=h.create();assert s['pending']['question']['choices']==[] and '「右盒子」' in s['pending']['question']['message']


def test_q3_refresh_preserves_candidate_and_local_answers_use_new_scene(factory):
    h=factory(tasks=[task(reference='flashlight')]);h.service.hardware_enabled=True
    h.service.device_status=lambda:devices(True,False)
    s=create_real(h);assert s['pending']['question']['code']=='unsupported_visual_reference'
    original=s['pending']['candidate'];assert s['pending']['question']['choices']==['pen','eraser']
    h.case['objects']=[o for o in h.case['objects'] if o['class_name']=='eraser']
    assert h.action('refresh_scene',{'version':s['pending']['version']})[0]==200
    s=h.wait('waiting_answer');assert s['pending']['candidate']==original and s['pending']['version']==2
    assert s['pending']['question']['choices']==['eraser']  # requires a second distinct instance at execution
    assert len(h.calls)==1 and h.backends==[] and len(h.order)==2
    assert h.action('answer',{'version':1,'task_index':1,'slot':'reference','value':'pen'})[0]==400


def test_q3_offline_reference_is_rechecked_before_newly_connected_real_execution(factory):
    h=factory(tasks=[task(reference='flashlight')]);h.service.device_status=lambda:devices(False,False)
    create_real(h);h.service.hardware_enabled=h.service.motion_enabled=True
    h.service.device_status=lambda:devices()
    assert h.confirm()[1]['error']=='unsupported_visual_reference'
    assert h.runtime.state(h.run_id)['status']=='waiting_answer' and h.backends==[]


def test_q3_failed_refresh_can_retry_then_complete_without_stale_failed_run(factory):
    h=factory(tasks=[task(reference=None)]);s=h.create();scene=h.runtime.scene_factory
    def fail(*args):raise RuntimeError('controlled_refresh_failure')
    h.runtime.scene_factory=fail
    h.action('refresh_scene',{'version':1});s=h.wait('waiting_answer')
    assert s['error']['reason']=='controlled_refresh_failure' and h.runtime._runs[h.run_id]['status']!='failed'
    assert s['pending']['version']==1
    h.runtime.scene_factory=scene;h.action('refresh_scene',{'version':1});s=h.wait('waiting_answer')
    assert s['error'] is None and s['pending']['version']==2
    assert h.action('answer',{'value':'pen','version':2,'task_index':1,'slot':'reference'})[0]==200
    h.confirm();assert h.wait('completed','failed')['status']=='completed'


def test_q3_failed_resource_cleanup_quarantines_semantic_run(factory):
    h=factory(tasks=[task(reference=None)]);h.create();run=h.runtime._runs[h.run_id]
    run['cleanup_error']=['camera:OSError']
    assert h.runtime.owns_resources()
    assert h.action('refresh_scene',{'version':1})[1]['error']=='ownership_quarantined'
    assert h.action('answer',{'value':'pen','version':1,'task_index':1,'slot':'reference'})[1]['error']=='ownership_quarantined'
    assert h.action('cancel')[0]==200


def recovery_payload(state):
    q=state['sequence']['question']
    return {k:q[k] for k in ('run_id','step_id','frame_id','plan_version')}


def missing_second_step(h,role='target'):
    from dataclasses import replace
    observe=h.runtime._observe;control={'missing':True}
    def capture(run,step,**kwargs):
        obs=observe(run,step,**kwargs)
        if step==2 and control['missing'] and not kwargs.get('semantic_only'):
            name=h.tasks[1][role]
            instances=tuple(i for i in obs.instances if i['class_name']!=name)
            return replace(obs,instances=instances,display=obs.display|{'instances':list(instances)})
        return obs
    h.runtime._observe=capture
    return control


@pytest.mark.parametrize('role',['target','reference'])
@pytest.mark.parametrize('action',['retry_step','skip_step'])
def test_q4_missing_second_step_recovers_and_completes_later_steps(factory,role,action):
    h=factory(tasks=[task(),task(),task()]);control=missing_second_step(h,role)
    h.create();h.confirm();s=h.wait('waiting_step_recovery','failed')
    assert s['status']=='waiting_step_recovery' and s['sequence']['step_id']==2
    q=s['sequence']['question'];assert q['role']==role and q['safe_start_verified'] and q['empty_hand_confirmed']
    assert len([e for e in s['events'] if e['event']=='object_released_at_place'])==1
    payload=recovery_payload(s)
    assert h.action(action,payload|{'step_id':1})[0]==400
    control['missing']=False
    assert h.action(action,payload)[0]==200
    s=h.wait('completed','failed');assert s['status']=='completed',s
    expected=['completed','skipped' if action=='skip_step' else 'completed','completed']
    assert [step['status'] for step in s['sequence']['steps']]==expected
    assert [step for event,step in h.order if event=='capture']==([1,2,3,3] if action=='skip_step' else [1,2,2,3,3])
    assert len(h.calls)==1
    assert len([e for e in s['events'] if e['event']=='object_released_at_place'])==(2 if action=='skip_step' else 3)
    assert h.action(action,payload)[0]==400


def test_q4_real_pause_retains_lease_cancel_releases_without_later_commands(factory,monkeypatch):
    from test_v2_console_integration import native_real
    h=factory(tasks=[task(),task(),task()]);order=native_real(h,monkeypatch);missing_second_step(h)
    create_real(h);h.confirm();s=h.wait('waiting_step_recovery')
    run=h.runtime._runs[h.run_id]
    assert run['lease'] is not None and order.count('lease_acquired')-order.count('lease_released')==1 and not h.runtime._holding()
    backend=h.backends[-1];commands=len(backend.calls)
    assert h.action('cancel')[0]==200;h.wait('cancelled')
    assert run['lease'] is None and order.count('lease_acquired')==order.count('lease_released') and len(backend.calls)==commands


@pytest.mark.parametrize('missing_step',[1,2,3])
def test_q4_swap_dependency_steps_cannot_be_skipped(factory,missing_step):
    from dataclasses import replace
    tasks=[task('near'),task('at_initial',target='pen',reference='eraser'),task('at_initial')]
    h=factory(tasks=tasks);observe=h.runtime._observe
    execute=h.runtime._execute
    def move_fixture(run,prepared,**kwargs):
        result=execute(run,prepared,**kwargs)
        step=run['sequence'].snapshot()['step_id'];x,y=prepared['place'].place_pose_mm_deg[:2]
        # This scene uses fx=fy=600, depth=.5m and the frozen XY reflection.
        for obj in h.case['objects']:
            if obj['class_name']==tasks[step-1]['target']:obj['center_uv']=[320+1.2*x,240-1.2*y]
        return result
    h.runtime._execute=move_fixture
    def missing(run,step,**kwargs):
        obs=observe(run,step,**kwargs)
        if step==missing_step:
            # First frame must retain frozen anchors; reject just the current target later.
            if step==1:
                return replace(obs,instances=tuple(i for i in obs.instances if i['class_name']!='eraser'))
            return replace(obs,instances=tuple(i for i in obs.instances if i['class_name']!=tasks[step-1]['target']))
        return obs
    h.runtime._observe=missing;h.create();h.confirm();s=h.wait('waiting_step_recovery','failed')
    assert s['status']=='waiting_step_recovery',s
    assert s['sequence']['question']['skip_allowed'] is False
    assert h.action('skip_step',recovery_payload(s))[1]['error']=='swap_step_cannot_be_skipped'
    h.action('cancel')


@pytest.mark.parametrize('empty_confirmed',[False,True])
def test_q4_recovery_requires_verified_empty_start(empty_confirmed):
    from test_task_sequence_v2 import Harness,instance,task as sequence_task
    h=Harness([sequence_task(),sequence_task()],objects=[instance('b','battery')])
    h.seq.step_recovery_enabled=True
    h.seq.initialize_before_observe=lambda *a,**k:{'control_completed':True,'start_verified':True,'empty_hand_confirmed':empty_confirmed}
    h.begin();s=h.seq.advance()
    assert s['status']==('waiting_step_recovery' if empty_confirmed else 'failed') and h.commands==[]


def test_q4_skipping_final_step_finishes_without_dispatch(factory):
    h=factory(tasks=[task(),task()]);missing_second_step(h);h.create();h.confirm();s=h.wait('waiting_step_recovery')
    assert h.action('skip_step',recovery_payload(s))[0]==200
    s=h.wait('completed');assert [x['status'] for x in s['sequence']['steps']]==['completed','skipped']
    assert len([e for e in s['events'] if e['event']=='object_released_at_place'])==1



@pytest.mark.parametrize('a,b',[('eraser','pen'),('pen','eraser')])
def test_swap_completes_with_updated_scene_and_preserves_initial_positions(factory,a,b):
    import math
    import numpy as np
    tasks=[task('near',target=a,reference=b),task('at_initial',target=b,reference=a),task('at_initial',target=a,reference=b)]
    h=factory(tasks=tasks);h.cfg['orientation'].update(place_orientation_policy_relative='align_reference',place_orientation_policy_swap='match_initial')
    initial={o['class_name']:[(o['center_uv'][0]-320)/1.2,-(o['center_uv'][1]-240)/1.2] for o in h.case['objects']}
    execute=h.runtime._execute;moves=[]
    def move_scene(run,prepared,**kwargs):
        result=execute(run,prepared,**kwargs);step=run['sequence'].snapshot()['step_id']
        g=prepared['geometry'];x,y=g['destination_center_robot_xy_mm'];name=tasks[step-1]['target']
        moves.append((step,name,x,y))
        for obj in h.case['objects']:
            if obj['class_name']==name:
                obj['center_uv']=[320+1.2*x,240-1.2*y]
                poly=np.asarray(g['destination_footprint_robot_xy_mm']);edges=np.roll(poly,-1,axis=0)-poly
                v=edges[np.argmax(np.linalg.norm(edges,axis=1))];obj['angle_deg']=math.degrees(math.atan2(-v[1],v[0]))
        return result
    h.runtime._execute=move_scene;h.create();h.confirm();s=h.wait('completed','failed','waiting_step_recovery')
    assert s['status']=='completed',s['sequence']
    assert [x['status'] for x in s['sequence']['steps']]==['completed']*3
    final={o['class_name']:[(o['center_uv'][0]-320)/1.2,-(o['center_uv'][1]-240)/1.2] for o in h.case['objects']}
    assert final[a]==pytest.approx(initial[b],abs=1e-4) and final[b]==pytest.approx(initial[a],abs=1e-4)
    assert len(moves)==3 and len(h.calls)==1
    assert [step for event,step in h.order if event=='capture']==[1,2,3,3]
    assert s['completion_capture']['status']=='captured' and not s['outcome_verified']


@pytest.mark.parametrize('invalid_class',['eraser','pen'])
def test_swap_rejects_initial_anchor_that_is_reference_only(factory,invalid_class):
    from dataclasses import replace
    tasks=[task('near'),task('at_initial',target='pen',reference='eraser'),task('at_initial')]
    h=factory(tasks=tasks);observe=h.runtime._observe
    def capture(run,step,**kwargs):
        obs=observe(run,step,**kwargs)
        instances=tuple(i|{'eligible':False,'reference_eligible':True,'rejection_reason':'instance_outside_workspace'} if i['class_name']==invalid_class else i for i in obs.instances)
        return replace(obs,instances=instances)
    h.runtime._observe=capture;h.create();h.confirm();s=h.wait('waiting_step_recovery','failed')
    assert s['status']=='waiting_step_recovery'
    assert s['sequence']['question']['class_name']==invalid_class and not s['sequence']['question']['skip_allowed']
    assert not any(e['event']=='object_released_at_place' for e in s['events'])
