"""Unified 1/2/3-step lifecycle, selector roles and direct controller boundary."""
from copy import deepcopy
from threading import Event, Thread

import pytest

from operator_console.task_pending_v2 import PendingTaskError, PendingTaskPlanV2
from operator_console.task_sequence_v2 import ObservationV2, SequenceV2Error, TaskSequenceV2


OPTIONS={'tools':{'pen':{'aliases':['筆']},'battery':{'aliases':['電池']}},'world_entities':{},
    'stations':[{'name':'右盒子','aliases':[]}]}


def task(relative=False,target='pen'):
    return {'target':target,'destination_type':'relative' if relative else 'named','destination_name':None if relative else '右盒子','reference':'battery' if relative else None,'relation':'right_of' if relative else None}


def instance(i,cls,eligible=True):
    return {'instance_id':i,'class_name':cls,'eligible':eligible,'rejection_reason':'' if eligible else 'invalid_depth'}


class Harness:
    def __init__(self,tasks=None,policy='each_task',objects=None):
        self.now=1000;self.captures=[];self.commands=[];self.preparations=[]
        self.objects=objects or [instance('pen1','pen'),instance('bat1','battery')]
        self.pending=PendingTaskPlanV2({'tasks':tasks or [task()]},original_text='受控抓放計畫',context_options=OPTIONS)
        self.seq=TaskSequenceV2(self.pending,run_id='run1',observe=self.observe,prepare=self.prepare,execute=self.execute,
            initialization_policy=policy,clock_ms=lambda:self.now,max_scene_age_ms=10)
    def observe(self,step):
        self.captures.append(step)
        return ObservationV2(f'frame{len(self.captures)}',self.now,tuple(deepcopy(self.objects)),payload={'all_scene_objects':deepcopy(self.objects)})
    def prepare(self,task,obs,selected):
        self.preparations.append((task,obs,selected))
        return {'geometry_passed':True,'frame_id':obs.frame_id,'task':task,'selected':selected,'all_scene_objects':obs.payload['all_scene_objects']}
    def execute(self,prepared,*,initialize,checkpoint):
        checkpoint();self.commands.append({'initialize':initialize,'prepared':prepared})
        return {'control_completed':True,'ik_validated':False,'path_validated':False,'collision_validated_3d':False,'outcome_verified':False}
    def begin(self):
        self.pending.confirm(version=1);self.seq.begin(version=1)
    def select(self,chosen_id,**patch):
        q=self.seq.snapshot()['question']
        args={k:q[k] for k in ('run_id','step_id','frame_id','plan_version','role')}
        return self.seq.select(**(args|{'instance_id':chosen_id}|patch))


@pytest.mark.parametrize('count',[1,2,3,4,5,20])
@pytest.mark.parametrize('policy',['each_task','once'])
def test_unified_counts_observe_each_step_and_confirm_whole_plan_once(count,policy):
    h=Harness([task(i%2==1) for i in range(count)],policy=policy);h.begin()
    for _ in range(count):h.seq.advance()
    s=h.seq.snapshot();assert s['status']=='completed' and not s['outcome_verified']
    assert h.captures==list(range(1,count+1))
    assert [c['initialize'] for c in h.commands]==[policy=='each_task' or i==0 for i in range(count)]
    assert sum(x['event']=='plan_confirmed' for x in h.pending.snapshot()['history'])==1
    h.seq.advance();assert len(h.commands)==count


def test_unconfirmed_or_later_incomplete_plan_has_no_observation_or_commands():
    h=Harness([task(),task()|{'target':None}])
    with pytest.raises(PendingTaskError):h.seq.begin(version=1)
    assert h.captures==h.commands==[]


@pytest.mark.parametrize('count',[1,2,3,4,5,20])
def test_each_step_start_verified_before_new_frame_and_no_second_initialization(count):
    h=Harness([task() for _ in range(count)]);order=[]
    def initialize(step,*,checkpoint):
        checkpoint();order.append(('start',step));return {'control_completed':True,'start_verified':True}
    observe=h.observe
    def capture(step):order.append(('capture',step));return observe(step)
    h.seq.initialize_before_observe=initialize;h.seq.observe=capture;h.begin()
    for _ in range(count):h.seq.advance()
    assert order==[item for step in range(1,count+1) for item in [('start',step),('capture',step)]]
    assert not any(c['initialize'] for c in h.commands)
    assert h.seq.snapshot()['status']=='completed'


@pytest.mark.parametrize('result',[{}, {'control_completed':True}, {'control_completed':False,'start_verified':True}])
def test_unverified_start_stops_before_any_capture(result):
    h=Harness([task(),task()]);h.seq.initialize_before_observe=lambda *a,**k:result;h.begin();h.seq.advance()
    assert h.seq.snapshot()['status']=='failed' and h.captures==h.commands==[]


def test_refresh_reverifies_start_but_selection_does_not_repeat_start_command():
    h=Harness(objects=[instance('pen1','pen'),instance('pen2','pen')]);starts=[]
    def initialize(step,*,checkpoint):starts.append(step);return {'control_completed':True,'start_verified':True}
    h.seq.initialize_before_observe=initialize;h.begin();h.seq.advance();h.seq.refresh();h.seq.advance()
    h.select('pen2');h.seq.advance()
    assert starts==[1,1] and h.captures==[1,1] and len(h.commands)==1


def test_refresh_of_later_step_also_reverifies_start_with_once_policy():
    h=Harness([task(target='battery'),task()],policy='once',objects=[instance('bat','battery'),instance('pen1','pen'),instance('pen2','pen')]);starts=[]
    def initialize(step,*,checkpoint):starts.append(step);return {'control_completed':True,'start_verified':True}
    h.seq.initialize_before_observe=initialize;h.begin();h.seq.advance();h.seq.advance()
    assert h.seq.snapshot()['status']=='waiting_instance' and starts==[1]
    h.seq.refresh();h.seq.advance()
    assert starts==[1,2] and h.captures==[1,2,2]


def test_unconfirmed_plan_never_runs_before_capture_start_callback():
    h=Harness();starts=[]
    h.seq.initialize_before_observe=lambda *a,**k:starts.append(1)
    with pytest.raises(PendingTaskError):h.seq.begin(version=1)
    assert starts==[]


def test_duplicate_begin_cannot_claim_plan_twice():
    h=Harness();h.begin()
    with pytest.raises(SequenceV2Error):h.seq.begin(version=1)


def test_manual_target_and_reference_selection_preserves_every_occupancy_object():
    objects=[instance('pen1','pen'),instance('pen2','pen'),instance('bat1','battery'),instance('bat2','battery'),instance('obstacle','box')]
    h=Harness([task(True)],objects=objects);h.begin();h.seq.advance()
    assert h.seq.snapshot()['question']['role']=='target' and h.commands==[]
    h.select('pen2');h.seq.advance()
    assert h.seq.snapshot()['question']['role']=='reference' and h.commands==[]
    h.select('bat1');h.seq.advance()
    assert h.commands[0]['prepared']['selected']=={'target':'pen2','reference':'bat1'}
    assert len(h.commands[0]['prepared']['all_scene_objects'])==5
    assert h.captures==[1]


@pytest.mark.parametrize('patch',[{'run_id':'other'},{'step_id':2},{'step_id':True},{'frame_id':'old'},
    {'plan_version':0},{'plan_version':True},{'role':'reference'},{'instance_id':'bat1'}])
def test_stale_step_frame_run_version_or_role_selection_has_zero_commands(patch):
    h=Harness(objects=[instance('pen1','pen'),instance('pen2','pen'),instance('bat1','battery')]);h.begin();h.seq.advance()
    with pytest.raises(SequenceV2Error):h.select('pen1',**patch)
    assert h.commands==[]


def test_invalid_candidate_is_disabled_and_does_not_auto_select_only_valid_member():
    h=Harness(objects=[instance('pen1','pen',False),instance('pen2','pen')]);h.begin();h.seq.advance()
    assert h.seq.snapshot()['status']=='waiting_instance'
    with pytest.raises(SequenceV2Error):h.select('pen1')
    h.select('pen2');h.seq.advance();assert len(h.commands)==1


def test_double_selection_does_not_start_duplicate_commands():
    h=Harness(objects=[instance('pen1','pen'),instance('pen2','pen')]);h.begin();h.seq.advance()
    q=deepcopy(h.seq.snapshot()['question']);h.select('pen1')
    with pytest.raises(SequenceV2Error):h.seq.select(**{k:q[k] for k in ('run_id','step_id','frame_id','plan_version','role')},instance_id='pen1')
    h.seq.advance();h.seq.advance();assert len(h.commands)==1


def test_refresh_invalidates_old_frame_selection_and_roles():
    h=Harness(objects=[instance('pen1','pen'),instance('pen2','pen')]);h.begin();h.seq.advance()
    old=deepcopy(h.seq.snapshot()['question']);h.seq.refresh();h.seq.advance()
    assert h.seq.snapshot()['question']['frame_id']=='frame2'
    with pytest.raises(SequenceV2Error):h.seq.select(**{k:old[k] for k in ('run_id','step_id','frame_id','plan_version','role')},instance_id='pen1')
    assert h.commands==[]


def test_stale_selection_and_scene_after_preparation_never_move():
    h=Harness(objects=[instance('pen1','pen'),instance('pen2','pen')]);h.begin();h.seq.advance();h.now+=11
    with pytest.raises(SequenceV2Error,match='scene_snapshot_stale'):h.select('pen1')
    assert h.commands==[]
    h=Harness();original=h.prepare
    def late(*args):
        result=original(*args);h.now+=11;return result
    h.seq.prepare=late;h.begin();h.seq.advance()
    assert h.seq.snapshot()['error']['reason']=='scene_snapshot_stale' and h.commands==[]


@pytest.mark.parametrize('fault',['geometry','wrong_frame','duplicate_ids','invalid_depth','future_frame','missing_target'])
def test_geometry_and_observation_failures_stop_entire_remaining_plan(fault):
    h=Harness([task(),task()]);h.begin()
    if fault=='geometry':h.seq.prepare=lambda *a:{'geometry_passed':False,'frame_id':'frame1'}
    if fault=='wrong_frame':h.seq.prepare=lambda *a:{'geometry_passed':True,'frame_id':'old'}
    if fault=='duplicate_ids':h.objects=[instance('same','pen'),instance('same','pen')]
    if fault=='invalid_depth':h.objects=[instance('pen1','pen',False)]
    if fault=='future_frame':h.seq.observe=lambda step:ObservationV2('future',2000,tuple(h.objects))
    if fault=='missing_target':h.objects=[instance('bat1','battery')]
    h.seq.advance();h.seq.advance()
    assert h.seq.snapshot()['status']=='failed' and h.commands==[]
    assert h.seq.snapshot()['steps'][1]['status']=='pending'


@pytest.mark.parametrize('flag',['control_completed','ik_validated','path_validated','collision_validated_3d','outcome_verified'])
def test_invalid_controller_result_never_runs_next_step(flag):
    h=Harness([task(),task()]);original=h.execute
    def wrong(*a,**k):
        r=original(*a,**k);r[flag]=False if flag=='control_completed' else True;return r
    h.seq.execute=wrong;h.begin();h.seq.advance();h.seq.advance()
    assert h.seq.snapshot()['status']=='failed' and len(h.commands)==1


def test_cancel_waiting_or_during_callback_is_observable_and_never_retries():
    h=Harness([task(),task()]);entered=Event();release=Event()
    def blocking(prepared,*,initialize,checkpoint):
        entered.set();assert release.wait(5);checkpoint();pytest.fail('cancelled callback continued')
    h.seq.execute=blocking;h.begin();worker=Thread(target=h.seq.advance);worker.start();assert entered.wait(5)
    assert h.seq.snapshot()['worker_busy']
    h.seq.cancel();assert h.seq.snapshot()['status']=='cancelled'
    release.set();worker.join(5);assert not worker.is_alive();h.seq.advance()
    assert h.commands==[] and h.captures==[1]


def test_explicit_initialization_policy_is_required():
    with pytest.raises(SequenceV2Error,match='explicit_initialization_policy_required'):Harness(policy='guess')


def test_completion_and_next_step_boundary_preserve_latest_display():
    from dataclasses import replace
    h=Harness([task(),task()]);observe=h.observe
    def capture(step):
        obs=observe(step)
        return replace(obs,display={'frame_id':obs.frame_id,'image_url':f'/frame/{step}'})
    h.seq.observe=capture;h.begin();h.seq.advance()
    assert h.seq.snapshot()['display']['image_url']=='/frame/1'
    h.seq.advance();assert h.seq.snapshot()['display']['image_url']=='/frame/2'


@pytest.mark.parametrize('role',['target','reference'])
def test_measured_outside_object_can_only_be_selected_as_reference(role):
    outside=instance('bat1','battery',False)|{'reference_eligible':True,'reference_rejection_reason':''}
    tasks=[task(True)] if role=='reference' else [task(target='battery')]
    h=Harness(tasks,objects=[instance('pen1','pen'),outside]);h.begin();h.seq.advance()
    assert h.seq.snapshot()['status']==('completed' if role=='reference' else 'failed')
    assert len(h.commands)==(1 if role=='reference' else 0)


def test_multiple_references_use_reference_eligibility_for_manual_selection():
    h=Harness([task(True)],objects=[instance('pen1','pen'),instance('bat1','battery',False)|{'reference_eligible':True},instance('bat2','battery',False)|{'reference_eligible':False}])
    h.begin();h.seq.advance();q=h.seq.snapshot()['question']
    assert q['role']=='reference' and [i['eligible'] for i in q['choices']]==[True,False]
    with pytest.raises(SequenceV2Error,match='instance_not_eligible'):h.select('bat2')
    h.select('bat1');h.seq.advance();assert h.seq.snapshot()['status']=='completed'


def test_same_class_reference_can_be_outside_but_cannot_be_the_target_instance():
    h=Harness([task(True)|{'reference':'pen'}],objects=[instance('pen1','pen'),instance('pen2','pen',False)|{'reference_eligible':True}])
    h.pending=PendingTaskPlanV2({'tasks':[task(True)|{'reference':'pen'}]},original_text='兩支筆',context_options=OPTIONS,instance_selection_available=True)
    h.seq.pending=h.pending
    h.begin();h.seq.advance();h.select('pen1');h.seq.advance()
    q=h.seq.snapshot()['question'];assert q['role']=='reference'
    assert [i['eligible'] for i in q['choices']]==[False,True]
    h.select('pen2');h.seq.advance();assert h.seq.snapshot()['status']=='completed'


@pytest.mark.parametrize('mode,fallback,claimed_ik',[
    ('preferred_ik',None,True),('cartesian','cartesian',False),
    ('fallback_cartesian',None,True),('standard_ik','standard_ik',False),('taught_joint',None,False)])
def test_completed_transfer_with_vertical_evidence_advances_two_steps(mode,fallback,claimed_ik):
    h=Harness([task(),task()]);original=h.execute
    def execute(*a,**k):
        result=original(*a,**k)
        result.update(ik_validated=claimed_ik,transfer_evidence=[
            {'transfer_mode':mode,'fallback_mode':fallback,'ik_validated':mode=='preferred_ik','motion_completed':True},
            {'record_type':'vertical_motion','ik_validated':False,'motion_completed':True}])
        return result
    h.seq.execute=execute;h.begin();h.seq.advance();h.seq.advance()
    state=h.seq.snapshot()
    assert state['status']=='completed' and len(h.commands)==2
    assert all(step['result']['ik_validated']==(mode=='preferred_ik') for step in state['steps'])


@pytest.mark.parametrize('record',[None,{}, {'transfer_mode':'cartesian','motion_completed':False},
    {'transfer_mode':'unknown','fallback_used':True,'motion_completed':True}])
def test_malformed_or_incomplete_transfer_evidence_cannot_release_next_step(record):
    h=Harness([task(),task()]);original=h.execute
    def execute(*a,**k):
        return {**original(*a,**k),'transfer_evidence':[record]}
    h.seq.execute=execute;h.begin();h.seq.advance();h.seq.advance()
    assert h.seq.snapshot()['status']=='failed' and len(h.commands)==1


@pytest.mark.parametrize('policy',['segmented_mode0','linear_mode1','single_mode0'])
@pytest.mark.parametrize('route',['preferred_ik','cartesian'])
def test_real_controller_contract_completes_two_steps_in_all_modes(policy,route):
    from test_direct_control_v2 import controller,CONFIG,plans
    cfg=deepcopy(CONFIG);cfg['robot'].update(vertical_motion_mode=policy,vertical_segment_step_mm=35.)
    c,b,_,_=controller(config=cfg)
    if route=='preferred_ik':c.select_preferred_ik=lambda *a,**k:[0,0,0,-90,0,-45]
    h=Harness([task(),task()])
    h.seq.execute=lambda *a,**k:c.execute_pick_and_place(*plans(),geometry_passed=True,initialize=False)
    h.begin();h.seq.advance();h.seq.advance()
    state=h.seq.snapshot()
    assert state['status']=='completed',state['error']
    assert sum(x==('gripper',1) for x in b.calls)==2
    assert all(step['result']['ik_validated']==(route=='preferred_ik') for step in state['steps'])
