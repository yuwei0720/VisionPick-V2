"""Whole-plan state tests; no live model or physical controller."""
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor

import pytest

from operator_console.task_pending_v2 import PendingTaskError, PendingTaskPlanV2


OPTIONS = {"tools": {"pen": {"aliases": ["筆"]}, "battery": {"aliases": ["電池"]}},
           "world_entities": {"box": {"aliases": ["盒子"]}},
           "stations": [{"name": "右盒子", "aliases": ["右邊盒子"]}, {"name": "左盒子", "aliases": ["左邊盒子"]}]}


def item(**patch):
    value = {"target": "pen", "destination_type": "named", "destination_name": "右盒子", "reference": None, "relation": None}
    return value | patch


def session(tasks=None, **options):
    return PendingTaskPlanV2({"tasks": tasks if tasks is not None else [item()]},
        original_text="把筆放到右盒子", context_options=OPTIONS, **options)


def reply(pending, value):
    s = pending.snapshot(); q = s["question"]
    return pending.answer(value, version=s["version"], task_index=q["task_index"], slot=q["slot"])


def test_ready_requires_confirmation_and_claim_is_once_for_whole_plan():
    p = session([item(), item(target="battery", destination_name="左盒子")])
    assert p.snapshot()["status"] == "waiting_confirmation"
    with pytest.raises(PendingTaskError): p.claim(version=1)
    p.confirm(version=1)
    claimed = p.claim(version=1)
    assert len(claimed["plan"]["tasks"]) == 2 and not claimed["authorizes_motion"]
    with pytest.raises(PendingTaskError): p.claim(version=1)
    assert sum(x["event"] == "plan_confirmed" for x in claimed["history"]) == 1
    assert sum(x["event"] == "execution_claimed" for x in claimed["history"]) == 1


def test_later_missing_slot_never_exposes_ready_prefix():
    p = session([item(), item(target=None)])
    assert p.snapshot()["plan"] is None
    assert p.snapshot()["question"]["task_index"] == 2
    for operation in (p.confirm, p.claim):
        with pytest.raises(PendingTaskError): operation(version=1)


@pytest.mark.parametrize("count", [0, 21, 22])
def test_over_limit_or_no_task_cannot_be_confirmed(count):
    p = session([item()] * count)
    assert p.snapshot()["status"] == "blocked"
    assert len(p.snapshot()["candidate"]["tasks"]) == count
    with pytest.raises(PendingTaskError): p.confirm(version=1)


def test_global_question_priority_and_only_requested_slot_changes():
    first = item(destination_type=None, destination_name=None)
    second = item(target=None)
    p = session([first, second])
    assert p.snapshot()["question"]["slot"] == "target"
    before = deepcopy(p.snapshot()["candidate"])
    after = reply(p, "電池")
    assert after["candidate"]["tasks"][0] == before["tasks"][0]
    assert after["candidate"]["tasks"][1] == before["tasks"][1] | {"target": "battery"}
    assert after["question"]["task_index"] == 1
    assert after["initial_candidate"]["tasks"][1]["target"] is None


def test_destination_answer_then_reference_then_relation_has_no_provider_calls(monkeypatch):
    from google import genai
    monkeypatch.setattr(genai, "Client", lambda *a, **k: pytest.fail("provider client created"))
    p = session([item(destination_type=None, destination_name=None)])
    assert reply(p, "relative")["question"]["slot"] == "reference"
    assert reply(p, "電池")["question"]["slot"] == "relation"
    assert reply(p, "右邊")["status"] == "waiting_confirmation"
    assert p.snapshot()["plan"]["tasks"][0]["reference"] == "battery"


def test_named_destination_answer_can_use_unique_alias():
    p = session([item(destination_type=None, destination_name=None)])
    result = reply(p, "左邊盒子")
    assert result["plan"]["tasks"][0]["destination_name"] == "左盒子"
    assert result["plan"]["tasks"][0]["destination_type"] == "named"


def test_unknown_explicit_destination_never_becomes_selected_default():
    p = session([item(destination_name="門口")], mode="fixed", default_station="右盒子")
    assert p.snapshot()["candidate"]["tasks"][0]["destination_name"] == "門口"
    assert p.snapshot()["defaults"] == []
    assert reply(p, "左盒子")["status"] == "waiting_confirmation"


@pytest.mark.parametrize("mode,default,expected", [("strict", "右盒子", "waiting_answer"),
    ("fixed", "不存在", "waiting_answer"), ("fixed", None, "waiting_answer"), ("fixed", "右邊盒子", "waiting_confirmation")])
def test_default_only_in_fixed_mode_with_registered_station(mode, default, expected):
    p = session([item(destination_type=None, destination_name=None)], mode=mode, default_station=default)
    s = p.snapshot(); assert s["status"] == expected
    if expected == "waiting_confirmation":
        assert "未指定目的地" in s["defaults"][0]["message"] and "請確認" in s["defaults"][0]["message"]
        with pytest.raises(PendingTaskError): p.claim(version=s["version"])
    else: assert s["defaults"] == []


def test_partial_explicit_relative_fields_never_get_defaulted():
    p = session([item(destination_type=None, destination_name=None, reference="battery")], mode="fixed", default_station="右盒子")
    assert p.snapshot()["status"] == "blocked" and p.snapshot()["defaults"] == []


def test_ambiguous_alias_rejects_answer_without_overwriting_any_slot():
    options = deepcopy(OPTIONS)
    options["stations"][0]["aliases"].append("盒位")
    options["stations"][1]["aliases"].append("盒位")
    p = PendingTaskPlanV2({"tasks": [item(destination_name=None)]}, original_text="把筆送走", context_options=options)
    before = p.snapshot()["candidate"]
    with pytest.raises(PendingTaskError, match="answer_not_unique"): reply(p, "盒位")
    assert p.snapshot()["candidate"] == before and p.snapshot()["version"] == 1
    assert p.snapshot()["history"][-1]["answer"] == "盒位"


@pytest.mark.parametrize("task_index,slot,version", [(2,"target",1),(1,"reference",1),(1,"target",0),(1,"target",True)])
def test_stale_or_wrong_role_answer_is_rejected(task_index,slot,version):
    p = session([item(target=None)])
    with pytest.raises(PendingTaskError): p.answer("筆",version=version,task_index=task_index,slot=slot)
    assert p.snapshot()["candidate"]["tasks"][0]["target"] is None


def test_settings_edit_invalidates_confirmation_and_freezes_detached_values():
    p = session(settings={"pose_verify_stable_polls":3});p.confirm(version=1)
    changed={"pose_verify_stable_polls":4}; p.set_settings(changed,version=1);changed["pose_verify_stable_polls"]=99
    assert p.snapshot()["status"]=="waiting_confirmation" and p.snapshot()["confirmed_version"] is None
    with pytest.raises(PendingTaskError): p.claim(version=1)
    p.confirm(version=2);claimed=p.claim(version=2)
    assert claimed["settings"]["pose_verify_stable_polls"]==4
    with pytest.raises(PendingTaskError): p.set_settings({},version=2)


def test_replacement_requires_new_confirmation_and_never_merges_dialogue():
    p=session();p.confirm(version=1)
    p.replace({"tasks":[item(destination_name="左盒子")]},original_text="改放到左盒子",version=1)
    assert p.snapshot()["original_text"]=="改放到左盒子" and p.snapshot()["confirmed_version"] is None
    assert p.snapshot()["history"][0]["original_text"] == "把筆放到右盒子"
    with pytest.raises(PendingTaskError):p.claim(version=2)
    p.confirm(version=2); assert p.claim(version=2)["plan"]["tasks"][0]["destination_name"]=="左盒子"


@pytest.mark.parametrize("state",["waiting_answer","waiting_confirmation","confirmed"])
def test_cancel_closes_every_pre_execution_state(state):
    p=session([item(target=None)] if state=="waiting_answer" else None)
    if state=="confirmed":p.confirm(version=1)
    p.cancel();p.cancel()
    assert p.snapshot()["status"]=="cancelled"
    for operation in (p.confirm,p.claim):
        with pytest.raises(PendingTaskError):operation(version=1)


def test_timeout_closes_pending_without_execution():
    now=[100.0];p=session(clock=lambda:now[0],timeout_s=2)
    p.confirm(version=1);now[0]=102
    assert p.snapshot()["status"]=="expired" and p.snapshot()["confirmed_version"] is None
    with pytest.raises(PendingTaskError):p.claim(version=1)


def test_duplicate_concurrent_claims_handoff_only_once():
    p=session();p.confirm(version=1)
    def claim(_):
        try:p.claim(version=1);return True
        except PendingTaskError:return False
    with ThreadPoolExecutor(max_workers=4) as pool:results=list(pool.map(claim,range(8)))
    assert sum(results)==1


def test_same_class_reference_is_saved_for_instance_selection_not_slot_overwrite():
    p=session([item(destination_type="relative",destination_name=None,reference="pen",relation="right_of")])
    assert p.snapshot()["question"]["kind"]=="instance_selection_required"
    with pytest.raises(PendingTaskError,match="instance_selection_required"):reply(p,"筆")


@pytest.mark.parametrize("patch",[{"mode":"bad"},{"timeout_s":0},{"timeout_s":True},{"timeout_s":float('inf')},{"settings":[]},{"settings":{"n":float('nan')}}])
def test_invalid_control_inputs_are_not_silently_clamped(patch):
    with pytest.raises((PendingTaskError,ValueError)):session(**patch)


def test_snapshots_are_detached_and_finish_requires_started_plan():
    p=session();s=p.snapshot();s["candidate"]["tasks"][0]["target"]="battery"
    assert p.snapshot()["candidate"]["tasks"][0]["target"]=="pen"
    with pytest.raises(PendingTaskError):p.finish(outcome="completed")
    p.confirm(version=1);p.claim(version=1);assert p.finish(outcome="completed")["status"]=="completed"
    with pytest.raises(PendingTaskError):p.claim(version=1)


def test_same_class_can_defer_identity_only_to_an_available_selector():
    same=item(destination_type="relative",destination_name=None,reference="pen",relation="right_of")
    p=session([same],instance_selection_available=True)
    s=p.snapshot()
    assert s["status"]=="waiting_confirmation" and s["plan"]["tasks"][0]==same|{'gap_mm':None,'displacement_mm':None}
    assert s["instance_constraints"][0]["code"]=="same_entity_reference_requires_disambiguation"
    assert not s["authorizes_motion"]
    p.confirm(version=1);assert p.claim(version=1)["instance_constraints"]


def test_instance_deferral_does_not_clear_other_missing_or_unsupported_slots():
    same=item(destination_type="relative",destination_name=None,reference="pen",relation="right_of")
    p=session([same,item(target=None)],instance_selection_available=True)
    assert p.snapshot()["status"]=="waiting_answer" and p.snapshot()["question"]["slot"]=="target"
    with pytest.raises(PendingTaskError):p.confirm(version=1)
    p=session([same|{"relation":"on_top_of"}],instance_selection_available=True)
    assert p.snapshot()["status"]=="blocked" and p.snapshot()["plan"] is None


@pytest.mark.parametrize("capability",[1,"yes",None])
def test_instance_selection_capability_requires_explicit_boolean(capability):
    with pytest.raises(PendingTaskError):session(instance_selection_available=capability)


def test_same_class_selector_never_accepts_same_physical_instance():
    from operator_console.task_sequence_v2 import ObservationV2,SequenceV2Error,TaskSequenceV2
    same=item(destination_type="relative",destination_name=None,reference="pen",relation="right_of")
    p=session([same],instance_selection_available=True);p.confirm(version=1);commands=[]
    seq=TaskSequenceV2(p,run_id="same_class",initialization_policy="once",clock_ms=lambda:1000,
        observe=lambda step:ObservationV2("frame1",1000,tuple({"instance_id":i,"class_name":"pen","eligible":True} for i in ("pen1","pen2"))),
        prepare=lambda *a:{"frame_id":"frame1","geometry_passed":True},execute=lambda *a,**k:commands.append(a))
    seq.begin(version=1);seq.advance();q=seq.snapshot()["question"]
    seq.select(**{k:q[k] for k in ("run_id","step_id","frame_id","plan_version","role")},instance_id="pen1")
    seq.advance();q=seq.snapshot()["question"]
    assert q["role"]=="reference"
    with pytest.raises(SequenceV2Error):seq.select(**{k:q[k] for k in ("run_id","step_id","frame_id","plan_version","role")},instance_id="pen1")
    assert commands==[]



def test_placement_angle_is_in_consent_digest_and_claimed_step_metadata():
    p=session();before=p.snapshot();p.confirm(version=before['version'])
    changed=p.set_settings({'placement_orientation_overrides':{'1':{'policy':'fixed_deg','rz_deg':72.}}},version=before['version'])
    assert changed['status']=='waiting_confirmation' and changed['version']==before['version']+1
    with pytest.raises(PendingTaskError):p.claim(version=before['version'])
    from operator_console.task_sequence_v2 import TaskSequenceV2
    p.confirm(version=changed['version'])
    seq=TaskSequenceV2(p,run_id='angles',observe=lambda _:None,prepare=lambda *a:None,execute=lambda *a:None,initialization_policy='once')
    snap=seq.begin(version=changed['version'])
    assert snap['steps'][0]['orientation_override']=={'policy':'fixed_deg','rz_deg':72.}
    with pytest.raises(PendingTaskError):p.set_settings({},version=changed['version'])
