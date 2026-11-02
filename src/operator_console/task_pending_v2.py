"""Versioned, local slot filling for a whole V2 plan; no provider or robot calls."""
from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import math
import threading
import time
import unicodedata

from .task_parser_v2 import CORE_SLOTS, _context, _validate
from .task_schema_v2 import SELF_RELATIONS, TaskPlanV2


class PendingTaskError(ValueError):
    pass


def _copy(value):
    """Detach JSON control data and reject non-finite or non-JSON settings."""
    return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))


class PendingTaskPlanV2:
    """Keep the original candidate, local patches and one whole-plan confirmation.

    A claim is a one-use handoff to the later execution consumer. It never
    authorizes robot motion by itself; geometry, instance and controller checks
    remain the consumer's responsibility. No ready prefix is ever returned.
    """

    _PRIORITY = {"target": 10, "destination_type": 20, "destination_basis": 20, "reference": 30,
                 "relation": 30, "destination_name": 40, "displacement_mm": 40}
    _TERMINAL = {"cancelled", "expired", "completed", "failed"}

    def __init__(self, payload, *, original_text, context_options=None, mode="strict",
                 default_station=None, settings=None, timeout_s=180.0, clock=time.monotonic,
                 instance_selection_available=False, visual_perception_required=False):
        if mode not in {"strict", "fixed"}:
            raise PendingTaskError("invalid_destination_mode")
        if not isinstance(original_text, str) or not original_text.strip():
            raise PendingTaskError("input_text_must_not_be_empty")
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise PendingTaskError("pending_timeout_must_be_finite_positive")
        if settings is not None and not isinstance(settings, dict):
            raise PendingTaskError("settings_must_be_object")
        if type(instance_selection_available) is not bool:
            raise PendingTaskError("instance_selection_capability_must_be_boolean")
        if type(visual_perception_required) is not bool:
            raise PendingTaskError('visual_perception_capability_must_be_boolean')
        self._lock = threading.RLock()
        self._clock = clock
        self._deadline = clock() + float(timeout_s)
        self._context = _copy(_context(**(context_options or {})))
        self._initial = _copy(payload)
        self._candidate = _copy(payload)
        self._original_text = original_text
        self._settings = _copy(settings or {})
        self._mode = mode
        self._instance_selection_available = instance_selection_available
        self._visual_perception_required = visual_perception_required
        self._instance_constraints = []
        self._default_station = default_station
        self._version = 1
        self._confirmed_version = None
        self._confirmed_digest = None
        self._history = []
        self._defaults = []
        self._status = "pending"
        self._result = None
        self._question = None
        self._record("candidate_received", original_text=original_text, candidate=self._initial)
        self._apply_defaults()
        self._refresh()

    def _record(self, event, **data):
        self._history.append({"event": event, "version": self._version, **_copy(data)})

    def _resolve(self, value, slot):
        if slot == 'displacement_mm':
            if isinstance(value, bool) or not isinstance(value, (str, int, float)):
                raise PendingTaskError('displacement_must_be_positive_finite_mm')
            try:
                distance = float(value)
            except (ValueError, OverflowError):
                raise PendingTaskError('displacement_must_be_positive_finite_mm') from None
            if not math.isfinite(distance) or distance <= 0:
                raise PendingTaskError('displacement_must_be_positive_finite_mm')
            return distance
        if not isinstance(value, str) or not value.strip():
            raise PendingTaskError("answer_must_be_registered_option")
        normalize = lambda text: unicodedata.normalize("NFKC", text).strip().casefold()
        token = normalize(value)
        if slot == "destination_name":
            entries = [(x["name"], x["aliases"]) for x in self._context["stations"]]
        elif slot in {"relation", 'self_relation'}:
            names = SELF_RELATIONS if slot == 'self_relation' else self._context['supported_relations']
            entries = [(name, self._context["relation_terms"][name]) for name in names]
        else:
            registry = self._context["objects" if slot == "target" else "references"]
            entries = [(name, entry["aliases"]) for name, entry in registry.items()]
        matches = {name for name, aliases in entries if token in {normalize(x) for x in [name, *aliases]}}
        if len(matches) != 1:
            raise PendingTaskError("answer_not_unique" if matches else "answer_not_registered")
        return next(iter(matches))

    def _apply_defaults(self):
        if self._mode != "fixed" or self._default_station is None:
            return
        try:
            name = self._resolve(self._default_station, "destination_name")
        except PendingTaskError:
            return  # Keep the missing destination; no guessing or silent fallback.
        tasks = self._candidate.get("tasks") if isinstance(self._candidate, dict) else None
        if not isinstance(tasks, list) or not 1 <= len(tasks) <= self._context["max_tasks"]:
            return
        for index, item in enumerate(tasks, 1):
            if (not isinstance(item, dict) or not set(CORE_SLOTS) <= set(item)
                    or set(item) - set(CORE_SLOTS) - {"gap_mm", "displacement_mm"}
                    or item.get("gap_mm") is not None
                    or item.get('displacement_mm') is not None
                    or any(item[s] is not None and (not isinstance(item[s], str) or item[s].strip())
                           for s in CORE_SLOTS[1:])):
                continue
            patch = {"destination_type": "named", "destination_name": name}
            item.update(patch)
            notice = {"task_index": index, "destination_name": name,
                      "message": f"第 {index} 項未指定目的地，將放到{name}，請確認。"}
            self._defaults.append(notice)
            self._record("default_applied", task_index=index, patch=patch, source="selected_default")

    def _refresh(self):
        self._result = _validate(self._candidate, self._context)
        self._instance_constraints = []
        # The frozen parser still reports a class-level self-reference as clarify.
        # An actual selector consumer resolves the distinct physical instances;
        # it must not force the operator to invent a different five-slot class.
        if self._instance_selection_available and self._result.status == "clarify":
            deferred = [i for i in self._result.issues if i["code"] == "same_entity_reference_requires_disambiguation"]
            if deferred:
                remaining = tuple(i for i in self._result.issues if i not in deferred)
                self._instance_constraints = _copy(deferred)
                if remaining:
                    self._result = replace(self._result, issues=remaining)
                else:
                    normalized = [{k: v.strip() or None if isinstance(v, str) else v for k, v in item.items()}
                                  for item in self._candidate["tasks"]]
                    self._result = replace(self._result, status="ready", issues=(), plan=TaskPlanV2.model_validate({"tasks": normalized}))
        self._question = None
        if self._result.status == "ready":
            self._status = "waiting_confirmation"
        elif self._result.status == "blocked":
            self._status = "blocked"
        else:
            self._status = "waiting_answer"
            issue = min(self._result.issues, key=lambda x: (self._PRIORITY.get(x.get("slot"), 0), x.get("task_index", 0)))
            slot = issue["slot"]
            choices = self._choices(slot, issue['task_index'])
            wording = {"target": "要抓取哪個物件？", "destination_type": "要放到哪個站點，或使用相對位置？",
                       "destination_name": "請選擇有效的放置站點。", "reference": "要以哪個物件作為參考？",
                       "relation": "要往哪個方向？", 'displacement_mm': '要移動幾毫米？（請輸入大於 0 的數值）',
                       'destination_basis': '請問是依自身原位往此方向平移，還是放到另一物件的該方向？'}
            self._question = {**deepcopy(issue), "version": self._version, "choices": choices,
                              "message": f"第 {issue['task_index']} 項：{wording[slot]}"}
            task = self._candidate['tasks'][issue['task_index']-1]
            if slot == 'displacement_mm':
                self._question.update(input_type='number', unit='mm', exclusive_minimum=0)
            if slot == 'destination_basis':
                direction = {'left_of':'左','right_of':'右','in_front_of':'前','behind':'後'}[task['relation']]
                self._question['message'] = f'第 {issue["task_index"]} 項：請問是依自身原位往{direction}平移，還是放到另一物件{direction}邊？'
                self._question['alternatives'] = [{'value':'__self_relative__','label':'依自身原位平移'}]
            if issue['code'] == 'destination_name_not_registered':
                self._question['message'] = f"⚠️ 無法解析出支援的放置區「{task['destination_name']}」，請從下方選擇已啟用的放置站點："
            if issue['code'] == 'unsupported_visual_reference':
                ref = task['reference']; aliases = self._context['semantic_entities'][ref]['aliases']
                name = aliases[0] if aliases else ref
                self._question['message'] = f'⚠️ 無法解析出支援的視覺參照物「{name}」（系統相機僅支援定位 7 種桌面工具與具名放置區），請改選畫面中支援的參照物或放置站點：'
                if self._default_station in {s['name'] for s in self._context['stations']}:
                    self._question['alternatives'] = [{'value':'__default_station__','label':'改放預設具名站點（'+self._default_station+'）'}]
                for station in self._context['stations']:
                    if station['name'] != self._default_station:
                        self._question.setdefault('alternatives',[]).append({'value':'__station__:'+station['name'],
                            'label':'改放具名站點（'+station['name']+'）','destination_name':station['name']})
            if slot in {'reference','target','destination_basis'} and not choices and self._context['visible_objects'] is not None:
                self._question['message'] += ' 請先將'+('參照物' if slot=='reference' else '目標物')+'放入視野並重新取像。'
                self._question['refresh_scene'] = True
            if issue["code"] == "same_entity_reference_requires_disambiguation":
                self._question["kind"] = "instance_selection_required"
            else:
                self._question["kind"] = "slot"

    def _choices(self, slot, task_index=None):
        if slot == "destination_type":
            return [x["name"] for x in self._context["stations"]] + ["relative", 'self_relative']
        if slot == "destination_name":
            return [x["name"] for x in self._context["stations"]]
        if slot == "relation":
            task = self._candidate['tasks'][task_index-1] if task_index else {}
            return list(SELF_RELATIONS if task.get('destination_type') == 'self_relative' else self._context["supported_relations"])
        if slot == 'displacement_mm':
            return [30, 50, 80]
        registry = self._context['objects' if slot == 'target' else 'references']
        choices = [name for name,item in registry.items() if item['visual_locatable'] and (slot != 'target' or item['graspable'])]
        visible = self._context['visible_objects']
        if visible is not None:
            choices = [name for name in choices if name in visible]
            # Same-class references remain distinct-instance requests; the
            # sequence requires two eligible instances and operator selection.
        return choices

    def update_visible_objects(self, objects, *, visual_perception_required):
        with self._lock:
            self._expire()
            if self._status != 'waiting_answer': raise PendingTaskError('not_waiting_for_answer')
            if (not isinstance(objects,list) or any(not isinstance(x,str) or not x.strip() for x in objects)
                    or type(visual_perception_required) is not bool):
                raise PendingTaskError('invalid_visible_objects')
            self._context['visible_objects'] = list(dict.fromkeys(objects))
            self._visual_perception_required = visual_perception_required
            self._version += 1
            self._confirmed_version = self._confirmed_digest = None
            self._record('visual_scene_refreshed',visible_objects=objects)
            self._refresh()
            return self.snapshot()

    def require_visual_perception(self):
        with self._lock:
            self._expire()
            if self._status in self._TERMINAL or self._status == 'executing':
                raise PendingTaskError('pending_session_closed')
            if not self._visual_perception_required:
                self._visual_perception_required = True
                self._refresh()
                if self._status == 'waiting_answer':
                    self._version += 1
                    self._confirmed_version = self._confirmed_digest = None
                    self._refresh()
            return self.snapshot()

    def _expire(self):
        if self._status not in self._TERMINAL and self._status != "executing" and self._clock() >= self._deadline:
            self._status = "expired"
            self._confirmed_version = self._confirmed_digest = None
            self._question = None
            self._record("expired")

    def _check_version(self, version):
        self._expire()
        if type(version) is not int or version != self._version:
            raise PendingTaskError("stale_plan_version")
        if self._status in self._TERMINAL:
            raise PendingTaskError("pending_session_closed")

    def _digest(self):
        data = {"candidate": self._candidate, "settings": self._settings, "context": self._context}
        if self._visual_perception_required: data['visual_perception_required'] = True
        return hashlib.sha256(json.dumps(data, sort_keys=True, ensure_ascii=False, allow_nan=False).encode()).hexdigest()

    def snapshot(self):
        with self._lock:
            self._expire()
            return {"version": self._version, "status": self._status,
                    "original_text": self._original_text, "initial_candidate": _copy(self._initial),
                    "candidate": _copy(self._candidate), "question": _copy(self._question),
                    "issues": _copy(list(self._result.issues)), "defaults": _copy(self._defaults),
                    "settings": _copy(self._settings), "history": _copy(self._history),
                    "plan": self._result.plan.model_dump() if self._result.plan is not None else None,
                    "confirmed_version": self._confirmed_version, "authorizes_motion": False,
                    "instance_selection_available": self._instance_selection_available,
                    "instance_constraints": _copy(self._instance_constraints)}

    def answer(self, value, *, version, task_index, slot):
        with self._lock:
            self._check_version(version)
            q = self._question
            if self._status != "waiting_answer" or q is None:
                raise PendingTaskError("not_waiting_for_answer")
            if type(task_index) is not int or task_index != q["task_index"] or slot != q["slot"]:
                raise PendingTaskError("answer_does_not_match_question")
            if q["kind"] != "slot":
                raise PendingTaskError("instance_selection_required")
            try:
                station_option = next((a for a in q.get('alternatives',[]) if a['value']==value),None)
                if slot == 'destination_basis':
                    if value == '__self_relative__':
                        patch = {'destination_type':'self_relative','reference':None,'destination_name':None,
                                 'gap_mm':None,'displacement_mm':None}
                    else:
                        patch = {'destination_type':'relative','reference':self._resolve(value,'reference'),
                                 'destination_name':None,'gap_mm':None,'displacement_mm':None}
                elif slot == 'reference' and station_option is not None:
                    station = self._default_station if value=='__default_station__' else station_option['destination_name']
                    patch = {'destination_type':'named','destination_name':self._resolve(station,'destination_name'),
                             'reference':None,'relation':None,'gap_mm':None,'displacement_mm':None}
                elif slot == "destination_type":
                    kind = unicodedata.normalize('NFKC',value).strip().casefold() if isinstance(value,str) else ''
                    if kind in {'relative','self_relative'}:
                        patch = {'destination_type':kind,'destination_name':None,'reference':None,'gap_mm':None,'displacement_mm':None}
                    else:
                        patch = {'destination_type':'named','destination_name':self._resolve(value,'destination_name'),
                                 'reference':None,'relation':None,'gap_mm':None,'displacement_mm':None}
                else:
                    resolver_slot = 'self_relation' if slot=='relation' and self._candidate['tasks'][task_index-1].get('destination_type')=='self_relative' else slot
                    patch = {slot: self._resolve(value, resolver_slot)}
            except PendingTaskError as exc:
                try:
                    rejected = _copy(value)
                except (ValueError, TypeError, OverflowError):
                    rejected = repr(value)
                self._record("answer_rejected", task_index=task_index, slot=slot, answer=rejected, reason=str(exc))
                raise
            self._candidate["tasks"][task_index - 1].update(patch)
            self._version += 1
            self._confirmed_version = self._confirmed_digest = None
            self._record("slot_patched", task_index=task_index, slot=slot, answer=value, patch=patch)
            self._refresh()
            return self.snapshot()

    def replace(self, payload, *, original_text, version):
        """A fresh whole instruction replaces tasks; no structural slot patch."""
        with self._lock:
            self._check_version(version)
            if self._status == "executing":
                raise PendingTaskError("plan_already_executing")
            if not isinstance(original_text, str) or not original_text.strip():
                raise PendingTaskError("input_text_must_not_be_empty")
            candidate = _copy(payload)
            self._version += 1
            self._confirmed_version = self._confirmed_digest = None
            self._candidate = candidate
            self._original_text = original_text
            self._defaults = []
            self._record("instruction_replaced", original_text=original_text, candidate=candidate)
            self._apply_defaults()
            self._refresh()
            return self.snapshot()

    def set_settings(self, settings, *, version):
        with self._lock:
            self._check_version(version)
            if self._status == "executing":
                raise PendingTaskError("plan_already_executing")
            if not isinstance(settings, dict):
                raise PendingTaskError("settings_must_be_object")
            settings = _copy(settings)
            if settings != self._settings:
                self._settings = settings
                self._version += 1
                self._confirmed_version = self._confirmed_digest = None
                self._record("settings_changed", settings=settings)
                self._refresh()
            return self.snapshot()

    def confirm(self, *, version):
        with self._lock:
            self._check_version(version)
            if self._status != "waiting_confirmation":
                raise PendingTaskError("whole_plan_not_ready_for_confirmation")
            self._confirmed_version = self._version
            self._confirmed_digest = self._digest()
            self._status = "confirmed"
            self._record("plan_confirmed", plan_digest=self._confirmed_digest)
            return self.snapshot()

    def claim(self, *, version):
        with self._lock:
            self._check_version(version)
            if (self._status != "confirmed" or self._confirmed_version != self._version
                    or self._confirmed_digest != self._digest() or self._result.plan is None):
                raise PendingTaskError("whole_plan_not_confirmed_or_already_claimed")
            self._status = "executing"
            self._record("execution_claimed")
            return self.snapshot()

    def cancel(self):
        with self._lock:
            self._expire()
            if self._status not in self._TERMINAL:
                self._status = "cancelled"
                self._confirmed_version = self._confirmed_digest = None
                self._question = None
                self._record("cancelled")
            return self.snapshot()

    def finish(self, *, outcome):
        with self._lock:
            if outcome not in {"completed", "failed"}:
                raise PendingTaskError("invalid_execution_outcome")
            if self._status != "executing":
                raise PendingTaskError("plan_not_executing")
            self._status = outcome
            self._record("execution_finished", outcome=outcome)
            return self.snapshot()
