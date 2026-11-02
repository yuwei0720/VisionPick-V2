"""Isolated five-slot LLM entrypoint; no robot, catalog mutation or motion.

Candidate transport permits null slots and zero/over-limit task counts so a
structured response does not force invented targets or truncated plans. Only
complete, registered, supported 1-20 task candidates become TaskPlanV2. Pending
state persistence, defaults/confirmation and runtime routing are later phases.

parse_task_plan_v2 requires an explicitly supplied client and makes one
application request without retry/fallback. An authorized provider runner must
configure that client's SDK retries separately; tests supply controlled clients.
"""
from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass, replace
import hashlib
import json
import math
from typing import Any

from jsonschema import Draft202012Validator

from .task_schema_v2 import MAX_TASKS, SELF_RELATIONS, TaskPlanV2


CORE_SLOTS = ("target", "destination_type", "destination_name", "reference", "relation")
PROMPT_VERSION = "3.0.0"
RELATION_TERMS = {
    "left_of": ["左邊", "左側"], "right_of": ["右邊", "右側"],
    "in_front_of": ["前面", "近端"], "behind": ["後面", "遠端"],
    "above": ["上方"], "below": ["下方"], "near": ["附近", "旁邊", "旁邊隨便一個空位", "周圍"],
    "inside": ["放進", "放入", "內部"], "on_top_of": ["上面", "頂部", "頂上", "疊在", "疊到", "疊在…上"],
    "at_initial": ["原始位置", "原位", "移動前的位置"],
}
DEFAULT_RELATIONS = ("left_of", "right_of", "above", "below")

# Descriptive prompt data only. No runtime intent-to-tool lookup executes here.
CAPABILITY_USES = {
    "write": "寫字", "sign": "簽名", "draw": "畫線、畫圖", "mark": "做記號",
    "tighten": "旋緊螺絲", "loosen": "旋鬆螺絲", "fasten": "固定螺絲",
    "power": "供電／裝置沒電時拿取電池", "energize": "提供電力", "store_energy": "儲存電力",
    "cover_written_mark": "遮住紙上原子筆、墨水錯字", "erase_pencil": "擦除鉛筆字跡",
    "mark_surface": "在紙面、紙箱上做記號",
}
SEMANTIC_CATEGORIES = {
    'pen': '文具/書寫與修正用品', 'eraser': '文具/書寫與修正用品',
    'correction tape': '文具/書寫與修正用品', 'marker': '文具/書寫與修正用品',
    'pencil lead case': '文具/書寫與修正用品', 'screwdriver': '五金/維修工具',
    'battery': '電源/電子元件', 'box': '日常容器/盒類',
}

SYSTEM_PROMPT = """你是台灣大學專題的抓放搬運語意解析器。由你理解口語與用途，直接輸出五槽位 JSON，不使用舊版十欄位 intent。
【輸出與順序】
只輸出 {"tasks":[...]}；每個 task 有 target、destination_type、destination_name、reference、relation 五個核心槽位，加 gap_mm 與 displacement_mm；七個欄位全部都要出現。
一次完整拿取→搬運→放置是一個 task；「拿筆再放到站點」合為一次。同物件多次搬動要保留，不能漏步、加步或改順序。
支援 1 到 max_tasks 步；四步、五步及更長計畫皆合法。超過安全上限仍完整輸出，由後端整份拒絕；禁止只輸出前三步或截掉尾端。
【倒裝句與語義角色錨定】
無論語序前後、主受詞是否倒裝（如「咖啡盒子幫我放筆」、「電池左邊移過去橡皮擦」、「疊在盒子上面啦那個修正帶」），被機械臂物理夾取搬運的物件始終為唯一的 target；不得與參照物顛倒。
「盒子／箱子／電池」若接續方位詞（如上面、頂部、右邊、附近），其候選 destination_type 必為 relative、reference 保留該物件 canonical ID（如 box、battery），嚴禁誤填為 named 或替換為咖啡盒子。box 等未具視覺定位能力的實體是待追問候選，不能直接執行。
【物件用途與桌面分類收納】
「桌上的文具都收進…」「維修工具和電源元件都放到…」是類別批次搬運。
有可見物品清單時，只從清單中選擇同類且可抓取的物品，依清單順序各展開一個 task；不能加入未看見的目錄物品。
登錄中 pen、eraser、correction tape、marker、pencil lead case 皆屬於文具類；清單中的修正帶與筆芯盒均為合法文具，必須一併展開，不可漏列。
篩選時必須剔除使用者明確否定／排除的物品；「文具都收進咖啡盒子，但不要動筆」只搬其餘可見文具，不搬pen或非文具。
可見清單非空但沒有任何符合類別且未被排除的物品時，輸出 {"tasks":[]}，不得改抓別類、加入不可見物品或退回全目錄。
清單為空陣列代表已取像但未見物品，輸出 tasks=[]；「尚未取像」才可依全目錄推論。未指定目的地仍按目的地槽位規則處理。
依下方工具用途對照表與登錄清單將推論出的 canonical ID 放入 target：寫字/簽名/畫線選 pen；擦除鉛筆字選 eraser；紙上墨水/原子筆錯字選 correction tape；做記號選 marker；旋螺絲選 screwdriver；需供電/電池選 battery；替換筆芯盒選 pencil lead case。缺少唯一合法目標填 null。
只允許 graspable=true 且 role=manipulable_tool 的工具成為 target。17 個世界實體 role=semantic_context，僅供用途情境推理，不是可抓取或可視覺定位物件，也不能成為可執行的空間 reference。
明確區分直接拿取受詞與功能需求：「最後拿螺絲」「把螺絲搬到站點」target=null，絕不換成 screwdriver；「拿手電筒／滑鼠／筆芯」同理不得換成 battery／筆芯盒。「螺絲鬆了要旋緊」才可選 screwdriver；「手電筒沒電，需要供電工具」才可選 battery。保留未支援拿取步驟與其已知目的地，交後端追問，不可刪步。
本系統僅抓取、搬運、放置工具。工具的旋緊、擦字、供電等用途僅用於選工具，不表示機械臂會鎖螺絲、擦字、插電或推動物品；所有搬運均為夾取後放置。
【台灣口語動作、否定與自我修正】
拿到、移到、移動到、放到、放置在、疊在、疊到、送到、移至、搬到、給我、取、借我、塞到、扔去都是合法搬運詞。
沒有肯定拿取、搬運或用途要求、只有否定時 tasks=[]；被否定的物件不得成為目標或目的地。
口語自我修正（如「放到咖啡盒子，不對，改放電池左邊」）以最後明確肯定的修正目的地為準，取消的目的地不另建 task。
【目的地五槽位規則】
named：destination_name 使用有效具名站點完整名稱（別名如 G 轉為咖啡盒子），reference/relation/gap_mm 嚴格為 null。即使口語帶有「放進去」、「塞到」，具名站點嚴禁填寫 relation。完整具名站點優先於普通容器。
relative：可執行 reference 僅使用 visual_locatable=true 的登錄 ID；若使用者明講未支援世界實體作為參照，候選保留該 ID 交後端追問，禁止替換成其他物品。relation 使用合法關係，destination_name=null、displacement_mm=null。下一步可參照前步搬動的物件。target 與 reference 同類別時嚴格代表兩個不同實例，絕不代表自身位移。
完全未提及目的地（「幫我拿橡皮擦」「給我起子」），四個目的地槽位及 gap_mm、displacement_mm 全填 null。
【自身位移與位置基準不明：必須區分】
self_relative 僅代表抓起後依自身原位平移；reference、destination_name、gap_mm 必須為 null。方向只允許 left_of、right_of、in_front_of、behind，依工作台顯示平面，不代表機器人 Z 軸。禁止自身 near、above、below、on_top_of、at_initial。
「把橡皮擦往右移30mm」→self_relative、right_of、displacement_mm=30；「往右移3公分」同樣為30mm。位移是自身足跡中心改變量，不是兩物間距。
「把橡皮擦往右移」→self_relative、right_of、displacement_mm=null，必須追問距離；「一點」不自動換成固定毫米數。
「把橡皮擦放到右邊／移到右邊」沒有位置基準→destination_type=null、reference=null、destination_name=null、relation=right_of、兩個距離=null，必須追問依自身原位還是另一物件，不得猜成 self_relative。
「把橡皮擦放到電池右邊，邊緣相隔30mm」→relative、reference=battery、right_of、gap_mm=30、displacement_mm=null。
「把這塊橡皮擦放到另一塊橡皮擦右邊」→relative、target=reference=eraser、right_of，代表兩件實物。
連續自身位移每步均以該步抓取前新影格的目標物中心為基準，絕不沿用整段計畫開始時的原位。一般自身位移不使用 at_initial。
named 與 relative 的 displacement_mm 必須為 null。不得將 self_relative 的距離塞入 gap_mm。
明示未登錄站點：destination_type 必須是 named，destination_name 保留未知名稱原文，其餘目的地槽填 null，由後端攔截。
明示相對位置但關係未支援或不合法：destination_type 必須是 relative，保留已知 reference，relation 填 null，嚴禁將未支援關係（如穿透方向、傾斜方向）換成另一個意思或替換為 above。
【兩物交換與空間映射】
「把A跟B交換／互換位置」必須依序展開三步：
1. target=A, reference=B, relation=near（暫存B附近空位）
2. target=B, reference=A, relation=at_initial（移至A整段開始時原位）
3. target=A, reference=B, relation=at_initial（移至B整段開始時原位）
三步均為 relative、destination_name=null、gap_mm=null、displacement_mm=null；at_initial 專指整段計畫開始時原位。若任一方未登錄（如手機），該方 target 填 null 並保留步驟由後端攔截。
【放置間距與方位】
gap_mm 為兩物邊緣間距（毫米），合法範圍 25.0 至 120.0：8公分/80mm→80.0、40mm→40.0、35mm→35.0；遠一點/寬一點→80.0；近一點/貼緊一點/靠緊一點→30.0；未提距離填 null。具名站點與頂部疊放的 gap_mm 必須為 null。
方位：左側→left_of；右側→right_of；前面(近端)→in_front_of；後面(遠端)→behind；上方→above；下方→below；疊在上面/頂部→on_top_of；附近/旁邊空位→near。
【邊界】
不輸出 intent/confidence/status/needs_clarification、座標、機械臂命令或額外欄位。嚴格輸出標準 JSON。"""


class TaskPlanProviderError(RuntimeError):
    """The request failed; this is not a successful semantic rejection."""


@dataclass(frozen=True)
class TaskPlanV2Result:
    status: str
    plan: TaskPlanV2 | None
    raw_payload: Any
    issues: tuple[dict, ...] = ()
    original_text: str = ""
    raw_output_text: str | None = None
    request_metadata: dict | None = None

    def to_dict(self) -> dict:
        return {"status": self.status,
                "plan": self.plan.model_dump() if self.plan is not None else None,
                "raw_payload": deepcopy(self.raw_payload), "issues": deepcopy(list(self.issues)),
                "original_text": self.original_text, "raw_output_text": self.raw_output_text,
                "request_metadata": deepcopy(self.request_metadata), "authorizes_motion": False}


def candidate_response_schema() -> dict:
    """Five slots only, all present; a candidate is not an executable contract."""
    nullable_string = {"anyOf": [{"type": "string"}, {"type": "null"}]}
    properties = {slot: deepcopy(nullable_string) for slot in CORE_SLOTS}
    properties["destination_type"] = {"anyOf": [
        {"type": "string", "enum": ["named", "relative", "self_relative"]}, {"type": "null"}]}
    properties["relation"] = {"anyOf": [
        {"type": "string", "enum": list(RELATION_TERMS)}, {"type": "null"}]}
    properties["gap_mm"] = {"anyOf": [{"type": "number", "minimum": 25.0, "maximum": 120.0},
                                         {"type": "null"}]}
    properties["displacement_mm"] = {"anyOf": [{"type": "number", "exclusiveMinimum": 0.0},
                                                 {"type": "null"}]}
    return {"type": "object", "additionalProperties": False, "required": ["tasks"],
            "properties": {"tasks": {"type": "array", "items": {
                "type": "object", "additionalProperties": False,
                "required": list(CORE_SLOTS), "properties": properties}}}}


def _alias_list(value: Any) -> list[str]:
    if not isinstance(value, (list, tuple)) or any(not isinstance(x, str) for x in value):
        raise ValueError("registry_aliases_must_be_strings")
    return [x.strip() for x in value if x.strip()]


def _context(*, tools=None, world_entities=None, stations=(), supported_relations=DEFAULT_RELATIONS,
             visible_objects=None) -> dict:
    from pathlib import Path
    registry_root = Path(__file__).resolve().parents[2]

    tools = json.loads((registry_root / "tools.json").read_text(encoding="utf-8")) if tools is None else tools
    world_entities = json.loads((registry_root / "world_entities.json").read_text(encoding="utf-8")) if world_entities is None else world_entities
    if not isinstance(world_entities, Mapping):
        raise ValueError("world_entity_registry_must_be_object")
    world = world_entities.get("entities", world_entities)
    if not isinstance(tools, Mapping) or not tools or not isinstance(world, Mapping):
        raise ValueError("entity_registries_must_be_objects")
    objects, references, semantic_entities = {}, {}, {}
    for registry, is_tool in ((tools, True), (world, False)):
        for name, entry in registry.items():
            if (not isinstance(name, str) or not name.strip() or name != name.strip() or name.lower() == "unknown"
                    or not isinstance(entry, Mapping)):
                raise ValueError("invalid_entity_registry_entry")
            role = entry.get('role', 'manipulable_tool' if is_tool else 'semantic_context')
            graspable = entry.get('graspable', is_tool)
            locatable = entry.get('visual_locatable', is_tool)
            action_scope = entry.get('executable_actions', ['pick_and_place'] if is_tool else [])
            capability_scope = entry.get('capability_scope', 'tool_selection_only' if is_tool else 'context_only')
            if (type(graspable) is not bool or type(locatable) is not bool
                    or role != ('manipulable_tool' if is_tool else 'semantic_context')
                    or (not is_tool and (graspable or locatable))
                    or action_scope != (['pick_and_place'] if is_tool else [])
                    or capability_scope != ('tool_selection_only' if is_tool else 'context_only')):
                raise ValueError('invalid_entity_role_capabilities')
            item = {"aliases": _alias_list(entry.get("aliases", [])),
                            "semantic_category": SEMANTIC_CATEGORIES.get(name, '依描述判斷用途分類'),
                            "description": entry.get("description", ""),
                            "capabilities": _alias_list(entry.get("capabilities", [])),
                            "related_objects": _alias_list(entry.get("related_objects", [])),
                            'role': role, 'graspable': graspable, 'visual_locatable': locatable,
                            'capability_scope': capability_scope, 'executable_actions': deepcopy(action_scope)}
            if not isinstance(item["description"], str):
                raise ValueError("registry_description_must_be_string")
            if is_tool:
                if graspable:
                    objects[name] = item
                if locatable:
                    references[name] = deepcopy(item)
            else:
                semantic_entities[name] = item
    if not isinstance(stations, (list, tuple)):
        raise ValueError("stations_must_be_list")
    catalog = []
    seen = set()
    for station in stations:
        if not isinstance(station, Mapping):
            raise ValueError("station_must_be_object")
        name = station.get("name") or station.get("display_name") or station.get("label")
        if not isinstance(name, str) or not name.strip() or name.strip() in seen:
            raise ValueError("station_name_missing_or_duplicated")
        seen.add(name.strip())
        catalog.append({"name": name.strip(), "aliases": _alias_list(station.get("aliases", []))})
    if (not isinstance(supported_relations, (list, tuple))
            or any(not isinstance(x, str) or x not in RELATION_TERMS for x in supported_relations)):
        raise ValueError("invalid_supported_relations")
    if visible_objects is not None:
        if (not isinstance(visible_objects, (list, tuple))
                or any(not isinstance(x, str) or not x.strip() for x in visible_objects)):
            raise ValueError('visible_objects_must_be_names_or_none')
        visible_objects = list(dict.fromkeys(x.strip() for x in visible_objects))
    return {"objects": objects, "references": references, "semantic_entities": semantic_entities, "stations": catalog,
            'self_relative_contract': {'relations':list(SELF_RELATIONS),'anchor_policy':'step_pre_pick',
                'distance_slot':'displacement_mm','distance_metric':'center_translation','coordinate_frame':'display_workbench_plane'},
            "visible_objects": visible_objects,
            "relation_terms": deepcopy(RELATION_TERMS),
            "supported_relations": list(dict.fromkeys(supported_relations)), "max_tasks": MAX_TASKS}


def _prompt(text: str, context: dict) -> str:
    uses = [{"canonical_id": name, "aliases": item["aliases"], "description": item["description"],
             "semantic_category": SEMANTIC_CATEGORIES.get(name, '依描述判斷用途分類'),
             "capabilities": item["capabilities"],
             "special_uses": [CAPABILITY_USES.get(cap, cap) for cap in item["capabilities"]],
             "related_objects": item["related_objects"]} for name, item in context["objects"].items()]
    visible = context['visible_objects']
    scene = '尚未取像，可依全目錄推論' if visible is None else json.dumps(visible, ensure_ascii=False)
    return (SYSTEM_PROMPT + "\n【當前桌面視覺偵測到的可見物品清單】\n" + scene
            + "\n工具用途對照表（JSON）：\n" + json.dumps(uses, ensure_ascii=False)
            + "\n完整詞彙與有效站點登錄（JSON）：\n" + json.dumps(context, ensure_ascii=False, sort_keys=True)
            + "\n使用者文字（JSON 字串）：\n" + json.dumps(text, ensure_ascii=False))


def build_task_plan_v2_prompt(text: str, **context_options) -> str:
    if not isinstance(text, str) or not text.strip():
        raise ValueError("input_text_must_not_be_empty")
    return _prompt(text, _context(**context_options))


def _validate(payload: Any, context: dict) -> TaskPlanV2Result:
    snapshot = deepcopy(payload)
    errors = list(Draft202012Validator(candidate_response_schema()).iter_errors(payload))
    if errors:
        issues = tuple({"code": "candidate_schema_invalid", "path": list(e.absolute_path),
                        "validator": e.validator} for e in errors)
        return TaskPlanV2Result("blocked", None, snapshot, issues)
    count = len(payload["tasks"])
    if not 1 <= count <= context["max_tasks"]:
        code = "no_transfer_tasks" if count == 0 else "task_count_exceeds_limit"
        return TaskPlanV2Result("blocked", None, snapshot, ({"code": code, "task_count": count},))
    station_names = {x["name"] for x in context["stations"]}
    normalized, issues = [], []
    for index, raw in enumerate(payload["tasks"], 1):
        item = {k: (v.strip() or None) if isinstance(v, str) else v for k, v in raw.items()}
        normalized.append(item)

        def issue(code, slot, *, blocked=False):
            issues.append({"code": code, "task_index": index, "slot": slot,
                           "status": "blocked" if blocked else "clarify"})

        gap = item.get("gap_mm")
        displacement = item.get('displacement_mm')
        if displacement is not None and (isinstance(displacement, bool) or not isinstance(displacement, (int, float))
                                         or not math.isfinite(displacement) or displacement <= 0):
            issue('displacement_out_of_range', 'displacement_mm', blocked=True)
        if displacement is not None and item['destination_type'] != 'self_relative':
            issue('displacement_not_applicable', 'displacement_mm', blocked=True)
        if gap is not None and (isinstance(gap, bool) or not isinstance(gap, (int, float))
                                or not math.isfinite(gap) or not 25.0 <= gap <= 120.0):
            issue("gap_out_of_range", "gap_mm", blocked=True)
        if gap is not None and (item["destination_type"] != "relative" or item["relation"] in {"on_top_of", "at_initial"}):
            issue("gap_not_applicable", "gap_mm", blocked=True)

        if item["target"] is None:
            issue("target_missing", "target")
        elif item["target"] not in context["objects"]:
            issue("target_not_registered", "target")
        elif not context['objects'][item['target']]['visual_locatable']:
            issue('unsupported_visual_target', 'target')
        kind = item["destination_type"]
        if kind is None:
            ambiguous = item['relation'] in SELF_RELATIONS and item['reference'] is None and item['destination_name'] is None
            issue('destination_basis_missing' if ambiguous else 'destination_missing',
                  'destination_basis' if ambiguous else 'destination_type')
            if item['destination_name'] is not None or item['reference'] is not None or (item['relation'] is not None and not ambiguous):
                issue("destination_fields_without_type", "destination_type", blocked=True)
        elif kind == "named":
            if item["destination_name"] is None:
                issue("destination_name_missing", "destination_name")
            elif item["destination_name"] not in station_names:
                issue("destination_name_not_registered", "destination_name")
            if item["reference"] is not None or item["relation"] is not None:
                issue("named_destination_has_relative_fields", "destination_type", blocked=True)
        elif kind == 'self_relative':
            if any(item.get(k) is not None for k in ('reference', 'destination_name', 'gap_mm')):
                issue('self_relative_has_foreign_fields', 'destination_type', blocked=True)
            if item['relation'] is None:
                issue('relation_missing', 'relation')
            elif item['relation'] not in SELF_RELATIONS:
                issue('self_relative_direction_not_supported', 'relation', blocked=True)
            if displacement is None:
                issue('displacement_missing', 'displacement_mm')
        else:
            if item["destination_name"] is not None:
                issue("relative_destination_has_name", "destination_name", blocked=True)
            if item["reference"] is None:
                issue("reference_missing", "reference")
            elif item["reference"] not in context["references"]:
                issue('unsupported_visual_reference' if item['reference'] in context['semantic_entities'] else 'reference_not_registered', 'reference')
            elif item["reference"] == item["target"] and item['relation'] != 'at_initial':
                issue("same_entity_reference_requires_disambiguation", "reference")
            if item["relation"] is None:
                issue("relation_missing", "relation")
            elif item["relation"] not in context["supported_relations"]:
                issue("relation_not_enabled", "relation", blocked=True)
    if issues:
        status = "blocked" if any(x["status"] == "blocked" for x in issues) else "clarify"
        return TaskPlanV2Result(status, None, snapshot, tuple(issues))
    return TaskPlanV2Result("ready", TaskPlanV2.model_validate({"tasks": normalized}), snapshot)


def validate_task_plan_v2_candidate(payload: Any, **context_options) -> TaskPlanV2Result:
    """Check format, registry membership, completeness and declared capabilities.

    Does not reinterpret original language or authorize execution. Candidate
    meaning remains the LLM's responsibility and must be measured independently.
    """
    return _validate(payload, _context(**context_options))


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate_json_key")
        result[key] = value
    return result


def parse_task_plan_v2(text: str, *, client, policy=None, profile_name=None,
                       **context_options) -> TaskPlanV2Result:
    """One caller-authorized client request; no client creation or app retries."""
    from pathlib import Path
    registry_root = Path(__file__).resolve().parents[2]

    if client is None:
        raise ValueError("explicit_client_required")
    if not isinstance(text, str) or not text.strip():
        raise ValueError("input_text_must_not_be_empty")
    context = _context(**context_options)
    policy = json.loads((registry_root / "model_policy.json").read_text(encoding="utf-8")) if policy is None else policy
    name = profile_name or policy["production"]["primary_profile"]
    profile = policy["profiles"][name]
    prompt, schema = _prompt(text, context), candidate_response_schema()
    metadata = {"model": profile["model"], "profile_name": name, "prompt_version": PROMPT_VERSION, "application_requests": 1,
                "sdk_retry_control": "caller_supplied_client",
                "prompt_sha256": hashlib.sha256(prompt.encode("utf-8")).hexdigest(),
                "schema_sha256": hashlib.sha256(json.dumps(schema, sort_keys=True).encode("utf-8")).hexdigest()}
    try:
        response = client.interactions.create(
            model=profile["model"], input=prompt,
            response_format={"type": "text", "mime_type": "application/json", "schema": schema},
            generation_config={"thinking_level": profile["thinking_level"]},
        )
    except Exception as exc:
        raise TaskPlanProviderError("task_plan_v2_request_failed") from exc
    output = getattr(response, "output_text", None)
    try:
        if not isinstance(output, str):
            raise ValueError("response_output_text_missing")
        payload = json.loads(output, object_pairs_hook=_unique_json_object)
    except (ValueError, TypeError):
        result = TaskPlanV2Result("blocked", None, None, ({"code": "response_json_invalid"},))
    else:
        result = _validate(payload, context)
    return replace(result, original_text=text, raw_output_text=output, request_metadata=metadata)
