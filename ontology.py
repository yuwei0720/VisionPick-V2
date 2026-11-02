import copy
import json
import re

REQUEST_NEGATION_RE = re.compile(
    r"不要|別|别|不想|不准|不準|不拿|不抓|不用|不必|不能|禁止|\b(?:not|never|don't|dont|without)\b", re.I
)
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
WORLD_ENTITIES_FILE = BASE_DIR / "world_entities.json"

INTENTS = {
    "pick", "use", "move", "place", "diagnose", "unknown"
}

SPATIAL_RELATIONS = {
    "unknown", "left_of", "right_of", "above", "below",
    "near", "inside", "on_top_of",
}

CLARIFICATION_REASONS = {
    "unknown",
    "missing_reference_object",
    "missing_pick_target",
    "missing_move_target",
    "missing_place_target",
    "missing_use_target_or_capability",
    "missing_affected_object",
    "multi_target_not_supported",
    "out_of_domain_or_unsupported",
    "deictic_reference_unresolved",
}

# Historical/model wording aliases canonicalized to the single finite reason vocabulary.
CLARIFICATION_REASON_ALIASES = {
    "missing_required_capability_or_target": "missing_use_target_or_capability",
}


TASK_FIELDS = {
    "intent",
    "target_object",
    "affected_object",
    "required_capability",
    "reference_object",
    "spatial_relation",
    "requires_perception",
    "needs_clarification",
    "clarification_reason",
    "confidence",
}

ENTITY_FIELDS = (
    "target_object",
    "affected_object",
    "reference_object",
)


class SchemaViolationError(ValueError):
    def __init__(self, violations):
        self.violations = violations
        super().__init__(
            "Semantic Task schema violation: "
            + "; ".join(violations)
        )


def load_world_entities():
    data = json.loads(
        WORLD_ENTITIES_FILE.read_text(encoding="utf-8")
    )

    entities = data.get("entities")
    if not isinstance(entities, dict) or not entities:
        raise ValueError(
            "world_entities.json 必須包含非空 entities object"
        )

    for entity_id, info in entities.items():
        if not isinstance(info, dict):
            raise ValueError(
                f"world entity {entity_id} 格式錯誤"
            )

        aliases = info.get("aliases")
        if not isinstance(aliases, list):
            raise ValueError(
                f"world entity {entity_id} 缺少 aliases list"
            )

    return data


def world_entity_map(world_entities=None):
    data = world_entities or load_world_entities()
    return data["entities"]


def entity_ids(tools, world_entities=None):
    world = world_entity_map(world_entities)
    return {
        "unknown",
        *tools.keys(),
        *world.keys(),
    }


def capability_ids(tools):
    return {
        "unknown",
        *[
            capability
            for info in tools.values()
            for capability in info.get("capabilities", [])
        ],
    }


def build_entity_alias_index(tools, world_entities=None):
    index = {"unknown": "unknown"}

    for entity_id, info in tools.items():
        for token in [
            entity_id,
            *info.get("aliases", []),
        ]:
            normalized = str(token).strip().lower()
            if normalized:
                index[normalized] = entity_id

    for entity_id, info in world_entity_map(
        world_entities
    ).items():
        for token in [
            entity_id,
            *info.get("aliases", []),
        ]:
            normalized = str(token).strip().lower()
            if normalized:
                index[normalized] = entity_id

    return index


def canonicalize_entity(
    value,
    tools,
    world_entities=None,
):
    value = str(value or "").strip()

    if not value:
        return "unknown"

    index = build_entity_alias_index(
        tools,
        world_entities,
    )

    return index.get(
        value.lower(),
        value,
    )


def entity_occurrences(
    text,
    tools,
    world_entities=None,
):
    """
    Return (start, end, canonical entity, alias) occurrences in text.

    Lexical matches are resolved across entity boundaries with a longest-span
    rule.  A shorter alias occurrence is ignored when that exact text span is
    strictly contained inside a longer alias occurrence for another entity.
    This prevents compound aliases such as 「螺絲起子」 from also creating a
    spurious 「螺絲」 entity, while preserving a separate explicit 「螺絲」
    occurrence elsewhere in the same sentence.

    Exact same-span aliases remain ambiguous and therefore keep both entities.
    """
    text_lower = text.lower()

    sources = []
    for entity_id, info in tools.items():
        sources.append(
            (
                entity_id,
                [entity_id, *info.get("aliases", [])],
            )
        )

    for entity_id, info in world_entity_map(
        world_entities
    ).items():
        sources.append(
            (
                entity_id,
                [entity_id, *info.get("aliases", [])],
            )
        )

    occurrences = []
    seen = set()

    for entity_id, aliases in sources:
        aliases = {
            str(alias).strip().lower()
            for alias in aliases
            if str(alias).strip()
        }

        for alias in aliases:
            start = 0
            while True:
                index = text_lower.find(alias, start)
                if index < 0:
                    break

                item = (
                    index,
                    index + len(alias),
                    entity_id,
                    alias,
                )
                if item not in seen:
                    occurrences.append(item)
                    seen.add(item)

                start = index + max(1, len(alias))

    # Catalog relations ground explicit container/content descriptions using
    # the same spans as aliases, so parts do not become extra pick targets.
    world = world_entity_map(world_entities)
    literal_occurrences = list(occurrences)
    descriptions = []
    contains_verb = r"(?:裝著|装着|裝有|装有|裝|装|放著|放着|存放|收著|收着|收納|收纳|容納|容纳)"
    for entity_id, info in tools.items():
        relation = info.get("grounding_description", {})
        head, content = relation.get("head_entity"), relation.get("contains_entity")
        if (world.get(head, {}).get("type") != "container"
                or world.get(content, {}).get("type") != "part"):
            continue
        for content_start, content_end, part, _ in literal_occurrences:
            if part != content:
                continue
            if re.match(r"\s*(?:和|與|及|、|跟|以及|或|and\b|or\b)", text_lower[content_end:]):
                continue
            prefix = text_lower[max(0, content_start - 16):content_start]
            match = re.search(contains_verb +
                              r"\s*(?:備用|备用|替換|替换)?\s*$", prefix)
            for head_start, head_end, container, _ in literal_occurrences:
                if container != head:
                    continue
                if (match and content_end <= head_start
                        and re.fullmatch(r"\s*的\s*(?:小|大)?\s*", text_lower[content_end:head_start])):
                    start = content_start - len(prefix) + match.start()
                    item = (start, head_end, entity_id, text_lower[start:head_end])
                elif (head_end <= content_start and re.fullmatch(
                        r"\s*(?:裡|里|內|内)?\s*" + contains_verb + r"\s*(?:備用|备用|替換|替换)?\s*",
                        text_lower[head_end:content_start])):
                    item = (head_start, content_end, entity_id, text_lower[head_start:content_end])
                else:
                    continue
                if item not in seen:
                    occurrences.append(item)
                    descriptions.append((item, head))
                    seen.add(item)

    # Link a demonstrative container head only to one unambiguous description.
    for head_start, head_end, head, alias in literal_occurrences:
        antecedents = {item[2] for item, described_head in descriptions
                       if described_head == head and item[1] <= head_start}
        prefix = text_lower[max(0, head_start - 3):head_start]
        if len(antecedents) == 1 and re.search(r"(?:這個|这个|該|该)$", prefix):
            prior_heads = [(start, end) for start, end, entity, _ in literal_occurrences
                           if entity == head and end <= head_start]
            if all(any(item[0] <= start and end <= item[1] for item, _ in descriptions)
                   for start, end in prior_heads):
                occurrences.append((head_start, head_end, next(iter(antecedents)), alias))
                occurrences = [item for item in occurrences
                               if item[:3] != (head_start, head_end, head)]

    found = []

    for start, end, entity_id, alias in occurrences:
        shadowed = any(
            other_entity != entity_id
            and other_start <= start
            and end <= other_end
            and (other_end - other_start) > (end - start)
            for (
                other_start,
                other_end,
                other_entity,
                _other_alias,
            ) in occurrences
        )

        if not shadowed:
            found.append((start, end, entity_id, alias))

    return found


def explicitly_mentioned_entities(text, tools, world_entities=None):
    return {entity_id for _, _, entity_id, _ in
            entity_occurrences(text, tools, world_entities)}


def build_hardened_schema(
    base_schema,
    tools,
    world_entities=None,
    parser_version="3.4.1",
    schema_version="3.4.1",
):
    schema = copy.deepcopy(base_schema)

    entities = sorted(entity_ids(
        tools,
        world_entities,
    ))
    capabilities = sorted(capability_ids(tools))
    reasons = sorted(CLARIFICATION_REASONS)

    schema["additionalProperties"] = False

    # v3.5.1 infrastructure hotfix:
    # Pydantic defaults are useful internally, but the live API boundary
    # must return the complete finite Task IR. Otherwise the generated
    # schema permits omitted defaulted fields while strict hardening
    # correctly rejects the incomplete payload.
    schema["required"] = sorted(TASK_FIELDS)

    properties = schema["properties"]

    for field in ENTITY_FIELDS:
        properties[field]["enum"] = entities
        properties[field]["description"] = (
            "只能輸出 canonical entity_id 或 unknown。"
            "禁止輸出自然語言別名、句子、推理過程或自由文字。"
        )

    properties["required_capability"]["enum"] = (
        capabilities
    )
    properties["required_capability"]["description"] = (
        "只能輸出目前 capability ontology 中的值或 unknown；"
        "不是工具名稱。"
    )

    properties["clarification_reason"]["enum"] = reasons
    properties["clarification_reason"]["description"] = (
        "只能輸出已定義 clarification reason 或 unknown。"
    )

    schema["x-schema-version"] = schema_version
    schema["x-parser-version"] = parser_version
    schema["x-closed-world-language-parser"] = True
    schema["x-entity-ontology-version"] = (
        world_entities or load_world_entities()
    ).get("ontology_version", "unknown")

    return schema


def harden_payload(
    payload,
    tools,
    world_entities=None,
    *,
    strict=False,
):
    """
    Canonicalize a model payload against the finite ontology.

    strict=True:
        live model output. Any out-of-ontology value is rejected.

    strict=False:
        historical/offline replay. Invalid values are converted
        to "unknown" and fully recorded in diagnostics.
    """
    if not isinstance(payload, dict):
        raise SchemaViolationError(
            ["payload must be a JSON object"]
        )

    data = dict(payload)

    diagnostics = {
        "corrections": [],
        "violations": [],
        "severe_correction": False,
    }

    extras = sorted(set(data) - TASK_FIELDS)
    missing = sorted(TASK_FIELDS - set(data))

    if extras:
        message = f"extra_fields={extras}"
        diagnostics["violations"].append(message)

        if strict:
            raise SchemaViolationError(
                diagnostics["violations"]
            )

        for field in extras:
            data.pop(field, None)

    defaults = {
        "intent": "unknown",
        "target_object": "unknown",
        "affected_object": "unknown",
        "required_capability": "unknown",
        "reference_object": "unknown",
        "spatial_relation": "unknown",
        "requires_perception": False,
        "needs_clarification": False,
        "clarification_reason": "unknown",
        "confidence": 0.0,
    }

    if missing:
        message = f"missing_fields={missing}"
        diagnostics["violations"].append(message)

        if strict:
            raise SchemaViolationError(
                diagnostics["violations"]
            )

        for field in missing:
            data[field] = defaults[field]
            diagnostics["corrections"].append(
                f"filled_missing_{field}"
            )

    allowed_entities = entity_ids(
        tools,
        world_entities,
    )
    allowed_capabilities = capability_ids(tools)

    for field in ENTITY_FIELDS:
        original = data.get(field)
        canonical = canonicalize_entity(
            original,
            tools,
            world_entities,
        )

        if canonical in allowed_entities:
            if canonical != original:
                diagnostics["corrections"].append(
                    f"canonicalized_{field}"
                )
            data[field] = canonical
            continue

        message = (
            f"{field} out_of_ontology: "
            f"{original!r}"
        )
        diagnostics["violations"].append(message)
        diagnostics["severe_correction"] = True

        if strict:
            raise SchemaViolationError(
                diagnostics["violations"]
            )

        data[field] = "unknown"
        diagnostics["corrections"].append(
            f"rejected_{field}_to_unknown"
        )

    capability = str(
        data.get("required_capability") or ""
    ).strip() or "unknown"

    if capability not in allowed_capabilities:
        message = (
            "required_capability out_of_ontology: "
            f"{capability!r}"
        )
        diagnostics["violations"].append(message)
        diagnostics["severe_correction"] = True

        if strict:
            raise SchemaViolationError(
                diagnostics["violations"]
            )

        capability = "unknown"
        diagnostics["corrections"].append(
            "rejected_required_capability_to_unknown"
        )

    data["required_capability"] = capability

    intent = str(
        data.get("intent") or ""
    ).strip() or "unknown"

    if intent not in INTENTS:
        message = f"intent out_of_ontology: {intent!r}"
        diagnostics["violations"].append(message)
        diagnostics["severe_correction"] = True

        if strict:
            raise SchemaViolationError(
                diagnostics["violations"]
            )

        intent = "unknown"
        diagnostics["corrections"].append(
            "rejected_intent_to_unknown"
        )

    data["intent"] = intent

    relation = str(
        data.get("spatial_relation") or ""
    ).strip() or "unknown"

    if relation not in SPATIAL_RELATIONS:
        message = (
            "spatial_relation out_of_ontology: "
            f"{relation!r}"
        )
        diagnostics["violations"].append(message)
        diagnostics["severe_correction"] = True

        if strict:
            raise SchemaViolationError(
                diagnostics["violations"]
            )

        relation = "unknown"
        diagnostics["corrections"].append(
            "rejected_spatial_relation_to_unknown"
        )

    data["spatial_relation"] = relation

    reason = str(
        data.get("clarification_reason") or ""
    ).strip() or "unknown"

    canonical_reason = CLARIFICATION_REASON_ALIASES.get(
        reason,
        reason,
    )

    if canonical_reason != reason:
        diagnostics["corrections"].append(
            "canonicalized_clarification_reason"
        )
        reason = canonical_reason

    if reason not in CLARIFICATION_REASONS:
        message = (
            "clarification_reason out_of_ontology: "
            f"{reason!r}"
        )
        diagnostics["violations"].append(message)

        if strict:
            raise SchemaViolationError(
                diagnostics["violations"]
            )

        reason = "unknown"
        diagnostics["corrections"].append(
            "rejected_clarification_reason_to_unknown"
        )

    data["clarification_reason"] = reason

    return data, diagnostics
