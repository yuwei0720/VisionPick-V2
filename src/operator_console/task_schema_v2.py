"""Version 2 task specification: unified 5-slot schema with tasks[] container."""
from __future__ import annotations

from typing import Literal
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


DestinationType = Literal["named", "relative", "self_relative"]
SELF_RELATIONS = ("left_of", "right_of", "in_front_of", "behind")
TASK_SCHEMA_VERSION = '3.0.0'
MAX_TASKS = 20
SpatialRelation = Literal[
    "left_of", "right_of", "in_front_of", "behind", "above", "below", "near", "inside", "on_top_of", "at_initial"
]

# JSON Schema validates raw JSON strings; Python strips whitespace before its
# target validator. Reject whitespace-only and case-insensitive unknown there
# as well, without changing the existing Python normalization behavior.
TARGET_JSON_PATTERN = r"^(?!\s*[uU][nN][kK][nN][oO][wW][nN]\s*$)[\s\S]*\S[\s\S]*$"


class TaskItem(BaseModel):
    """Core 5-slot task representation.

    Every task represents a single atomic pick -> move -> place action.
    """
    model_config = ConfigDict(
        extra="forbid",
        str_strip_whitespace=True,
        json_schema_extra={"allOf": [
            {"if": {"properties": {"destination_type": {"const": "self_relative"}}, "required": ["destination_type"]},
             "then": {"properties": {"reference": {"type": "null"}, "destination_name": {"type": "null"},
                                      "gap_mm": {"type": "null"}, "relation": {"enum": [*SELF_RELATIONS, None]}}},
             "else": {"properties": {"displacement_mm": {"type": "null"}}}},
        ]},
    )

    target: str = Field(
        ...,
        min_length=1,
        json_schema_extra={"pattern": TARGET_JSON_PATTERN},
        description="抓取目標之 canonical entity ID（如 pen、battery），不可為空白。"
    )
    destination_type: DestinationType | None = Field(
        default=None,
        description="目的地類型：named（具名站點）、relative（另一實例）、self_relative（當步抓取前自身中心位移）；未指定為 null。"
    )
    destination_name: str | None = Field(
        default=None,
        description="已啟用的具名站點名稱（如 Station A、咖啡盒子）；若非具名站點則為 null。"
    )
    reference: str | None = Field(
        default=None,
        description="另一可視覺定位實例之 entity ID（如 battery、pen）；自身位移與非相對關係必須為 null。"
    )
    relation: SpatialRelation | None = Field(
        default=None,
        description="相對於參照物的空間幾何關係（如 left_of、above）；若非相對關係則為 null。"
    )
    gap_mm: float | None = Field(
        default=None, ge=25.0, le=120.0, strict=True, allow_inf_nan=False,
        description="桌面相對放置的物品邊緣間距（mm），25 至 120；null 採系統預設。",
    )
    displacement_mm: float | None = Field(
        default=None, gt=0.0, strict=True, allow_inf_nan=False,
        description="自身中心平移距離（mm），有限正數；僅 self_relative 使用，null 須追問。以各步抓取前當幀為基準，方向依工作台顯示平面。",
    )

    @model_validator(mode="after")
    def validate_distance_contract(self):
        if self.destination_type == "self_relative":
            if self.reference is not None or self.destination_name is not None or self.gap_mm is not None:
                raise ValueError("self_relative_has_foreign_fields")
            if self.relation is not None and self.relation not in SELF_RELATIONS:
                raise ValueError("self_relative_direction_not_supported")
        elif self.displacement_mm is not None:
            raise ValueError("displacement_not_applicable")
        return self

    @field_validator("target")
    @classmethod
    def validate_target_not_empty(cls, v: str) -> str:
        s = v.strip()
        if not s or s.lower() == "unknown":
            raise ValueError("target 必須為有效實體名稱，不可為空白或 'unknown'")
        return s


class TaskPlanV2(BaseModel):
    """Unified task plan container for both single-step and multi-step executions."""
    model_config = ConfigDict(
        extra="forbid",
        str_strip_whitespace=True,
    )

    tasks: list[TaskItem] = Field(
        ...,
        min_length=1,
        max_length=MAX_TASKS,
        description="依原指令順序執行的任務清單；支援 1 至 20 步，20 為安全上限。",
    )

    @field_validator("tasks", mode="before")
    @classmethod
    def validate_tasks_count(cls, v: list[TaskItem]) -> list[TaskItem]:
        if isinstance(v, (list, tuple)) and not (1 <= len(v) <= MAX_TASKS):
            raise ValueError(f"tasks 清單長度必須介於 1 至 {MAX_TASKS} 步，目前收到 {len(v)} 步")
        return v
