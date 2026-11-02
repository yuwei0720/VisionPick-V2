from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Any, Iterable


SUPPORTED_2D_RELATIONS = frozenset({
    "left_of",
    "right_of",
    "above",
    "below",
    "near",
})


class SceneGroundingError(RuntimeError):
    def __init__(self, reason: str, message: str, *, details: dict[str, Any] | None = None):
        super().__init__(message)
        self.reason = reason
        self.details = dict(details or {})


@dataclass(frozen=True)
class SceneObject2D:
    index: int
    class_name: str
    confidence: float
    raw_center_uv: tuple[float, float]
    workspace_center_xy: tuple[float, float]

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "class_name": self.class_name,
            "confidence": round(float(self.confidence), 4),
            "raw_center_uv": [round(float(x), 2) for x in self.raw_center_uv],
            "workspace_center_xy": [
                round(float(x), 2) for x in self.workspace_center_xy
            ],
        }


@dataclass(frozen=True)
class SceneSnapshot2D:
    raw_width: int
    raw_height: int
    workspace_width: int
    workspace_height: int
    workspace_view_rotation_deg: int
    objects: tuple[SceneObject2D, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "raw_size": [self.raw_width, self.raw_height],
            "workspace_size": [self.workspace_width, self.workspace_height],
            "workspace_view_rotation_deg": self.workspace_view_rotation_deg,
            "objects": [obj.to_dict() for obj in self.objects],
        }


@dataclass(frozen=True)
class SceneResolution2D:
    selected_index: int
    reference_index: int
    relation: str
    target_filter: str | None
    snapshot: SceneSnapshot2D
    relation_metric: float

    def to_dict(self) -> dict[str, Any]:
        selected = self.snapshot.objects[self.selected_index]
        reference = self.snapshot.objects[self.reference_index]
        return {
            "status": "resolved",
            "relation": self.relation,
            "target_filter": self.target_filter,
            "selected": selected.to_dict(),
            "reference": reference.to_dict(),
            "relation_metric": round(float(self.relation_metric), 6),
            "workspace_view_rotation_deg": self.snapshot.workspace_view_rotation_deg,
        }


def normalize_rotation_deg(value: int | float | str) -> int:
    try:
        rotation = int(value) % 360
    except (TypeError, ValueError) as exc:
        raise ValueError("workspace_view_rotation_deg must be 0/90/180/270") from exc
    if rotation not in {0, 90, 180, 270}:
        raise ValueError("workspace_view_rotation_deg must be 0/90/180/270")
    return rotation


def raw_to_workspace_point(
    u: float,
    v: float,
    *,
    raw_width: int,
    raw_height: int,
    rotation_deg: int,
) -> tuple[float, float]:
    """Map a raw-camera pixel center into the user-facing workspace view.

    Stage 3A defines the workspace frame as the human/robot-table view. With
    rotation_deg=90, the raw image is interpreted as if rotated clockwise 90°:
    raw right -> workspace down (robot direction), raw up -> workspace right.
    """

    rotation = normalize_rotation_deg(rotation_deg)
    u = float(u)
    v = float(v)

    if rotation == 0:
        return u, v
    if rotation == 90:
        return float(raw_height - 1) - v, u
    if rotation == 180:
        return float(raw_width - 1) - u, float(raw_height - 1) - v
    return v, float(raw_width - 1) - u


def workspace_size(raw_width: int, raw_height: int, rotation_deg: int) -> tuple[int, int]:
    rotation = normalize_rotation_deg(rotation_deg)
    if rotation in {90, 270}:
        return int(raw_height), int(raw_width)
    return int(raw_width), int(raw_height)


def build_scene_snapshot(
    detections: Iterable[Any],
    *,
    raw_width: int,
    raw_height: int,
    rotation_deg: int = 90,
) -> SceneSnapshot2D:
    rotation = normalize_rotation_deg(rotation_deg)
    ww, wh = workspace_size(raw_width, raw_height, rotation)
    objects = []
    for index, det in enumerate(detections):
        u, v = det.center_uv
        x, y = raw_to_workspace_point(
            u,
            v,
            raw_width=raw_width,
            raw_height=raw_height,
            rotation_deg=rotation,
        )
        objects.append(SceneObject2D(
            index=index,
            class_name=str(det.class_name).strip(),
            confidence=float(det.confidence),
            raw_center_uv=(float(u), float(v)),
            workspace_center_xy=(x, y),
        ))
    return SceneSnapshot2D(
        raw_width=int(raw_width),
        raw_height=int(raw_height),
        workspace_width=ww,
        workspace_height=wh,
        workspace_view_rotation_deg=rotation,
        objects=tuple(objects),
    )


def _canonical_name(name: str, aliases: dict | None) -> str:
    aliases = aliases or {}
    raw = str(name).strip()
    if raw in aliases:
        return str(aliases[raw]).strip().lower()
    lower = {str(k).strip().lower(): str(v).strip() for k, v in aliases.items()}
    return lower.get(raw.lower(), raw).strip().lower()


def _matching_indices(snapshot: SceneSnapshot2D, name: str, aliases: dict | None) -> list[int]:
    canonical = _canonical_name(name, aliases)
    return [
        obj.index
        for obj in snapshot.objects
        if obj.class_name.strip().lower() == canonical
    ]


def resolve_spatial_target(
    detections: list[Any],
    *,
    reference_object: str,
    relation: str,
    target_object: str = "unknown",
    aliases: dict | None = None,
    raw_width: int,
    raw_height: int,
    scene_cfg: dict | None = None,
) -> tuple[Any, SceneResolution2D]:
    """Resolve one relation-grounded pick target from current YOLO detections.

    Stage 3A is intentionally conservative: the reference must be unique, the
    relation must be one of the supported 2D relations, and exactly one target
    candidate must satisfy the relation. Ambiguous scenes are blocked rather
    than ranked by an implicit policy.
    """

    cfg = dict(scene_cfg or {})
    rotation = normalize_rotation_deg(cfg.get("workspace_view_rotation_deg", 90))
    relation_margin_fraction = float(cfg.get("relation_margin_fraction", 0.03))
    near_max_fraction = float(cfg.get("near_max_fraction", 0.25))

    if relation not in SUPPORTED_2D_RELATIONS:
        raise SceneGroundingError(
            "relation_not_supported_stage3a",
            f"Stage 3A does not support relation: {relation}",
            details={"relation": relation},
        )
    if not detections:
        raise SceneGroundingError("no_detections", "No YOLO detections available")

    snapshot = build_scene_snapshot(
        detections,
        raw_width=raw_width,
        raw_height=raw_height,
        rotation_deg=rotation,
    )

    ref_indices = _matching_indices(snapshot, reference_object, aliases)
    if not ref_indices:
        raise SceneGroundingError(
            "reference_not_detected",
            f"Reference object not detected: {reference_object}",
            details={"reference_object": reference_object},
        )
    if len(ref_indices) != 1:
        raise SceneGroundingError(
            "reference_ambiguous",
            f"Reference object is not unique: {reference_object}",
            details={"reference_object": reference_object, "matches": ref_indices},
        )

    ref_index = ref_indices[0]
    ref = snapshot.objects[ref_index]
    candidates = [obj for obj in snapshot.objects if obj.index != ref_index]

    target_filter = None
    if target_object and target_object != "unknown":
        target_filter = _canonical_name(target_object, aliases)
        candidates = [
            obj for obj in candidates
            if obj.class_name.strip().lower() == target_filter
        ]
        if not candidates:
            raise SceneGroundingError(
                "target_class_not_detected",
                f"Target class not detected: {target_object}",
                details={"target_object": target_object},
            )

    ref_x, ref_y = ref.workspace_center_xy
    margin_x = max(1.0, snapshot.workspace_width * relation_margin_fraction)
    margin_y = max(1.0, snapshot.workspace_height * relation_margin_fraction)
    near_px = hypot(snapshot.workspace_width, snapshot.workspace_height) * near_max_fraction

    qualified: list[tuple[SceneObject2D, float]] = []
    for obj in candidates:
        x, y = obj.workspace_center_xy
        dx = x - ref_x
        dy = y - ref_y
        distance = hypot(dx, dy)

        if relation == "left_of" and dx < -margin_x:
            qualified.append((obj, abs(dx)))
        elif relation == "right_of" and dx > margin_x:
            qualified.append((obj, abs(dx)))
        elif relation == "above" and dy < -margin_y:
            qualified.append((obj, abs(dy)))
        elif relation == "below" and dy > margin_y:
            qualified.append((obj, abs(dy)))
        elif relation == "near" and distance <= near_px:
            qualified.append((obj, distance))

    if not qualified:
        raise SceneGroundingError(
            "no_relation_match",
            f"No object satisfies {relation} relative to {reference_object}",
            details={
                "reference_object": reference_object,
                "relation": relation,
                "target_filter": target_filter,
            },
        )
    if len(qualified) != 1:
        raise SceneGroundingError(
            "ambiguous_relation_match",
            f"Multiple objects satisfy {relation} relative to {reference_object}",
            details={
                "reference_object": reference_object,
                "relation": relation,
                "candidate_indices": [obj.index for obj, _ in qualified],
                "target_filter": target_filter,
            },
        )

    selected_obj, metric = qualified[0]
    resolution = SceneResolution2D(
        selected_index=selected_obj.index,
        reference_index=ref_index,
        relation=relation,
        target_filter=target_filter,
        snapshot=snapshot,
        relation_metric=metric,
    )
    return detections[selected_obj.index], resolution
