"""Model canonical station name -> unique ready local pose, without A/B/C slots."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from types import SimpleNamespace
import unicodedata

from .direct_control_v2 import _six


class StationLookupError(ValueError):
    pass


def _token(value):
    return unicodedata.normalize("NFKC", value).strip().casefold()


class PlaceCatalogV2:
    """Freeze local placement data; only names and aliases are sent to the model.

    Historical versions collapse only when their family has one active version.
    Different ready stations must not share a name/alias. Unknown/inactive names
    never resolve to a selected default. The initial model result must use the
    canonical name; aliases are resolved only for a local clarification answer.
    """

    def __init__(self, snapshot):
        if not isinstance(snapshot, dict) or not isinstance(snapshot.get("profiles"), list):
            raise StationLookupError("invalid_station_catalog")
        families = {}
        self.default_profile_id = snapshot.get("default_profile_id")
        seen_ids = set()
        for raw in snapshot["profiles"]:
            if not isinstance(raw, dict):
                raise StationLookupError("invalid_station_record")
            row = deepcopy(raw)
            name = row.get("display_name") or row.get("label")
            aliases = row.get("aliases") or []
            if (not isinstance(name, str) or not name.strip() or not isinstance(row.get("profile_id"), str) or not row["profile_id"].strip()
                    or not isinstance(aliases, list) or any(not isinstance(a, str) for a in aliases)):
                raise StationLookupError("invalid_station_record")
            if row["profile_id"] in seen_ids:
                raise StationLookupError("duplicate_station_id")
            seen_ids.add(row["profile_id"])
            row["display_name"] = name.strip()
            row["aliases"] = list(dict.fromkeys(a.strip() for a in aliases if a.strip()))
            families.setdefault(row.get("profile_key") or row["profile_id"], []).append(row)
        self._rows = []
        for rows in families.values():
            active = [r for r in rows if r.get("effective_status") == "active"]
            self._rows.extend(active if active else [max(rows, key=lambda r: int(r.get("version", 0)))])
        self._active = [r for r in self._rows if r.get("effective_status") == "active"]
        self._tokens = {}
        for row in self._active:
            for token in {_token(t) for t in [row["display_name"], *row["aliases"]]}:
                self._tokens.setdefault(token, {})[row["profile_id"]] = row
        self.conflicts = {k: sorted(v) for k, v in self._tokens.items() if len(v) > 1}
        ignored = {"logical_slot", "bound_slots", "resolved_logical_slot", "resolution_source",
                   "resolution_source"}
        identity = []
        for row in sorted(self._rows, key=lambda r: r["profile_id"]):
            value = {k: v for k, v in row.items() if k not in ignored}
            value["planner_mode"] = "cartesian" if row.get("planner_mode") in {None, "hybrid", "cartesian"} else row.get("planner_mode")
            identity.append(value)
        try:
            encoded = json.dumps({"stations": identity, "default_profile_id": self.default_profile_id}, sort_keys=True, ensure_ascii=False, allow_nan=False).encode()
        except (TypeError, ValueError) as exc:
            raise StationLookupError("non_json_or_nonfinite_station_data") from exc
        self.revision = hashlib.sha256(encoded).hexdigest()

    def _valid_catalog(self):
        if self.conflicts:
            raise StationLookupError("station_catalog_name_or_alias_conflict")

    def prompt_stations(self):
        self._valid_catalog()
        return [{"name": r["name"], "aliases": deepcopy(r["aliases"])} for r in self.status()["stations"] if r["ready"]]

    def resolve_name(self, canonical_name):
        self._valid_catalog()
        matches = [r for r in self._rows if r["display_name"] == canonical_name]
        if len(matches) != 1:
            raise StationLookupError("station_name_ambiguous" if matches else "station_not_registered")
        if matches[0].get("effective_status") != "active":
            raise StationLookupError("station_not_ready")
        return deepcopy(matches[0])

    def resolve_answer(self, name_or_alias):
        self._valid_catalog()
        if not isinstance(name_or_alias, str):
            raise StationLookupError("station_not_registered")
        matches = self._tokens.get(_token(name_or_alias), {})
        if len(matches) != 1:
            raise StationLookupError("station_not_registered")
        return deepcopy(next(iter(matches.values())))

    def direct_place(self, canonical_name):
        row = self.resolve_name(canonical_name)
        if row.get("planner_mode") not in {None, "hybrid", "cartesian", "joint"}:
            raise StationLookupError("invalid_station_control_mode")
        mode = "hybrid"
        data = {"profile_id": row["profile_id"], "label": row["display_name"], "planner_mode": mode,
                "pre_place_pose_mm_deg": _six(row.get("pre_place_pose_mm_deg"), "pre_place_pose"),
                "place_pose_mm_deg": _six(row.get("place_pose_mm_deg"), "place_pose"),
                "retreat_pose_mm_deg": _six(row.get("retreat_pose_mm_deg"), "retreat_pose"),
                "transfer_mode": 0, "place_mode": 1, "retreat_mode": 1}
        data["preferred_ik_reference_angles_deg"] = deepcopy(row.get("preferred_ik_reference_angles_deg")) if row.get("preferred_ik_enabled") else None
        data["pre_place_joint_angles_deg"] = deepcopy(row.get("pre_place_joint_angles_deg"))
        source = row.get("placement_source")
        data["placement_source"] = deepcopy(source) if isinstance(source, dict) else {"type":"manual_entry"}
        data["clear_path_acknowledged"] = row.get("clear_path_acknowledged") is True
        data["destination_check_policy"] = ("operator_taught" if data["placement_source"].get("type")
            in ("robot_teaching", "robot_pose_read") and data["clear_path_acknowledged"] else "rgbd_envelope")
        data["legacy_transit_policy"] = "retained_asset_not_executed_by_hybrid_transfer"
        data["legacy_transit_present"] = row.get("safe_transit_joint_angles_deg") is not None
        if data["pre_place_joint_angles_deg"] is not None:
            data["pre_place_joint_angles_deg"] = _six(data["pre_place_joint_angles_deg"], "pre_place_angles")
        if row.get("planner_mode") == "joint":
            data["place_joint_angles_deg"] = _six(row.get("place_joint_angles_deg"), "place_angles")
            data["retreat_joint_angles_deg"] = _six(row.get("retreat_joint_angles_deg"), "retreat_angles")
        return SimpleNamespace(**data)

    def status(self):
        stations = []
        for r in self._rows:
            reason = None
            try:
                self.direct_place(r["display_name"])
            except ValueError as exc:
                reason = str(exc)
            except RuntimeError as exc:
                reason = str(exc)
            stations.append({"name": r["display_name"], "aliases": deepcopy(r["aliases"]),
                "profile_id": r["profile_id"], "status": r.get("effective_status"), "ready": reason is None, "reason": reason})
        default = next((s for s in stations if s["profile_id"] == self.default_profile_id and s["ready"]), None)
        return {"revision": self.revision, "ready_count": sum(r["ready"] for r in stations),
                "default_profile_id":self.default_profile_id, "default_station":default["name"] if default else None, "default_ready":default is not None,
                "conflicts": deepcopy(self.conflicts), "stations": stations}
