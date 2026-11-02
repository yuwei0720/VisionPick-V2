from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
import math
import shutil
import xml.etree.ElementTree as ET

import yaml


class VendorModelAuditError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    h = sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _floats(text: str | None, n: int, label: str) -> tuple[float, ...]:
    if text is None:
        raise VendorModelAuditError(f"{label} is missing")
    parts = text.split()
    if len(parts) != n:
        raise VendorModelAuditError(f"{label} must contain {n} values")
    try:
        out = tuple(float(x) for x in parts)
    except ValueError as exc:
        raise VendorModelAuditError(f"{label} must be numeric") from exc
    if not all(math.isfinite(x) for x in out):
        raise VendorModelAuditError(f"{label} must be finite")
    return out


def load_vendor_baseline(path: str | Path) -> dict:
    p = Path(path)
    data = yaml.safe_load(p.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("kind") != "visionpick_vendor_robot_baseline":
        raise VendorModelAuditError("unexpected vendor baseline schema")
    if data.get("exact_model") != "myCobot 280 M5":
        raise VendorModelAuditError("this audit currently supports only myCobot 280 M5")
    return data


def _package_mesh_path(uri: str, package_root: Path, description_package: str) -> Path | None:
    prefix = f"package://{description_package}/"
    if not uri.startswith(prefix):
        return None
    rel = uri[len(prefix):]
    if ".." in Path(rel).parts:
        raise VendorModelAuditError(f"unsafe mesh path in URDF: {uri}")
    return (package_root / rel).resolve()


def audit_vendor_urdf(urdf_path: str | Path, baseline: dict, *, package_root: str | Path | None = None) -> dict:
    urdf_path = Path(urdf_path).resolve()
    package_root_path = Path(package_root).resolve() if package_root is not None else None
    errors: list[str] = []
    warnings: list[str] = []

    if not urdf_path.exists():
        return {"status": "FAIL", "errors": [f"URDF not found: {urdf_path}"], "warnings": [], "artifacts": {}}

    try:
        root = ET.parse(urdf_path).getroot()
    except Exception as exc:
        return {"status": "FAIL", "errors": [f"URDF parse failed: {exc}"], "warnings": [], "artifacts": {}}

    robot = baseline.get("robot") or {}
    if root.tag != "robot":
        errors.append(f"root element is {root.tag!r}, expected 'robot'")
    expected_robot_name = str(robot.get("urdf_robot_name") or "")
    if expected_robot_name and root.attrib.get("name") != expected_robot_name:
        errors.append(f"robot name {root.attrib.get('name')!r} != expected {expected_robot_name!r}")

    links = {node.attrib.get("name"): node for node in root.findall("link") if node.attrib.get("name")}
    expected_links = [str(x) for x in robot.get("expected_links") or []]
    for name in expected_links:
        node = links.get(name)
        if node is None:
            errors.append(f"missing expected link: {name}")
            continue
        if (baseline.get("validation") or {}).get("require_collision_geometry_on_links") and node.find("collision/geometry") is None:
            errors.append(f"link has no collision geometry: {name}")

    joints = {node.attrib.get("name"): node for node in root.findall("joint") if node.attrib.get("name")}
    arm_joint_names = [str(x) for x in robot.get("arm_joint_names") or []]
    if len(arm_joint_names) != 6 or len(set(arm_joint_names)) != 6:
        errors.append("baseline arm_joint_names must contain six unique joints")

    tol = float((baseline.get("validation") or {}).get("joint_limit_tolerance_rad", 1e-4))
    axis_tol = float((baseline.get("validation") or {}).get("axis_tolerance", 1e-6))
    expected_joint_data = robot.get("joints") or {}
    for name in arm_joint_names:
        node = joints.get(name)
        expected = expected_joint_data.get(name) or {}
        if node is None:
            errors.append(f"missing expected revolute joint: {name}")
            continue
        if node.attrib.get("type") != "revolute":
            errors.append(f"joint {name} type={node.attrib.get('type')!r}, expected revolute")
        parent = node.find("parent")
        child = node.find("child")
        if parent is None or parent.attrib.get("link") != expected.get("parent"):
            errors.append(f"joint {name} parent mismatch")
        if child is None or child.attrib.get("link") != expected.get("child"):
            errors.append(f"joint {name} child mismatch")
        axis = node.find("axis")
        try:
            actual_axis = _floats(axis.attrib.get("xyz") if axis is not None else None, 3, f"joint {name} axis")
            expected_axis = tuple(float(x) for x in expected.get("axis") or [])
            if len(expected_axis) != 3 or any(abs(a - b) > axis_tol for a, b in zip(actual_axis, expected_axis)):
                errors.append(f"joint {name} axis {actual_axis} != expected {expected_axis}")
        except VendorModelAuditError as exc:
            errors.append(str(exc))
        limit = node.find("limit")
        if limit is None:
            errors.append(f"joint {name} has no limit")
        else:
            for attr, key in (("lower", "lower_rad"), ("upper", "upper_rad")):
                try:
                    actual = float(limit.attrib[attr])
                    expected_value = float(expected[key])
                    if abs(actual - expected_value) > tol:
                        errors.append(f"joint {name} {attr}={actual} != expected {expected_value}")
                except Exception:
                    errors.append(f"joint {name} has invalid {attr} limit")

    mesh_uris: list[str] = []
    for mesh in root.findall(".//mesh"):
        uri = str(mesh.attrib.get("filename") or "").strip()
        if uri and uri not in mesh_uris:
            mesh_uris.append(uri)

    artifacts = {"urdf": {"path": str(urdf_path), "sha256": _sha256(urdf_path)}}
    mesh_artifacts: list[dict] = []
    require_meshes = bool((baseline.get("validation") or {}).get("require_mesh_files_present"))
    description_package = str((baseline.get("source") or {}).get("description_package") or "mycobot_description")
    for uri in mesh_uris:
        if package_root_path is None:
            warnings.append(f"mesh existence not checked without package_root: {uri}")
            continue
        try:
            mesh_path = _package_mesh_path(uri, package_root_path, description_package)
        except VendorModelAuditError as exc:
            errors.append(str(exc))
            continue
        if mesh_path is None:
            warnings.append(f"external/non-package mesh URI not checked: {uri}")
            continue
        if not mesh_path.exists():
            if require_meshes:
                errors.append(f"mesh missing: {uri} -> {mesh_path}")
            else:
                warnings.append(f"mesh missing: {uri} -> {mesh_path}")
            continue
        mesh_artifacts.append({"uri": uri, "path": str(mesh_path), "sha256": _sha256(mesh_path), "size_bytes": mesh_path.stat().st_size})
    artifacts["meshes"] = mesh_artifacts

    return {
        "status": "PASS" if not errors else "FAIL",
        "model": baseline.get("exact_model"),
        "baseline_id": baseline.get("baseline_id"),
        "urdf_robot_name": root.attrib.get("name"),
        "base_link": robot.get("base_link"),
        "tip_link": robot.get("tip_link"),
        "arm_joint_names": arm_joint_names,
        "links_found": sorted(links),
        "mesh_uri_count": len(mesh_uris),
        "errors": errors,
        "warnings": warnings,
        "artifacts": artifacts,
    }


def snapshot_vendor_model(*, package_root: str | Path, urdf_path: str | Path, destination: str | Path) -> Path:
    """Copy only the M5 URDF directory into a reviewable local snapshot.

    This helper does not mark the snapshot verified. The caller must separately
    preserve the upstream license and complete hardware verification.
    """
    package_root = Path(package_root).resolve()
    urdf_path = Path(urdf_path).resolve()
    destination = Path(destination).resolve()
    try:
        rel = urdf_path.parent.relative_to(package_root)
    except ValueError as exc:
        raise VendorModelAuditError("URDF must live under package_root") from exc
    target = destination / rel
    if target.exists():
        shutil.rmtree(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(urdf_path.parent, target)
    return target
