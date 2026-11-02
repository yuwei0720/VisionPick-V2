"""Shared grasp admission catalog. tools.json is the single object registry.

Model detections still supply the actual visible classes; this catalog is not
permission to move, nor evidence that a class has passed a hardware grasp test.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def load_catalog(root=ROOT):
    return json.loads((Path(root) / "tools.json").read_text(encoding="utf-8"))


def class_display_name(name):
    aliases = load_catalog().get(name, {}).get("aliases", [])
    return aliases[0] if aliases else name


def graspable_classes(library=None):
    library = load_catalog() if library is None else library
    return sorted(name for name, item in library.items()
                  if item.get("graspable") is True
                  and item.get("detector_class") == name)
