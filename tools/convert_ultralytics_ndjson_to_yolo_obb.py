#!/usr/bin/env python3
"""
Convert an Ultralytics Platform OBB NDJSON snapshot into a standard
local YOLO OBB dataset.

Default project layout:
    datasets/raw/**/rgb_*.jpg
    datasets/robot_tools_v0/annotations/robot-tools-v0-obb.ndjson

Generated:
    datasets/robot_tools_v0/images/train
    datasets/robot_tools_v0/images/val
    datasets/robot_tools_v0/images/test
    datasets/robot_tools_v0/labels/train
    datasets/robot_tools_v0/labels/val
    datasets/robot_tools_v0/labels/test
    datasets/robot_tools_v0/conversion_report.json
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from collections import Counter, defaultdict
from pathlib import Path


VALID_IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
VALID_SPLITS = {"train", "val", "test"}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--ndjson",
        default="datasets/robot_tools_v0/annotations/robot-tools-v0-obb.ndjson",
        help="Ultralytics Platform NDJSON export.",
    )
    p.add_argument(
        "--raw-root",
        default="datasets/raw",
        help="Root directory containing the original RGB images. Searched recursively.",
    )
    p.add_argument(
        "--output",
        default="datasets/robot_tools_v0",
        help="Output YOLO OBB dataset root.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate NDJSON and image matching without writing/copying files.",
    )
    p.add_argument(
        "--reset-generated",
        action="store_true",
        help="Delete ONLY generated images/{train,val,test} and labels/{train,val,test} before conversion.",
    )
    return p.parse_args()


def load_ndjson(path: Path):
    header = None
    images = []
    with path.open("r", encoding="utf-8-sig") as f:
        for line_no, line in enumerate(f, 1):
            if not line.strip():
                continue
            obj = json.loads(line)
            if obj.get("type") == "dataset":
                if header is not None:
                    raise ValueError(f"Multiple dataset headers found (line {line_no}).")
                header = obj
            elif obj.get("type") == "image":
                images.append(obj)
    if header is None:
        raise ValueError("NDJSON dataset header not found.")
    if header.get("task") != "obb":
        raise ValueError(f"Expected task='obb', got {header.get('task')!r}.")
    return header, images


def build_image_index(raw_root: Path):
    matches = defaultdict(list)
    if not raw_root.exists():
        return matches
    for p in raw_root.rglob("*"):
        if p.is_file() and p.suffix.lower() in VALID_IMAGE_EXTS:
            matches[p.name].append(p)
    return matches


def validate_annotation(ann, class_ids, image_name, ann_no):
    if not isinstance(ann, list) or len(ann) != 9:
        raise ValueError(
            f"{image_name}: OBB #{ann_no} must contain class_id + 8 coordinates; got {ann!r}"
        )
    cls = int(ann[0])
    if cls not in class_ids:
        raise ValueError(f"{image_name}: unknown class id {cls}.")
    coords = [float(x) for x in ann[1:]]
    for c in coords:
        if not math.isfinite(c) or c < 0.0 or c > 1.0:
            raise ValueError(f"{image_name}: invalid normalized coordinate {c}.")
    return cls, coords


def reset_generated(output: Path):
    for kind in ("images", "labels"):
        for split in ("train", "val", "test"):
            d = output / kind / split
            if d.exists():
                for child in d.iterdir():
                    if child.name == ".gitkeep":
                        continue
                    if child.is_file() or child.is_symlink():
                        child.unlink()
                    elif child.is_dir():
                        shutil.rmtree(child)


def main():
    args = parse_args()
    ndjson = Path(args.ndjson)
    raw_root = Path(args.raw_root)
    output = Path(args.output)

    header, records = load_ndjson(ndjson)
    class_names_raw = header.get("class_names", {})
    class_names = {int(k): str(v) for k, v in class_names_raw.items()}
    class_ids = set(class_names)

    image_index = build_image_index(raw_root)

    if args.reset_generated and not args.dry_run:
        reset_generated(output)

    split_counts = Counter()
    class_counts = Counter()
    missing = []
    ambiguous = []
    validated = []

    for rec_no, rec in enumerate(records, 1):
        filename = rec.get("file")
        split = str(rec.get("split", "")).lower()
        if split not in VALID_SPLITS:
            raise ValueError(f"{filename}: unsupported split {split!r}.")

        candidates = image_index.get(filename, [])
        if len(candidates) == 0:
            missing.append(filename)
            continue
        if len(candidates) > 1:
            ambiguous.append((filename, [str(x) for x in candidates]))
            continue

        anns = rec.get("annotations", {}).get("obb", [])
        if not isinstance(anns, list):
            raise ValueError(f"{filename}: annotations.obb is not a list.")

        label_rows = []
        for i, ann in enumerate(anns, 1):
            cls, coords = validate_annotation(ann, class_ids, filename, i)
            class_counts[cls] += 1
            row = f"{cls} " + " ".join(f"{c:.8f}" for c in coords)
            label_rows.append(row)

        split_counts[split] += 1
        validated.append((rec, candidates[0], label_rows))

    print("=" * 72)
    print("Ultralytics NDJSON -> YOLO OBB")
    print("=" * 72)
    print(f"Dataset : {header.get('name')}")
    print(f"Task    : {header.get('task')}")
    print(f"Images  : {len(records)}")
    print(f"Classes : {class_names}")
    print(f"Splits  : {dict(split_counts)}")
    print(f"Instances: {dict(class_counts)}")
    print(f"Missing local images   : {len(missing)}")
    print(f"Ambiguous local images : {len(ambiguous)}")

    if missing:
        print("\nMissing examples:")
        for x in missing[:20]:
            print("  -", x)
    if ambiguous:
        print("\nAmbiguous filenames:")
        for name, paths in ambiguous[:10]:
            print("  -", name)
            for x in paths:
                print("      ", x)

    if missing or ambiguous:
        raise SystemExit(
            "\nConversion stopped. Fix missing/duplicate image filenames under --raw-root first."
        )

    if args.dry_run:
        print("\nDRY RUN PASS: NDJSON and all local image matches are valid.")
        return

    for split in VALID_SPLITS:
        (output / "images" / split).mkdir(parents=True, exist_ok=True)
        (output / "labels" / split).mkdir(parents=True, exist_ok=True)

    for rec, source_image, label_rows in validated:
        split = str(rec["split"]).lower()
        filename = str(rec["file"])

        dst_image = output / "images" / split / filename
        dst_label = output / "labels" / split / (Path(filename).stem + ".txt")

        shutil.copy2(source_image, dst_image)
        dst_label.write_text(
            ("\n".join(label_rows) + ("\n" if label_rows else "")),
            encoding="utf-8",
            newline="\n",
        )

    report = {
        "source_ndjson": str(ndjson),
        "raw_root": str(raw_root),
        "output": str(output),
        "dataset_name": header.get("name"),
        "task": header.get("task"),
        "classes_from_ndjson": class_names,
        "image_counts": dict(split_counts),
        "instance_counts": {str(k): v for k, v in sorted(class_counts.items())},
        "total_images": len(records),
    }
    (output / "conversion_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False),
        encoding="utf-8",
        newline="\n",
    )

    print("\nCONVERSION PASS")
    print(f"Dataset written to: {output.resolve()}")
    print("Next:")
    print("  python tools/validate_yolo_obb_dataset.py")


if __name__ == "__main__":
    main()
