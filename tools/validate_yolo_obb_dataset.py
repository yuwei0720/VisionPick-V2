#!/usr/bin/env python3
"""Basic QA for the generated local YOLO OBB dataset."""

from __future__ import annotations

from collections import Counter
from pathlib import Path
import math
import sys

DATASET = Path("datasets/robot_tools_v0")
CLASS_NAMES = {0: "pen", 1: "battery", 2: "screwdriver"}
SPLITS = ("train", "val", "test")
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def main():
    errors = []
    image_counts = Counter()
    instance_counts = Counter()

    for split in SPLITS:
        image_dir = DATASET / "images" / split
        label_dir = DATASET / "labels" / split

        images = {
            p.stem: p for p in image_dir.iterdir()
            if p.is_file() and p.suffix.lower() in IMAGE_EXTS
        } if image_dir.exists() else {}

        labels = {
            p.stem: p for p in label_dir.glob("*.txt")
        } if label_dir.exists() else {}

        image_counts[split] = len(images)
        if not image_dir.is_dir() or not label_dir.is_dir():
            errors.append(f"{split}: required image/label directory is missing")
        if not images:
            errors.append(f"{split}: no images found; an absent/empty dataset is not a PASS")

        for stem in sorted(images.keys() - labels.keys()):
            errors.append(f"{split}: image without label: {images[stem].name}")
        for stem in sorted(labels.keys() - images.keys()):
            errors.append(f"{split}: label without image: {labels[stem].name}")

        for stem in sorted(images.keys() & labels.keys()):
            label_path = labels[stem]
            for line_no, line in enumerate(label_path.read_text(encoding="utf-8").splitlines(), 1):
                if not line.strip():
                    continue
                parts = line.split()
                if len(parts) != 9:
                    errors.append(f"{label_path}:{line_no}: expected 9 fields, got {len(parts)}")
                    continue
                try:
                    cls = int(parts[0])
                    coords = [float(x) for x in parts[1:]]
                except ValueError:
                    errors.append(f"{label_path}:{line_no}: non-numeric field")
                    continue
                if cls not in CLASS_NAMES:
                    errors.append(f"{label_path}:{line_no}: invalid class id {cls}")
                for c in coords:
                    if not math.isfinite(c) or not (0.0 <= c <= 1.0):
                        errors.append(f"{label_path}:{line_no}: coordinate outside [0,1]: {c}")
                instance_counts[cls] += 1

    print("Dataset:", DATASET.resolve())
    print("Images:", dict(image_counts))
    print("Instances:", {
        CLASS_NAMES.get(k, str(k)): v for k, v in sorted(instance_counts.items())
    })

    if errors:
        print(f"\nFAIL: {len(errors)} dataset error(s)")
        for err in errors[:50]:
            print(" -", err)
        sys.exit(1)

    print("\nPASS: local YOLO OBB dataset structure and label syntax are valid.")


if __name__ == "__main__":
    main()
