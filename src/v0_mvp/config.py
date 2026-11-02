from __future__ import annotations
from pathlib import Path
import yaml


def project_root() -> Path:
    return Path(__file__).resolve().parents[2]


def resolve_path(value: str | Path, root: Path | None = None) -> Path:
    p = Path(value)
    if p.is_absolute():
        return p
    return (root or project_root()) / p


def load_config(path: str | Path) -> tuple[dict, Path]:
    root = project_root()
    p = Path(path)
    if not p.is_absolute():
        p = root / p
    if not p.exists():
        raise FileNotFoundError(f"找不到設定檔: {p}")
    with p.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError("設定檔根節點必須是 mapping")
    return cfg, root
