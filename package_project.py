#!/usr/bin/env python3
"""依明確白名單打包目前專案；只使用 Python 標準函式庫。

雙擊 Package_Project.cmd，或執行 python package_project.py。
執行 python package_project.py --check 可檢查白名單而不建立壓縮檔。
新增交付檔案時，請將相對路徑加入 package_whitelist.json 的 files。
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import sys
import tempfile
from time import perf_counter
import uuid
import zipfile


PROJECT_ROOT = Path(__file__).resolve().parent
MANIFEST_NAME = "PACKAGING_MANIFEST.json"
TAIPEI = timezone(timedelta(hours=8), "Asia/Taipei")
FORBIDDEN_DIRS = {
    ".git", ".codex", ".agents", "__pycache__", ".pytest_cache",
    "venv", "backups", "runs", "development_runs", "acceptance_runs",
    "snapshots", "secrets", "credentials", "_deprecated_v1", "_archive", "node_modules", "operator_data",
}


def load_selection(root: Path, whitelist: Path) -> list[tuple[str, Path]]:
    """先檢查全部路徑；任一錯誤均停止，不會產生不完整的 ZIP。"""
    root = root.resolve()
    data = json.loads(whitelist.read_text(encoding="utf-8-sig"))
    if (not isinstance(data, dict) or type(data.get("schema_version")) is not int
            or data["schema_version"] != 1):
        raise ValueError("白名單 schema_version 必須為 1")
    names = data.get("files")
    if not isinstance(names, list) or not names:
        raise ValueError("白名單 files 必須是非空的檔案路徑陣列")
    # Verification scripts need a second explicit opt-in. This does not admit
    # arbitrary tests, test outputs, credentials, links, or out-of-root paths.
    verification = data.get('verification_files', [])
    if (not isinstance(verification, list)
            or any(not isinstance(name, str) or len(PurePosixPath(name).parts) != 2
                   or PurePosixPath(name).parts[0] != 'tests'
                   or PurePosixPath(name).suffix not in {'.py','.js','.cjs'} for name in verification)):
        raise ValueError('verification_files 必須明列 tests/ 下的驗證腳本')
    if len(set(verification)) != len(verification) or any(name not in names for name in verification):
        raise ValueError('verification_files 必須無重複且同時存在 files')

    selected = []
    seen = set()
    errors = []
    for name in names:
        if not isinstance(name, str) or not name:
            errors.append(f"無效的檔案路徑：{name!r}")
            continue
        relative = PurePosixPath(name)
        if (not relative.parts or relative.is_absolute() or relative.as_posix() != name
                or ".." in relative.parts or "\\" in name or ":" in name
                or any(char in name for char in "*?\x00")):
            errors.append(f"請填寫明確的相對檔案路徑（使用 /）：{name}")
            continue
        key = name.casefold()
        if key in seen:
            errors.append(f"白名單重複路徑：{name}")
            continue
        seen.add(key)
        if key == MANIFEST_NAME.casefold():
            errors.append(f"{MANIFEST_NAME} 由打包工具產生，不能加入白名單")
            continue
        parts = [part.casefold() for part in relative.parts]
        if (any(part in FORBIDDEN_DIRS or part.startswith((".venv", ".tmp", "project_handoff_"))
                for part in parts)
                or relative.name.casefold().startswith(".env")
                or relative.suffix.casefold() in {".pyc", ".pyo", ".tmp", ".bak", ".key", ".pem"}):
            errors.append(f"禁止交付快取、暫存或憑證檔案：{name}")
            continue
        if parts[0] == "tests" and not (
                len(parts) == 3 and parts[1] == "data" and relative.suffix.casefold() == ".json"
                or name in verification):
            errors.append(f"tests 只允許明列示範 JSON 或 verification_files：{name}")
            continue
        source = root.joinpath(*relative.parts)
        current = root
        linked = False
        for part in relative.parts:
            current = current / part
            if current.is_symlink() or (hasattr(current, "is_junction") and current.is_junction()):
                linked = True
                break
        if linked or not source.resolve().is_relative_to(root):
            errors.append(f"不允許連結或專案範圍外的路徑：{name}")
        elif not source.is_file():
            errors.append(f"必要檔案不存在或不是檔案：{name}")
        else:
            selected.append((name, source))
    if errors:
        raise ValueError("白名單檢查失敗：\n" + "\n".join(f"  - {error}" for error in errors))
    return sorted(selected, key=lambda item: item[0].casefold())


def verify_package(archive: Path, project_name: str, records: dict) -> None:
    """重新讀取 ZIP，檢查精確檔案集合、CRC 與每一份內容的雜湊。"""
    prefix = project_name + "/"
    with zipfile.ZipFile(archive) as bundle:
        expected = {prefix + name for name in records} | {prefix + MANIFEST_NAME}
        actual = bundle.namelist()
        if len(actual) != len(expected) or set(actual) != expected:
            raise ValueError("ZIP 檔案清單與白名單不一致")
        bad = bundle.testzip()
        if bad:
            raise ValueError(f"ZIP CRC 檢查失敗：{bad}")
        manifest = json.loads(bundle.read(prefix + MANIFEST_NAME))
        if manifest["files"] != records or manifest["source_file_count"] != len(records):
            raise ValueError("ZIP 內的雜湊清單不一致")
        for name, expected_record in records.items():
            content = bundle.read(prefix + name)
            if (len(content) != expected_record["bytes"]
                    or hashlib.sha256(content).hexdigest() != expected_record["sha256"]):
                raise ValueError(f"ZIP SHA-256 檢查失敗：{name}")


def build_package(root: Path, selected: list[tuple[str, Path]], output_dir: Path) -> Path:
    root = root.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    created = datetime.now(TAIPEI)
    filename = f"{root.name}_Clean_{created:%Y%m%d_%H%M%S}_{uuid.uuid4().hex[:8]}.zip"
    output = output_dir / filename
    descriptor, temporary_name = tempfile.mkstemp(prefix=".visionpick_package_", suffix=".tmp", dir=output_dir)
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        records = {}
        with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED) as bundle:
            for name, source in selected:
                content = source.read_bytes()
                records[name] = {"bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()}
                bundle.writestr(root.name + "/" + name, content)
            manifest = {
                "schema_version": 1,
                "project": root.name,
                "created_at": created.isoformat(timespec="seconds"),
                "selection": "explicit_whitelist",
                "source_file_count": len(records),
                "files": records,
            }
            bundle.writestr(root.name + "/" + MANIFEST_NAME,
                            json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
        verify_package(temporary, root.name, records)
        for name, source in selected:
            if hashlib.sha256(source.read_bytes()).hexdigest() != records[name]["sha256"]:
                raise ValueError(f"打包期間來源檔案變更，請重新打包：{name}")
        if output.exists():
            raise FileExistsError(f"輸出檔案已存在：{output}")
        temporary.rename(output)
        return output
    finally:
        temporary.unlink(missing_ok=True)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="只檢查白名單，不建立 ZIP")
    parser.add_argument("--whitelist", type=Path, default=PROJECT_ROOT / "package_whitelist.json",
                        help="白名單 JSON 路徑")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT.parent,
                        help="輸出目錄，預設為專案的上一層")
    args = parser.parse_args(argv)
    start = perf_counter()
    try:
        selected = load_selection(PROJECT_ROOT, args.whitelist)
        total_bytes = sum(source.stat().st_size for _, source in selected)
        print(f"白名單檢查通過：{len(selected)} 個來源檔案，{total_bytes / 1024**2:.2f} MiB")
        if args.check:
            print("僅檢查，未建立 ZIP。新增必要檔案時請更新 package_whitelist.json。")
            return 0
        output = build_package(PROJECT_ROOT, selected, args.output_dir)
        print(f"ZIP：{output}")
        print(f"大小：{output.stat().st_size / 1024**2:.2f} MiB")
        print(f"驗證通過：{len(selected)} 份來源檔案 SHA-256、ZIP CRC；另附 1 份 {MANIFEST_NAME}")
        print(f"耗時：{perf_counter() - start:.2f} 秒")
        return 0
    except (OSError, ValueError, RuntimeError, zipfile.BadZipFile) as exc:
        print(f"打包失敗：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
