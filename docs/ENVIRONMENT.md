# VisionPick V2 執行與驗證環境

## 本次實際驗證環境

| Item               | Value       |
|--------------------|-------------|
| Operating System   | Windows     |
| Python             | 3.14（本機） |
| 前端執行 | 原生 HTML／CSS／ES6；不需 Node 建置 |
| 瀏覽器驗收 | Node Playwright + Edge headless |

## Install for normal development

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements.txt
python -m pip install -r requirements-dev.txt
```

## 舊環境鎖定檔

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements-lock.txt
```

## Verified Tests

```powershell
python -m compileall . -q
python -m pytest -v
```

舊 19 項 V0 測試數不作為目前 V2 驗收依據。此次測試指令、通過數與限制見 [V2_VERIFICATION.md](V2_VERIFICATION.md)。`requirements-lock.txt` 保留為原工作站的套件參考，並非本次重新產生的鎖定檔。

## Important

- `.venv` is local only and must not be committed.
- `requirements.txt` defines supported development dependencies (minimum versions).
- `requirements-lock.txt` records the exact V0 environment that was verified.
- RealSense, YOLO model inference, and MyCobot hardware operation require
  separate hardware validation; passing software tests does not establish physical arrival or pick/place success.
