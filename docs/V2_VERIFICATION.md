# VisionPick V2 驗證紀錄

日期：2026-11-02（Asia/Taipei）。只整理與本機驗證，不建立遠端、不上傳。

## 測試基線

- 修改前：`python -m pytest -q`，771 passed、0 failed、0 errors，232.41 秒。
- 修改前 Node：24 passed、0 failed。
- 原副本無 Git 儲存庫；本次建立本機 Git 儲存庫，未設定遠端。

## 最終驗證

- 最終完整：`python -m pytest -q --junitxml=development_runs/v2_native_20261102/full_verified.xml`，**808 passed、0 failed、0 errors、0 skipped**，294.59 秒。2 則第三方套件棄用警告（pymycobot locale、google-genai typing），沒有測試失敗。

- Node：`node --test tests/test_placement_preview_v2.js tests/test_v2_system_repairs.js tests/test_native_calibration_v2.js`：28 passed、0 failed、0 skipped。
- Python 編譯：`python -m compileall -q src run_v0.py` 通過。
- AST 盤點：58 個來源模組、442 個 import 節點、13 個 Python 測試檔；正式入口可達 42 個模組；V1 禁用業務 import 0。完整清單見 `V2_DEPENDENCY_AUDIT.json`。

## 無頭瀏覽器

使用 Edge／Playwright、1440×900、640×480 受控 RGB-D，不授權實體運動。8 張截圖與機器可讀紀錄在本機 `development_runs/v2_native_20261102/browser/`。

| 驗證 | 結果 |
|---|---|
| 四分頁切換及鍵盤 Arrow／Home／End | 只有一個 tabpanel 可見，標籤狀態正確 |
| Tab 1／2／3／4 主頁高度 | `scrollHeight=900`、`clientHeight=900` |
| 桌面寬度 1050／1440／1920，高度 900 | 主頁零捲動 |
| 0／90／180／270° 實際拖曳 | 均還原為原始 ROI `[160,80,319,239]`，Z 約 120 mm |
| ROI 預覽與儲存 | 有效深度 100%，保存為 active／ready 站點 |
| 機械臂示教讀值 | TCP Rx/Ry/Rz `[-172,-7,34]` 原樣保存；資料來自控制替身 |
| 鍵盤數字座標、Esc | 可預覽，取消後停用儲存 |
| 桌面 ROI | 完成儲存並同步原有效站點指紋 |
| HTTP 寫入 | 全部使用 `/api/v2/`；頁面 JS／console error 0 |

截圖檔：`tab-1-stations.png`、`tab-1-roi-0.png`、`tab-1-roi-90.png`、`tab-1-roi-180.png`、`tab-1-roi-270.png`、`tab-2-geometry.png`、`tab-3-motion.png`、`tab-4-diagnostics.png`。

## 版面掃描的界線

Impeccable 掃描記錄在本機 `design_detect.json`：69 筆風格／品質提示，包含既有主畫面的 49 筆小字提示、2 筆 tiny-text、1 筆 nested-cards 等。未宣稱 WCAG 全面驗收。新分頁使用原生可鍵盤操作標籤、文字狀態、12 px 以上說明字及數字框選替代；本次依需求保留原主畫面字級、密度與動畫。掃描將程式載入前沒有 src 的隱藏 GIF 欄位列為 broken-image；實際瀏覽器測試沒有影像載入錯誤。

## 交付與 Git

- `.gitignore` 排除工作站 `operator_data/`、退役備查、runs、暫存、虛擬環境、`.env`／憑證與 ZIP。
- 打包器本身另拒絕 `operator_data`、`_deprecated_v1`、`_archive` 與 `node_modules`，避免白名單誤加入。
- 本機 47 份退役來源的逐檔 SHA-256 收據在 `retired_files.json`；不放進 Git。
- 工作站資料收據涵蓋 14 個檔案，最後修改時間均早於此次工作；原 `config/v0.yaml` SHA-256 與本次修改前備份一致。未以來源檔 SHA-256 作為實物效果證據。
- 目前交付白名單的憑證格式掃描匹配 0；此檢查不宣稱涵蓋所有形式的秘密。
- 白名單 154 份來源檔案，21.71 MiB；ZIP 約 19.10 MiB。全部來源 SHA-256 與 ZIP CRC 核對通過，root `PACKAGING_MANIFEST.json` 記錄同一份交付清單。
- 乾淨解包不含 `operator_data/` 或 `_deprecated_v1/`：CLI help 通過；示教更新、ROI 儲存、桌面指紋、讀位姿及指定設定檔 5 項測試通過；Node 28 項通過。
- Git 只建立本機儲存庫與整理提交，不設定遠端、不上傳；收尾使用 `git status --porcelain=v1`、`git diff --check` 及 `git remote -v` 核對。

## 待現場驗證

實際相機 ROI 深度、Free mode 扭力釋放／恢復、串口關閉與故障恢復、實機抓放與復位尚待現場驗收。`active/ready`、離線測試通過與固定等待均不證明機械臂實際到位或物件結果。此次軟體驗證不代替這些項目。
