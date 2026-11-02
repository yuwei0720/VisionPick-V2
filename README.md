# VisionPick V2

以自然語言安排桌面抓放工作的本機工作台：V2 任務解析、RGB-D／OBB 幾何、具名站點與模組化機械臂控制。前端使用原生 HTML、CSS、ES6 JavaScript，沒有建置程序。

桌面介面維持一頁式相機、步驟進度、解析明細與人工確認；工程抽屜提供「站點與標定」「抓取與幾何」「運動與夾爪」「診斷與紀錄」四個分頁。

## 架構與目錄

```text
run_v0.py → operator_console.cli → service_v2 + server
                                    └→ runtime_v2
                                        ├→ task_parser_v2 / task_pending_v2 / task_sequence_v2
                                        ├→ place_profiles / place_catalog_v2 / place_capture / table_surface
                                        └→ direct_control_v2 / hybrid_transfer / robot_adapter
```

| 目錄 | 用途 |
|---|---|
| `src/operator_console/` | V2 任務、站點、標定、資產與 HTTP 工作台 |
| `src/operator_console/static/v2/` | 無編譯依賴的一頁式介面 |
| `src/v0_mvp/` | 相機、18 點剛體標定、OBB 深度幾何及底層通訊；名稱保留相容於既有工具 |
| `src/integration/` | 共用相對放置與場景幾何；不載入 V1 任務模型 |
| `src/voice_input/` | 語音擷取與逐字稿 |
| `src/planning/` | 可選的規劃驗證合約與模型工具 |
| `config/`、`calibration/`、`models/` | 設備設定、標定資料及偵測權重 |
| `tests/`、`tools/` | 離線測試與標定／依賴盤點工具 |
| `operator_data/`、`runs/` | 本機生成的站點、桌面、指紋與紀錄；Git 忽略 |

V1 runtime、10 欄位流程、舊多步業務、A/B/C 槽位與 `/legacy/` 已退出正式執行路徑。本機 `_deprecated_v1/` 僅供備查，不納入 Git 或交付白名單。[拆分盤點](docs/V2_MIGRATION_AUDIT.md)記錄來源行數與保留邊界。

## 安裝

建議使用 Python 3.10 以上，並確認所選 Python 版本有 RealSense、PyTorch 與機械臂 SDK 對應套件。

```powershell
python -m venv .venv-console
.venv-console\Scripts\Activate.ps1
python -m pip install -r requirements-console.txt
python -m pip install -r requirements-dev.txt
```

實機環境另安裝 `requirements.txt`。偵測權重位於 `models/best.pt`；相機、COM 埠、關節初始角與標定矩陣由 `config/v0.yaml` 設定。既有校準屬於原工作站，更換相機位置或工作站後須重新執行 18 點標定與桌面量測。

Gemini 金鑰只透過目前程序的環境變數提供，不寫入原始碼：

```powershell
$env:GEMINI_API_KEY = '<你的金鑰>'
```

本機相機或手臂未連線時，仍可使用離線固定案例檢視介面。離線結果使用合成影像與控制替身，不能作為實物抓放成功率。

## 啟動與操作

```powershell
python run_v0.py ui
# 僅啟用相機
python run_v0.py ui --hardware
# 實機完整模式，亦可雙擊 Launch_Full.cmd
python run_v0.py ui --hardware --enable-motion
```

開啟 [本機工作台](http://127.0.0.1:8765)。服務只綁定 loopback；寫入請求檢查同源 JSON 與工作階段 token。

1. 開啟工程模式的「站點與標定」。輸入站點名稱與別名，確認現場路徑淨空。
2. 示教站點：調整實機位置，讀取目前 TCP 與六軸角，再儲存。需要手動拖動時，先人工確認空手並托住手臂，再進入 Free mode；讀取後先離開示教恢復扭力，再儲存。實測 Rx/Ry/Rz 和配對放置點關節角原樣保存，不推造上方接近點的配對角。
3. ROI 站點：選擇新影格或 60 秒內最近實機 RGB-D，按「在左側畫布框選落點」。拖曳或輸入像素座標，核對 XYZ／有效深度比例，再確認儲存。0/90/180/270° 顯示旋轉由後端逆轉換；Rx/Ry 預設 −175°/−5°，Rz 可自行指定。
4. 桌面標定：確認中央區域為空的水平桌面後一鍵量測，或指定桌面 ROI；也可輸入現場量測 Z。更新後同步原有效站點的運動指紋。校準變更及原已失效站點不會被自動重新啟用。
5. 依需要調整抓取、幾何、運動參數。輸入自然語言、核對解析明細及追問，再確認執行。完成後仍需現場核對物件結果。

儲存站點不會移動機械臂。示教站點的目的地淨空由操作者確認；ROI 站點保留 RGB-D 包絡檢查。兩者均不代表系統完成全路徑動態碰撞驗證。

## V2 標定 API

| 端點 | 用途 |
|---|---|
| `GET /api/v2/places` | 具名站點、ready 狀態及版本指紋 |
| `POST /api/v2/robot/pose` | 讀取配對 TCP／六軸角，不送運動命令 |
| `POST /api/v2/robot/teaching` | 明確進入／離開 Free mode |
| `POST /api/v2/places/manage` | teach、metadata、activate/revalidate、disable、delete、default |
| `POST /api/v2/places/capture` | 凍結新／目前 RGB-D 影格 |
| `POST /api/v2/places/roi_capture` | 以 `capture_id`、`roi_xyxy`、`rotation_deg` 預覽或儲存 ROI 站點 |
| `GET /api/v2/places/captures/{id}/frame` | 框選用原始影格 |
| `POST /api/v2/table-surface/calibrate` | RGB-D／指定 ROI／手動 Z 桌面標定 |
| `POST /api/v2/robot/home`、`POST /api/v2/recover-empty` | V2 原生復位與現場空手確認 |

`roi_xyxy` 使用旋轉後畫面的像素邊界座標，寬高按旋轉交換；可反向拖曳。API 以保存的深度重新計算 XYZ，不接受前端直接提供的 XYZ。預覽 `save:false`，確認儲存 `save:true` 並附名稱、別名與 `clear_path_acknowledged:true`。

## 驗證與交付

```powershell
python -m pytest -q
node --test tests/test_placement_preview_v2.js tests/test_v2_system_repairs.js tests/test_native_calibration_v2.js
python tools/audit_v2_dependencies.py --output docs/V2_DEPENDENCY_AUDIT.json
python package_project.py --check
```

瀏覽器驗收使用 Node Playwright 與 Edge。可在測試環境安裝 `npm install --no-save playwright`；前端執行不需要 Node 套件。完整測試包含瀏覽器四分頁、1440×900 零主頁捲動及旋轉 ROI 拖曳驗證；驗收圖另由 `tests/native_calibration_v2_browser.cjs` 產生。

本機 `operator_data/` 不隨 Git 上傳，乾淨副本會建立空站點清單；必須建立本站點、量測桌面並確認空手才能運動。測試使用暫存目錄及控制替身，沒有從測試送出實機運動或付費模型請求。

本次只整理與驗證，不建立遠端、不上傳。[驗證紀錄](docs/V2_VERIFICATION.md)區分軟體通過結果與尚待實機核對事項。
