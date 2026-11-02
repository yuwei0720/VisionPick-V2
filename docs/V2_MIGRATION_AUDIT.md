# VisionPick V2 拆分與標定整合盤點

日期：2026-11-02（Asia/Taipei）。工作根目錄為內層 `VisionPick_V2_Final_Version/`；外層報告與簡報資料不在修改範圍。原副本沒有 `.git`。

## 結論與範圍

正式入口為 `run_v0.py → cli → ConsoleServiceV2 → server / ConsoleRuntimeV2`。V1 任務 runtime、10 欄位語意模型、舊 2–3 步執行與 A/B/C 槽位不再被正式入口或底層共用庫 import。舊檔保留於本機 `_deprecated_v1/`，Git 與打包工具均排除。

盤點採 AST，包含函式內 import；完整掃描 `src/`、`tests/` 的 Python 檔與 `config/` 檔案清單。[機器可讀依賴圖](V2_DEPENDENCY_AUDIT.json)列出每個 import 的原始碼行數、正式入口可達模組及違規引用。這是 import 依賴盤點，不是完整呼叫圖、資料流圖或實機驗證。

修改前共有 77 個 `src` Python 模組、732 個 import 節點。原始碼備份與盤點保存在本機 `development_runs/v2_native_20261102/before.zip`、`imports_before.json`。修改後數量以目前機器可讀依賴圖為準。

## 原始耦合證據與處理

下表「原始行數」對應修改前備份；「目前行數」對應本次整合後來源。

| 原始耦合 | 原始檔案與行數 | 處理與目前來源 |
|---|---|---|
| 啟動時建構 V1 `ConsoleService` | `cli.py:110` | `cli.py` 改用獨立 `service_v2.py`，沒有 V1 繼承、轉呼叫或 import shim |
| HTTP transport import V1 錯誤類別 | `server.py:19` | 移除 import；V2 使用自身的 RuntimeError／ValueError 回應 |
| 舊前端與機械臂復位路由 | `server.py:106,276` | `/legacy/`、所有 `/api/…` V1 路由與根目錄舊靜態資源回傳 404；V2 GET 在目前 `server.py:95–140`，POST 在 `:142–191` |
| V2 讀取 V1 worker／示教／active robot 狀態 | `runtime_v2.py:315–318` | 改成 V2 `_maintenance_idle`，只檢查原生示教連線及隔離中的租約 |
| V2 實機前檢 import 舊 pipeline | `runtime_v2.py:700`、`doctor.py:30` | 將相同校準／RMSE 檢查搬為 `v0_mvp/motion_safety.py`；保留檢查內容 |
| 工程讀位姿呼叫 V1 | `engineering_v2.py:203–207` | `runtime_v2.py:322–350` 直接使用 V2 controller，讀 TCP／六軸角，finally 釋放或隔離租約 |
| 桌面儲存呼叫 V1 | `engineering_v2.py:274–276` | `runtime_v2.py:532–567` 原生量測、儲存與指紋同步 |
| 具名站點共用管理器反向載入舊目的地 resolver | `place_profiles.py:965–966` | 刪除該單元與槽位推斷／綁定／解析；舊 JSON 的槽位欄位只在讀入時移除，不更動本機原檔 |
| 共用站點管理仍執行 A/B/C 業務 | `place_profiles.py:28,120,468,490–492` 等，完整行數在原備份 | 移除槽位常數、helper、解析及綁定 API；V2 依唯一名稱／別名管理 |
| V2 parser 為讀取 JSON 登錄資料而載入舊模型 | `task_parser_v2.py:152,345` | 直接讀 V2 所需工具、世界實體與模型政策 JSON，不再載入 `AI_MVP.RobotTask` |
| 共用 integration package 的 lazy export 指向 V1 | `integration/__init__.py:18–19` | 移除舊語意 gateway export；保留幾何共用庫 |

## 拆分對照

| 分類 | 模組／功能 | 結果 |
|---|---|---|
| V2 原生 | TCP／六軸角讀取、Free mode 進出、空手確認、Start 復位 | 原生 V2 HTTP／runtime，沒有 V1 service |
| V2 原生 | 示教站點新增／版本更新／名稱與別名／預設／啟停／刪除 | `engineering_v2.station`，`runtime_v2` 裝配，交易失敗回復站點、資產與事件紀錄 |
| V2 原生 | ROI 擷取、四種旋轉逆轉換、XYZ／有效深度預覽與儲存 | `runtime_v2.capture_rgbd/_resolve_roi/roi_capture` 接上 `place_capture` |
| V2 原生 | 桌面中央 ROI、指定 ROI、手動 Z | `runtime_v2.table_calibrate`；更新桌面後同步原有效站點，原失效站點保持失效 |
| 保留共用 | `v0_mvp` 的 camera、calibration、calibration_reference、geometry、grasp、robot、types、core、config、detector | 實體設備與幾何核心；新共用 `motion_safety.py` 不載入任務模型 |
| 保留共用 | `integration.relative_placement/scene_grounding`、`operator_console.hybrid_transfer/robot_adapter/preferred_ik` | 目的地／OBB 幾何與底層運動。原專案的 `hybrid_transfer.py` 實際位於 `operator_console/`，未複製第二份 |
| 保留共用 | `voice_input/`、`place_profiles.py`、`place_capture.py`、`assets.py`、`table_surface.py` | 不反向 import V1 任務業務；`AssetRegistry` 類別原本位於 `assets.py`，沒有新增重複的 `asset_registry.py` |
| 退役備查 | `runtime.py`、`real_sequence.py`、`task_plan.py`、`task_plan_model.py`、`destination_resolver.py`、`semantic.py`、`resolution.py` 等 | 移出 `src`，只保留本機備查。逐檔 SHA-256 收據見下列本機紀錄 |
| 退役備查 | `AI_MVP.py`、`Task Schema.json`、`integration.semantic_gateway/v0_pick_adapter`、`v0_mvp.pipeline` | 舊 10 欄位任務業務退出正式來源 |
| 退役備查 | 舊 static、precision、relative_place_preview 頁面及舊 precision launcher | 無正式路由、無打包白名單項目 |

逐檔退役清單（含 47 個程式、頁面、啟動檔及舊操作文件）與原始雜湊保存在本機 `development_runs/v2_native_20261102/retired_files.json`。這些原始內容不納入 Git。

## Bug 與設計選擇

此次架構拆分與四分頁是使用者指定的設計變更。採用範圍如下，沒有另行更換技術棧或主畫面視覺系統。

| 方案 | 優點 | 缺點 | 適用場景 |
|---|---|---|---|
| 原生 V2＋本機退役備查（本次採用） | 正式執行零 V1 業務引用，保留原檔與雜湊便於核對 | 本機仍佔用備查檔空間 | 本次拆分、驗收與準備交付 |
| 原生 V2＋永久刪除 V1 | 本機目錄更少 | 失去逐檔原始實作與回復線索 | 已完成版本控制與另存歷史後的後續清理 |

| 方案 | 優點 | 缺點 | 適用場景 |
|---|---|---|---|
| 100vh 主畫面＋四分頁抽屜（使用者指定，本次採用） | 不更動主畫面四象限，各工程分頁固定高度、獨立捲動 | 站點分頁內仍有局部捲動 | 1440×900 桌面工作台 |
| 工程設定獨立頁面 | 可一次展開更多設定 | 離開主畫面，相機與設定無法同時操作 | 後續若另建管理端，須再確認需求 |

另外修正兩項具體 Bug：新加入 ROI 函式的作用域錯置造成 `beginRoi is not defined`；CSS 縮放後 `79.999988…` 被 floor 為第 79 列。前者將 ROI 邏輯移回頂層，後者在前端將 pointer 座標對齊整數像素邊界。原始失敗與修正後記錄均保存在本機驗證目錄。

像素邊界修正程式碼（`app.js` 的 `canvasPointer`）：

```js
function canvasPointer(event){
  const canvas=$('scene'),box=canvas.getBoundingClientRect();
  return [
    Math.round(Math.max(0,Math.min(canvas.width,(event.clientX-box.left)*canvas.width/box.width))),
    Math.round(Math.max(0,Math.min(canvas.height,(event.clientY-box.top)*canvas.height/box.height)))
  ];
}
```

## 標定與安全資料規則

- 示教保存真實 `[rx,ry,rz]`、放置點配對關節角與 Preferred IK 參考角；不推造 PRE-PLACE 配對角。需要最新 60 秒內讀值，並檢查設定與手臂狀態標記。
- ROI 使用凍結的彩色影像、對齊深度、內參與校準矩陣。`roi_xyxy` 為旋轉後的 display-edge 座標；逆轉換四角後排序為原始半開區域，再計算有效深度中位數與機械臂 XYZ。
- 框選的 0/90/180/270° 與主畫布現有旋轉定義一致。反向拖曳、貼邊區域、超界、非有限值、深度不足均由後端驗證。
- 固定站點儲存需操作者明確確認淨空；示教使用 operator_taught，ROI 使用 rgbd_envelope。`active/ready` 表示軟體資料與指紋可用，不表示實物放置成功。
- 桌面 Z、confirmed 及校準 digest 納入運動安全指紋。交易失敗回復桌面、站點、registry 與事件檔。既有校準變更不會被指紋同步掩蓋。
- Free mode 扭力釋放回覆失敗時保留 V2 連線與租約，供明確退出；連線／建構失敗則釋放未使用的租約。底層關閉失敗時保留隔離租約。
- 本機 `operator_data/` 原檔、既有相機標定與模型權重沒有被測試覆寫。測試只在暫存根目錄建立站點與桌面資料。
