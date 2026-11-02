# 第一階段硬體參數與現場資產整合驗收

2026-10-15（Asia/Taipei）。A：`E:\下載\VisionPick_DEV9_PlaceUI_Update\VisionPick_DEV9_Final`；B：`E:\下載\VisionPick_DEV9_Final_大改更好\VisionPick_DEV9_Final`。

本階段四項指定修改與離線驗證完成。桌面 ready，咖啡盒子在舊管理器 active、V2 ready；不代表實機運動／放置結果已驗收。沒有模型 API、裝置連線或實體運動。

## 1. 修改檔案與測試結果

修改前已保存 A/B 對應檔案的完整 unified diff（包括 A-only 檔案）於 `development_runs/stage1_hardware_20261015/diffs/`；原始碼、設定、UI、站點與 registry 備份於同目錄的 `before/`。原始 hash inventory：`before_manifest.json`。本報告沒有覆寫 B。

實際讀碼範圍：A `robot.py:15–83,174–253,269–376,428–470,482–517`；`direct_control_v2.py:1–308`；`place_profiles.py:248–475,588–628,793–839,917–947`；`assets.py:1–311`；`table_surface.py:1–57`；`place_catalog_v2.py:1–129`；`relative_placement.py:39–467`；`real_sequence.py:57–202,487–794,1465–1528,1698–1721`；`static/app.js:1723–1847,1929–1985,2064–2119`；`place_capture.py:1–175`。B 主要實際行數見下一節；保存的 diff 全部以修改前的 A 為基準。

### 變更清單（28 個交付檔，另有驗證證據／備份）

| 檔案／範圍 | 修改內容 |
|---|---|
| `config/v0.yaml` | TCP XY=-6/-6、grasp Z=1、timeout=25、fallback=3、gripper.wait_s=3；RX/RY 原先即為 -175/-5，保留 |
| `src/v0_mvp/robot.py` | `_wait()` 讀 gripper.wait_s；新鮮讀取最多 3 次、兩次 0.08 秒間隔；僅接受 6 個有限數值；失敗 None；零距離分段也不回傳快取 |
| `src/operator_console/direct_control_v2.py` | `_feedback()` 共用新鮮重試；初始化／全任務檢查實際使用的 gripper.wait_s |
| `src/operator_console/place_profiles.py` | status 提供 config 驅動的 default_orientation_deg；不改示教數據 |
| `src/operator_console/assets.py` | 防奇異點角度、timeout、fallback、gripper 等待納入 motion fingerprint；桌面與擷取檔納入 registry |
| `src/operator_console/static/app.js` | 新建空欄位與相機選點套用後端提供的角度；重新讀取不覆寫既有示教／手填欄位 |
| `src/operator_console/static/index.html`、`static/precision/index.html` | 修正預設姿態說明；原檔先備份，未改 UI 路由或版型 |
| `operator_data/place_profiles.json` | 同一 profile ID 合入咖啡盒子名稱／別名與 B 資料，更新其 calibration／motion fingerprint；不重驗其他 3 個站點、不改槽位 |
| `operator_data/asset_registry.json` | 使用 AssetRegistry.register_current() 重新登錄，保留原歷史快照 |
| `operator_data/table_surface.json` | B 原檔逐位元複製 |
| `operator_data/place_captures/pcap_22166cbdee0374f0/` | depth_m.npy、frame.png、metadata.json，3 檔逐位元複製 |
| `operator_data/place_captures/pcap_6804a83d32d0e2e4/` | 同上，3 檔 |
| `operator_data/place_captures/pcap_cebae162b1e56f52/` | 同上，3 檔 |
| `tests/test_direct_control_v2.py` | 空回饋測試改為連續 3 次讀取失敗，仍檢查穩定次數重置；10 次串口讀取對應 8 次輪詢 |
| `tests/test_place_ui_functions.js` | 以設定資料驗證新預設與變更後的角度 |
| `tests/test_real_sequence_place.py` | 合成校正的測試夾具使用獨立暫存桌面；保留正式校正指紋阻擋 |
| `tests/test_stage1_hardware_feedback.py` | 36 項新鮮重試／快取拒絕／逾時／不重送命令／等待測試 |
| `tests/test_stage1_assets.py` | 14 項預設姿態／示教保留／相對姿態／指紋／擷取資產測試 |
| `tests/test_stage1_place_defaults.js` | 空欄位、示教保留、相機新建預設與設定缺失拒絕 |
| `HANDOFF.md`、本報告 | 本階段結果、既有風險與後續決定 |

`relative_placement.py` 原本即讀 config 的 RX/RY（346–347），`real_sequence.py` 從該 planner 的三個 candidate poses 建立 PlacePlan（1500–1521），`PlaceProfileManager.build_plan()` 原樣使用具名站點三個姿態。這三條路徑沒有為了預設姿態去改寫示教資料。

### 測試證據

- 主回歸：21 套、379 項；初次 376 passed / 3 failed / 0 skipped，232.89s。三項失敗為合成校正夾具讀到現場桌面，正確觸發 table_surface_calibration_changed。
- 修正夾具後：`test_real_sequence_place.py`、`test_placement_surface.py`、`test_ux_table_surface.py` 共 43 passed / 0 failed，83.94s。
- 兩批聯集 379 項均有通過證據；不是同一批 379 全綠。正式程式在兩批之間沒有再修改，只有測試夾具獨立化。
- 必跑四套：`test_robot.py` 2/2、`test_direct_control_v2.py` 59/59、`test_placement_surface.py` 11/11、`test_place_catalog_v2.py` 20/20。
- 其餘涵蓋 motion verification、segmented approach、pregrasp、50 項新增 Python 測試、相對幾何／instance、平台／站點、single-step、實機序列受控後端、桌面、V2 HTTP 整合 43/43。
- 前端 7 支腳本均通過。`test_relative_place_preview.js` 必須提供工具產生的 plans.json；初次未帶參數失敗，依正確命令重跑通過。不是正式程式失敗。
- SDK 產生 1 類既有 DeprecationWarning。完整全專案 regression 本輪未執行。

命令與結果：`development_runs/stage1_hardware_20261015/regression.log`、`regression.xml`、`sequence_fixed.log`、`sequence_fixed.xml`、`test_summary.json`、`js_results.json`。

重現資產驗收：

```powershell
python -X utf8 development_runs/stage1_hardware_20261015/verify_assets.py
```

主回歸命令完整保存於 `commands.txt`。前端 preview 應先執行 `python -X utf8 tools/preview_s3_relative_place.py --out <新目錄>`，再 `node tests/test_relative_place_preview.js <新目錄>/plans.json`。

### 真實本地 status（摘要；完整 JSON 在 status.json）

```json
{
  "TableSurface": {"status":"ready", "ready":true, "z_mm":112.69639708709478, "hardware_motion_validated":false},
  "PlaceProfileManager": {"active_count":1, "coffee_effective_status":"active", "coffee_revalidation_reasons":[], "default_orientation_deg":[-175.0,-5.0,-45.0]},
  "PlaceCatalogV2": {"ready_count":1, "coffee_status":"active", "coffee_ready":true, "coffee_reason":null, "conflicts":{}},
  "motion_safety_asset_id":"motion_safety_profile:2c8ae4ce0433",
  "asset_snapshot_id":"assets:6871016d8abea52d",
  "legacy_named_resolution":"active",
  "preserved_taught_profiles":4,
  "preserved_taught_field_count":28,
  "capture_count":9,
  "start_tcp_present":false
}
```

咖啡盒子 place `[7.4,170.7,217.8,177.3,-4.14,20.23]`、pre-place Z=277.8、retreat Z=297.8，全部角度原樣保存。其 preferred IK 關節 `[8.17,-7.38,-73.12,-7.55,9.05,0.43]` 原樣保存；place／retreat／safe-transit joint 原始 null 仍為 null，未偽造為配對關節。A 其餘 3 個 profile 的整筆資料未變，仍因 calibration／motion assets 不同而 needs_revalidation。

## 2. 專案 B 額外設計發現與保留建議清單

下表行數均為 B 現在的原始檔案；「展示收益」是由程式行為推論的潛在用途，不是對原作者意圖或實機成功率的證明。除四項指派工作外，下列策略均未合入 A。

| ID／B 檔案行數 | 改動及分類 | 專題實機的潛在好處 | 代價／保留建議 |
|---|---|---|---|
| D1 `robot.py:58–92,263–266` | 接受 ≥6 個值並裁切、新讀取即寫 last_verified；全失敗回快取，Start 無回報時寫入目標角 | 容忍串口尾部資料／短暫斷訊、畫面不中斷 | 寬鬆形狀屬策略；無回報當到位與本輪明確紅線衝突，不合入。最後已知值若保留只能另設診斷狀態，標記時間／非新鮮，不能餵到位或 IK |
| D2 `robot_adapter.py:585–624` | 偏好 IK→標準 IK→直接 Cartesian；Cartesian 預檢失敗也繼續，設計選擇 | SDK IK 不穩定時降低展示中斷次數 | 少了預先可達性門檻；保留新鮮到位、速度／範圍／持物控制，以顯式展示選項評估；V2 已走直接 Cartesian，不必整包套舊 hybrid |
| D3 `real_sequence.py:709–738` | 捕捉任意預檢例外，只查 6 個 endpoint XYZ 邊界便繼續 | 避開 SDK／預檢故障中斷 | fallback 屬策略；仍寫 ik_validated/path_validated=True 與實際未通過預檢不一致，是證據欄位錯誤。若採用 fallback，須分開記錄預檢失敗與實際回饋，不照抄成功旗標；workspace 缺失時 in_bounds 初值 True 也須處理 |
| D4 `real_sequence.py:442–479,1422–1440`、`relative_placement.py:192–243` | 缺模型時補 RGB-D support/object 預設；flat/stable 強制 True；容量500g、質量20g、高度20mm或RGB-D推算、COM=0；放寬 source | 少做離線模型輸入，容易開始頂部疊放展示 | 深度量不到承重／穩定性／質量。可保留「展示假設」模式並明列假設，不能叫已量測；A 最新幾何頂部目前已獨立演進，不直接把 B 舊政策覆蓋回去 |
| D5 `relative_placement.py:297–305,340–347` | 人工模型仍檢查支撐包絡與 COM；RGB-D 模式略過部分模型檢查，頂部深度容差擴為 max(tol, abs(top-table)+tol) | 容許較高支撐周圍仍看見較低桌面，減少頂部放置被深度門檻擋住 | 擴大整體高度容差也會放過更高障礙。比較 A 的「頂部允許較低周圍、較高侵入仍拒絕」策略後再決定，不單看門檻寬鬆程度 |
| D6 `real_sequence.py:65–76,118–184,226,270–273` | ordered_segments、較寬的固定位置文字匹配、模型漏填 fixed_zone 時由文字補值；移除部分關係 cue／頂部明示檢查及具名 catalog 路由 | 容忍日常說法和模型漏槽，減少解析阻擋 | 可能改變「上方」與「頂部」語意及具名站點優先次序。可只採「模型漏槽才補值」的狹窄單元並加案例；不整包退回舊路由 |
| D7 `place_profiles.py:911–974` | 直接 regex 名稱／別名、沒目的地自動 A、以 reference+relation 找站點 | 單步指令省略目的地時也能展示，容忍較短語句 | A 已有完整 DestinationResolver；slot A 未綁定／多站點相同 reference+relation 要處理。先比較同一批省略／具名／相對案例，再決定要不要保留狹窄 default 行為 |
| D8 `real_sequence.py:1402–1421` | 桌面不是 ready 就以全部物件 Z 中位數回退，無候選用0 | 缺桌面資料時仍可走相對放置 | stale 校正與 invalid 也被回退；物件頂部中位數不等於桌面。若需要現場容錯，建議只在 missing 且明示允許時回退，保留 stale/invalid 阻擋 |
| D9 `real_sequence.py:1649–1665`、`static/app.js:1017–1047` | 取消事後取像驗收並精簡畫面；卻把 skipped_by_policy 寫 passed/outcome_verified=True | 更短的流程、更少等待與視覺雜訊 | 省略驗收是設計選擇，A 本來已允許；「沒驗收寫已驗證」是證據語意錯誤。可用簡潔「動作完成」UI，持續保留 outcome_verified=False |
| D10 `static/app.js:568–575,976–982`、`real_sequence.py:610–630,1450–1457` | 全放置模式隱藏間距設定、序列描述去除 gap；B 不再將 sequence 設定 gap 重新寫入 relative destination | 降低操作者設定負擔 | 單步與多步間距來源不再清楚；缺 gap 時會落到各步 fallback。可做較簡潔 UI，但保留實際 gap／來源與必要設定，不照抄參數遺漏 |
| D11 diff 中 A `real_sequence.py:1057–1059,1398–1403,1482–1483,1494–1497,1548–1551` 對應 B 已移除；B `real_sequence.py:1234–1239` | 減少 effective config、分步 RGB-D／偵測／相對計畫／拒絕原因輸出；實機判斷改為僅 backend_factory 是否存在 | 減少寫碟、降低保存負擔；避免真正裝置的等待被模擬 delay cap 縮短 | 紀錄減量的原作者動機未證實，應量測各項 I/O 再決定；不能犧牲失敗重現資料。backend 模式也應保留明確硬體／模擬身分 |
| D12 `operator_data/place_profiles.json:93–177,179–194` | 額外「右盒子」為 disabled、有 BOXBOX 別名、max_coordinate_speed=100；B 預設 layout 綁 A=咖啡／B=右盒子 | 同一套展示可快速選多站點與預定布局 | 未獲本輪匯入此站點／布局的指派，未合入；它是視覺選點且姿態 -180/0，配對 place/retreat joints 是 null，不應視為示教完整。可另審站點、速度與綁定；A full_place 的總速度上限仍有自身門檻 |

上述 D2–D12 待使用者決定下一階段範圍。D1 的運動快取回退不在可合入範圍；可另外討論純 UI 診斷。

| 下一階段方案 | 優點 | 缺點 | 適用場景 |
|---|---|---|---|
| 小單元審核 D2/D6 的明示 fallback／漏槽補值，保留真實證據欄位 | 降低現場中斷且可追溯；每個策略可獨立關閉 | 需要各自凍結測試與受控實機量測 | 想改善展示容錯率但維持可驗收紀錄 |
| 保持現有策略，先補 Start TCP 與現場驗收資料 | 先確定現有可重現流程與參數 | 仍會遇到現場 SDK／解析阻擋 | 第一階段資產已 ready，優先取得實機證據 |

## 3. 驗證範圍與待辦

此次修改的離線驗證完成；尚有以下未驗證項目：實機串口三次重試時序、機械臂在 -175/-5 姿態的垂直運動與 IK 行為、現場碰撞／放置結果、真實相機場景變化。

- `robot.start_pose_mm_deg` 仍缺；這是 Start 配對 TCP，與咖啡盒子不存在舊 safe_transit 角度而可通過 catalog 的條件不同。未猜測／回算／刪中繼來繞過。
- 咖啡盒子資產 active/ready 能通過舊具名解析與V2 direct_place；完整 V2 實機啟動仍須量測 Start TCP。
- 舊 01/02/03 保留原始座標／關節與fingerprint，仍 needs_revalidation；未宣告有效。
- 桌面 hardware_motion_validated=False、planning.enabled=False、collision_validated_3d=False 與未驗收的放置語意均保留。
- B 額外策略的選擇、全專案歷史 provenance／封裝依賴等原有議題留待下一階段，本輪不混入修正。

現場可依序驗收：先人工取得與既有 Start 角度配對的 TCP；再空載確認串口失敗時停止／正常回饋才到位；之後測 -175/-5 空載垂直路徑與咖啡盒子示教姿態；最後才記錄實物抓放結果。這些步驟本輪未執行。
