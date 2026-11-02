# 第二階段：混合運動、Start 關節復位、動態預設站點與 RGB-D

日期：2026-10-15。A：`E:\下載\VisionPick_DEV9_PlaceUI_Update\VisionPick_DEV9_Final`；B：`E:\下載\VisionPick_DEV9_Final_大改更好\VisionPick_DEV9_Final`。

本階段已實作四項指定邏輯。驗收以離線控制替身、實際本機資產、API、瀏覽器及回歸測試為證據；未連接實體機械臂、相機或模型供應端。最終逐套計數見 `development_runs/stage2_hybrid_20261015/test_summary.json`，主批 XML 為 `final_verified.xml`，日誌為 `final_verified.log`。

## 讀碼、比對與備份

修改前已保存 A 對應檔案，逐檔完成 A/B 全部差異比對，包括四個下一階段語意檔案。B 沒有的 V2 檔案記為 A-only，未假定存在 B 實作。完整差異：`development_runs/stage2_hybrid_20261015/diffs/index.json` 與同目錄 `.diff`；A 修改前備份為 `before/`。最終 A-before/A-final 差異為 `final_diffs/`。

已重新讀取最終 `hybrid_transfer.py` 全檔、`direct_control_v2.py:1–318`、`place_catalog_v2.py:1–137`、`robot.py:1–108`、`place_profiles.py:930–979`、`runtime_v2.py:75–129`；其他函式行號保存在 `source_function_ranges.json`，可據此重現定位。原始修改前 diff 與最終來源 SHA-256 分別保存在 `before_manifest.json`、`final_hashes.json`。

## 指定行為與證據

### 混合控制

- Start 使用 `start_angles_deg` 的六軸動作；抓取上方與放置上方共用 `hybrid_transfer.move_hybrid_transfer`。
- 有明確 `pre_place_joint_angles_deg` 才直接走配對示教關節角。使用者已確認：咖啡盒子的 `preferred_ik_reference_angles_deg` 僅是手肘姿態參考，不冒充 pre-place 配對角。
- 無配對角時，依序嘗試 Preferred IK、標準 IK；二者皆失敗且端點在工作空間內，改以 TCP 跨區。只捕捉求解例外，已發送動作的到位失敗不再派發另一種動作。
- 關節跨區到位後仍新鮮讀取 TCP 並核對端點，才進入下降。Start 僅核對新鮮關節到位，不需要 TCP。
- 抓取下降／60mm 抬升／放置下降／退離均為 mode=1 直線 TCP，保留設定姿態或站點真實示教姿態。
- `transfer_evidence`／事件記錄求解失敗、實際選用路徑與動作到位。Preferred IK 成功才標示該段 `ik_validated=True`；標準回退、示教直接角、Cartesian 回退均保守記 False；標準成功另有 `selected_ik_solution_available=True`。`path_validated`、`collision_validated_3d`、`outcome_verified` 維持 False。
- 舊 safe-transit／place／retreat 關節資料完整保存；新流程依指定混合控制執行，不執行舊中繼，明列 `legacy_transit_executed=False` 及 retained-asset policy，不猜缺失 TCP。

### Start 與串口

- `direct_control_v2.initialize_for_pick`、`runtime_v2` 預檢／bootstrap 使用有效 Start 六角與速度；缺少或無效診斷 TCP 不阻擋。
- 串口成功條件是 list/tuple、長度至少六、前六個可轉有限數值且不是 bool，回傳前六個；最多三次，間隔 0.08s，失敗 None。
- 兩個 last-verified 欄位僅保留診斷用途。到位輪詢與標準／Preferred IK seed 走新鮮讀取，不以快取補缺。

### 動態預設

- `PlaceProfileManager.set_default/get_default` 使用頂層 `default_profile_id`；只能切到 effective active；停用／刪除預設清空設定，同族新版啟用接續 ID。
- 共享 API：`POST /api/place-profiles/default`，JSON：`{"profile_id":"有效站點ID"}`。原站點管理區與 `/v2/` 均顯示當前預設並可切換。
- V2 fixed 自動補值、舊單步與多步省略目的地均使用當前預設；明示名稱、相對關係、near 優先。舊 A/B/C 明示相容資料保留，省略目的地不依賴槽位。
- 凍結計畫保存 default ID 與 catalog revision。確認前切換預設，舊計畫因 revision 改變被拒，需重新建立；執行中的任務仍用原凍結資料，下一份計畫才用新預設。
- 咖啡盒子為初始預設。現有 01/02/03 未重驗，不會被擅自變成 active。

### RGB-D、near 與結果

- `real_sequence` 不再讀取支撐模型 registry；V2／舊流程 `on_top_of` 使用 A 原有純 RGB-D 頂部高度、占用與侵入檢查。未塞入假重量或容量。
- 支援 near；多步明確 gap 保持傳遞。新增兩個 legacy end-to-end 替身案例，禁止 registry.load，對照 35mm／未指定 gap，驗證一次確認與兩次取像。
- 明示通用「頂部」關係走當幀幾何，具名站點自身的 relation metadata 不攔截此請求；完整站點名稱仍先解析。舊無原句呼叫仍可解析已登錄的 top/table 語意站點，保留相容性。
- 放置後不多取相機影格；UI completed 顯示「動作完成」，後端 `outcome_verified=False`，不宣稱物品落點成功。

## 最終本機 status

`verify_status.py` 直接建立 TableSurface／PlaceProfileManager／PlaceCatalogV2，另以不連接硬體的 backend 驗證 Start。完整原始輸出在 `status.json`。

```json
{
  "TableSurface": {"status":"ready", "z_mm":112.69639708709478, "hardware_motion_validated":false},
  "PlaceProfileManager": {"active_count":1, "default_ready":true, "default_profile_id":"2026-9-30-7-30-51_v1_336e9037"},
  "PlaceCatalogV2": {"ready_count":1, "default_station":"咖啡盒子", "default_ready":true},
  "V2": {"start_tcp_present":false, "start_tcp_ready":false, "start_angles_ready":true,
         "start_ready":true, "start_verification":"fresh_joint_feedback"},
  "preserved_taught_profiles":4, "preserved_taught_fields":28,
  "physical_device_connections":0, "physical_motion_commands":0, "external_api_calls":0
}
```

所有 4 組×7 重要示教欄位共 28 欄逐欄不變；另核對每筆原有欄位值全數不變。`config/v0.yaml`、桌面與 9 個擷取檔本階段不改；資產登錄按最終來源重新計算。有效咖啡盒子的安全指紋未失效，其餘三站仍 `needs_revalidation`（校正／motion-safety 已變）。

## 測試、已知失敗與限制

最終33套離線回歸712項：711通過／1既有失敗／0略過，152.31s。前端8支Node腳本全部通過，不將重疊重跑累加為新案例。瀏覽器10項在 `test_v2_console_integration.py` 的43項中。逐套結果以 XML／JSON 為準。

| 指定／核心測試 | 通過 | 失敗 |
| --- | ---: | ---: |
| `test_robot.py` | 2 | 0 |
| `test_direct_control_v2.py` | 59 | 0 |
| `test_placement_surface.py` | 11 | 0 |
| `test_place_catalog_v2.py` | 20 | 0 |
| `test_real_sequence_place.py` | 19 | 0 |
| `test_stage2_hybrid_control.py` | 25 | 0 |
| `test_stage2_defaults.py` | 8 | 0 |
| `test_stage2_real_rgbd.py` | 2 | 0 |

初次最終回歸發現2個新增無原句相容失敗；已修正單一DestinationResolver分支，相關76項重測通過，再執行上列同一批33套得到711／1。這兩項不是以修改測試或略過測試處理。

保留一項既有失敗：`tests/test_operator_flow_dev8.py::test_explicit_place_profile_overrides_text_destination_semantics`。該案例將 `intent=use, reference=box, relation=inside` 與手選 profile 混用；既有 DestinationResolver 先檢查已知語意目的地而拒絕。從 before 備份載入原 manager／resolver 的隔離探測與最終碼均得到 `ValueError:place_profile_not_found_for_semantic_destination`，見 `prior_failure.json`。本階段不暗中改成「手選覆蓋明示語意」。

| 後續處理選項 | 優點 | 缺點 | 適用場景 |
| --- | --- | --- | --- |
| 維持明示目的地優先，另修訂舊契約／測試 | 保留現有語意優先規則 | 手選不能覆蓋已知衝突語意 | LLM 主導命名／相對目的地 |
| 明確新增人工強制覆蓋模式 | 可保留舊示範操作 | 需新 UI 提示與獨立證據，避免無聲覆寫 | 操作者明確決定目的地 |

尚未驗證：實機串口時序、Preferred IK 手肘姿態、六軸跨區路徑、垂直抓放與 RGB-D 遮擋／侵入誤差；全專案所有 suite 未重跑。工作空間端點與 IK 解不等於碰撞路徑驗證。回退後仍要求實際到位；新鮮 TCP／關節缺失、逾時、持物不確定或取消仍停止，不屬於「求解失敗不中斷」的範圍。

## B 的下一階段語意與路由發現（本階段未合入）

| B 檔案與行數 | 改動／數量 | 專題展示用途 | 取捨與保留建議 |
| --- | --- | --- | --- |
| `AI_MVP.py:298–319` | 放置口語 cues 10→20，新增移到、到、移動到、拿到、放置在、疊在、疊到、送到、移至、搬到 | 容納口語搬運命令 | 納入 V2 Prompt 範例與測題；「到」「拿到」須區分拿取／搬運，避免單詞直接授權 |
| `AI_MVP.py:289–296` | 「上面」映射 on_top_of | 日常疊放說法 | 與圖像 above／支撐頂面歧義一起設計澄清例；「上方」不自動變頂面 |
| `AI_MVP.py:818–829` | move 有 reference／relation／肯定放置 cue 即改 place | 降低模型 move/place 波動的路由中止 | V2 已以五槽位動作為主，建議將意義寫入 Prompt，保留原始模型輸出證據 |
| `runtime.py:2394–2401,2442–2449,2658–2677` | full_pick/full_place/sequence_place 的 full 模式自動導向 real sequence，語音轉寫後也分流 | 少選模式即可執行多動作 | B 分流仍只認2／3段；V2宜用模型 tasks[]，同步擴充計畫／確認／執行契約 |
| `runtime.py:2686–2706` | 漏目的地選手選 profile，否則槽位 A | 單純抓放少問一題 | 本階段已用持久 default_profile_id 取代，保留便利性不回植固定 A |
| `runtime.py` A-before:2649–2687 的命名 resolver 段在 B 被移除；A-before:2750 附近的 full_place_requires_place_intent gate 在 B 被移除 | 放寬具名／語意與完整放置意圖阻擋 | 演示流程不因有限契約中止 | 屬設計放寬，非一概 Bug；V2 不移除完整名稱優先與必要槽位檢查，可用 Prompt 補意圖 |
| `task_plan_model.py:128–135` | 多步 allowed relations 加 on_top_of、取消每步必須含精確關係 cue | 接受口語頂部疊放 | 採完整五槽位模型契約；B 此處仍無 near，不能視為全相對路由打通 |
| `src/operator_console/tools.py:45–58` | move 與 place 都選 fixed_place_target／pick_and_place | 避免可搬物被判 move 而失去抓放路由 | 建議作 Prompt 動作映射範例；根目錄 tools.py 在 A/B 都不存在 |
| `task_plan_model.py:20,44–49,63–71` | 仍限2／3段、maxItems=3；Prompt 描述要求3D top澄清，但編譯器已許可top | 舊有限步數可簡化演示 | B 未解除三步上限，亦未提供 V2 五槽位升級；第三階段應成套修訂 |
| A-only schema helper 在 B 缺少：A `AI_MVP.py:156–169` | B 無 TASK_SCHEMA_V2_FILE／export_task_schema_v2 | B 使用舊語意主線 | 保留 A V2 schema／parser；不因 B 檔較短而刪除新契約 |

| 第三階段方案 | 優點 | 缺點 | 適用場景 |
| --- | --- | --- | --- |
| 語彙／例句移入 V2 Prompt，同步擴展 tasks[]、確認及序列上限 | 保留 LLM 主判斷，長指令有一致契約 | 需凍結新增離線測題與定義可執行步數 | V2 作主要展示入口 |
| 保留 B 的 deterministic intent／路由相容層 | 舊流程少改動即可接受展示口語 | 兩套路由可能歧義，仍需修三步與near缺口 | 舊入口仍為主的過渡期 |

本階段不修改 `AI_MVP.py`、`task_plan_model.py`、`tools.py`、V2 Prompt／Schema、三步上限或供應端預算。下一階段由使用者決定兩方案與長計畫步數；不自動發模型請求。

## 可重現入口

- `python -X utf8 development_runs/stage2_hybrid_20261015/verify_status.py`：本機資產＋受控 Start；會更新 asset_registry。
- 最終 pytest／Node 命令清單：同目錄 `commands.json`；測試總表：`test_summary.json`；修改清單：`changed_files.json`。
- 備份原 UI 為 `before/src/operator_console/static/`，未改視覺方向／入口；只增加指定預設點操作與簡潔完成文字。

## 修改檔案清單

共29個正式檔案：15個程式／UI、10個測試、2個資料、2個文件。開發證據資料夾的備份／日誌／驗證腳本另存，不計為產品檔案。

| 分組 | 完整相對路徑 |
| --- | --- |
| 控制 | `src/v0_mvp/robot.py`；`src/operator_console/hybrid_transfer.py`（新增）；`src/operator_console/robot_adapter.py`；`src/operator_console/direct_control_v2.py` |
| 站點／語意目的地 | `src/operator_console/place_profiles.py`；`src/operator_console/place_catalog_v2.py`；`src/operator_console/destination_resolver.py` |
| 執行／API | `src/operator_console/runtime.py`；`src/operator_console/runtime_v2.py`；`src/operator_console/real_sequence.py`；`src/operator_console/task_sequence_v2.py`；`src/operator_console/server.py` |
| UI | `src/operator_console/static/app.js`；`src/operator_console/static/v2/app.js`；`src/operator_console/static/v2/index.html` |
| 更新測試 | `tests/test_direct_control_v2.py`；`tests/test_place_catalog_v2.py`；`tests/test_place_motion_guard_v08.py`；`tests/test_real_sequence_place.py`；`tests/test_v2_console_integration.py`；`tests/test_ux_ui_functions.js` |
| 新增測試 | `tests/test_stage2_hybrid_control.py`；`tests/test_stage2_defaults.py`；`tests/test_stage2_real_rgbd.py`；`tests/test_stage2_default_ui.js` |
| 資料 | `operator_data/place_profiles.json`；`operator_data/asset_registry.json` |
| 文件 | `HANDOFF.md`；`docs/STAGE2_HYBRID_INTEGRATION_20261015.md` |
