# AI Handoff

更新：2026-11-07（Asia/Taipei）。本檔所在目錄就是程式倉庫根目錄；先讀 AGENTS.md 與 [雙電腦流程](docs/TWO_PC_WORKFLOW.md)。

## Current Focus

- 報告最新標準為 VisionPick_成果報告_1105_第三版.docx，另存原電腦，不是本倉庫必要執行檔。
- 先讀 [共用修改與寫作規則](docs/SHARED_WORKING_RULES.md)：程式必要最小改動；Word 自然流暢、便於口述，不主動添加防禦性說明。驗收期間依實況修訂 Word，使用者確認 Word 定稿後才製作 PPT。
- 實機驗收批次 	est_records/HW_20261107_001/summary.md：受測 Commit 16e2f1eea0dc9b465d0472e843337538f2a4376c。完成「筆跟麥克筆交換」實機測試 4 案例（含 2 成功完成案例 db94bf47...、bf3439f...，與 2 關節超限案例 8c4ee550...、5320cfa7...）。
- 出發標籤 pre-hardware-20261106；實機分支 hardware/acceptance-01。原電腦保留 main，未驗收修改與結果先上測試分支。
- 不新增 Agent、重試管理類別／例外框架或 LLM 呼叫，不改 Prompt／任務格式。
- 物品結果門檻不變：XY／疊放高度各 20 mm、Named 位移 25 mm 或不可見通過，短位移未動仍可通過。不是物理精度／入盒證據。
- 包絡框重疊解耦依使用者指示暫緩；實機初驗限制每類一件。

## Last Meaningful Changes

- 第三輪（11/07）：實機現場反饋與問題修正（受測 Commit 16e2f1eea0dc9b465d0472e843337538f2a4376c）：
  1. **Bug A（控制器代碼 2 精確提示）**：在 direct_control_v2.py、task_sequence_v2.py 與 app.js 中，當機械臂發送 -1 且韌體回傳代碼 2 時，明確標記並於 UI 提示「機械臂姿態到達運動學極限或逆解無解（代碼 2）」，排除原連線中斷誤報。
  2. **Bug B（深度雜訊容差）**：在 relative_placement.py 的 _surface_clear 中允許最多 2% 深度像素無效容差，濾除邊緣雜訊（解決 Step 3 原因 31 點缺失被判死問題）。
  3. **設定微調**：config/v0.yaml 微調 tcp_xy_offset_mm 為 [-3, -3]，open_wait_s 調整為 1.8 秒。
  4. **實機驗證**：修復後執行 3 步交換任務，驗證案例 4 批共 12 步嘗試中 10 步成功完成（db94bf47... 與 abf3439f... 實機全部 3 步連續完成物理抓放）。
  5. 測試套件新增 2% 容差邊界與控制器錯誤碼標記單元測試，全套 399 項單元測試通過。
- 第二輪（11/06）：控制器單次回讀、SDK transport lock、端點 1.5 秒 / 容差 2 次、逾時判定、Start/垂直終點證據接線、bounded recovery 重拍。
- 第一輪（11/05）：本地結果檢查＋當步最多一次重做，保留首次終點／疊放期望，不重呼叫 LLM。

## Open Decisions / Risks

- 1.5 秒是應用層端點／診斷查詢間隔，不是已證明的 ESP32 穩定頻率；SDK 內部最多三次重送與 IK 求解通訊仍存在。
- 清理與新查詢不能證明裝置端樣本無延遲；到位表示回讀在容差內，不證明抓取、支撐、入盒或路徑安全。
- outcome_verified／path_validated／collision_validated_3d 仍為 False；arrival_verified 依關節／座標回讀判定，關節確認不等同 TCP 量測。
- 同名配對、Named 不可見通過、短位移容差及遮罩下方桌面假設維持既有政策。
- config/v0.yaml、calibration/、models/ 受 Git 追蹤；operator_data/、執行資料、憑證與虛擬環境不隨 Git。現場設定可完整複製到 operator_data/local_config.yaml 並用 --config 選用，不是疊加設定。
- 第二台需核對 COM埠、依賴／驅動、模型／韌體、相機／手臂幾何與站點。幾何改變須重新標定，不能直接沿用原站點或持物旗標。
- 倉庫目前公開，該倉庫的 Releases 附件也公開；上傳前核對金鑰及現場影像內容。

## Verification Status

- 函式修改逐一 compile() 通過；
- 執行受影響測試集：pytest tests/test_placement_preview_v2.py tests/test_direct_control_v2.py tests/test_bounded_recovery_v2.py tests/test_task_sequence_v2.py：399 passed／0 failed；
- 實機回算驗證：runs/v2/8c4ee550... 之 Step 3 經 2% 容差重算通過（7555/7586 有效點，0 違規，RZ: -52.4 deg）；
- 實機物理測試：HW_20261107_001 完成 4 案例（共 12 步動作，10 步成功，2 步超限中斷），其中 db94bf47... 與 abf3439f... 連續兩次 3 步交換實機全數完成。

## Next Step

- 將 test_records/HW_20261107_001/summary.md 與 HANDOFF.md 提交並 Push 至 hardware/acceptance-01；
- 於 GitHub Releases 建立 pre-release 標籤 evidence-HW_20261107_001 並上傳 evidence_HW_20261107_001.zip 附件（9.14 MB）；
- 回填 Release 網址至 summary.md；
- 原電腦 Pull 後進行審查與 PR 合併，接續修訂成果報告 Word 檔。
