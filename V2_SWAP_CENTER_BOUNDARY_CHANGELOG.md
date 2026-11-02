# VisionPick V2 — 交換規劃與中心工作範圍修正

版本：2026-10-21（修改版，尚未實機驗證）

## 修改內容

1. 工作台規劃方向仍採原來的 90° 設定；UI 視角切換沒有更動。
2. 交換 Step 1 仍使用既有 `align_reference` 政策把 A 對齊 B，並保留初始 `temp_a_deg` / `temp_b_deg` 角度快照；後續 Step 2/3 使用既有 `match_initial` 政策，完成 A/B 位置與角度交換。
3. 暫存 A 的夾爪包絡不得與原始 A、B 的完整夾爪安全包絡接觸，而不只是與原始物件輪廓不接觸。
4. 交換專用 `near` 候選搜尋：先沿用四個基本方向與原本距離；仍無合法結果時，再搜尋斜向以及可選的較遠距離（預設上限 120 mm）。有明確設定的候選方向、最大距離時尊重設定。全部候選仍通過相同 `plan_relative_place()` 檢查。
5. 相對放置的 `surface.bounds_mm`、`workspace_mm` XY 邊界改用放置物件中心座標判定；具名放置也同步改用中心座標。維持完整包絡障礙物與 RGB-D `_surface_clear()` 高度檢查，以及 TCP 姿態邊界檢查。
6. 不可行候選點僅保留錯誤原因，不再輸出成可畫在畫面上的正式預計落點。
7. 如果交換前一步預覽不可行，後續步驟標記 `previous_step_geometry_failed`，不再從被拒絕的位置推進虛擬場景。
8. 放置 RZ 欄位的錯誤提示改為「放置檢查未通過」，不再誤導為 RZ 本身錯誤。

## 修改檔案

- `src/integration/relative_placement.py`
- `src/operator_console/runtime_v2.py`
- `src/operator_console/static/v2/app.js`
- `tests/test_placement_preview_v2.py`
- `tests/test_placement_preview_v2.js`
- `tests/placement_preview_v2_browser.cjs`

## 已驗證

- Python 新增的 4 個幾何回歸測試：4 passed。
- Python 交換、預覽與幾何相關篩選測試：8 passed。
- Python 原既有對照測試篩選：5 passed（另一次篩選為 5 passed）。
- Node.js 預覽繪圖測試：11 passed。
- 已通過 Python 語法編譯和 JavaScript 語法檢查。

## 尚待驗證

- 大型整合測試執行超過允許的時間；不宣稱整套測試通過。
- 沒有連接真實機械臂、RGB-D 相機或 Gemini 服務。
- 真實放置前仍必須以低速、空手及有人工急停監護的方式實機驗證。

## 特別說明

- `_surface_clear()` 未修改；包絡即使跨出 XY 工作範圍，若其投影跨出 RGB-D 有效視野或深度無效，仍可能被既有高度／可見性規則拒絕。
- `tcp_pose_bounds` 仍檢查每個 TCP 姿態的 X/Y/Z，不因放置中心在範圍內就放行不可達 TCP。
- 使用者若手動改寫放置角度策略，程式仍尊重人工覆寫值。
