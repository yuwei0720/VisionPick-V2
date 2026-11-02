# AI Handoff

更新：2026-11-02（Asia/Taipei）。實際工作根目錄：此檔所在的內層 VisionPick_V2_Final_Version。外層報告與簡報未修改。

## Current Focus

- 使用者要求純 V2 架構、原生站點／桌面標定、四分頁工程抽屜、100vh 主畫面與 GitHub 整備；使用者後續明確指示「先整理不上傳」。
- 正式入口只載入 service_v2／runtime_v2；V1 無正式路由或 import。47 份退役程式、頁面與舊操作文件保存在本機 _deprecated_v1/，排除 Git 與打包。

## Last Meaningful Changes

- 新 service_v2 管理設定與共用資產，不繼承 V1。server 僅提供 V2 UI 與 /api/v2/；/legacy/ 及 V1 API 回傳 404。
- runtime_v2 原生讀取 TCP／六軸角、Free mode 進出、Start 復位、RGB-D 擷取、四旋轉 ROI 逆轉換及桌面標定。
- 示教保存真實姿態與 PLACE 配對關節角，儲存後啟用並同步 registry；不推造 PRE-PLACE 配對角。ROI 固定使用 rgbd_envelope 檢查。
- 桌面數值納入運動指紋；僅同步原有效站點。站點／桌面／registry／事件寫入失敗會回復。扭力釋放失敗保留租約，未使用租約則釋放。
- 保留相機、18 點剛體標定、OBB 幾何、底層控制、relative_placement、hybrid_transfer 與 voice_input；移出舊任務模型與 A/B/C 業務。
- 工程抽屜分為站點與標定、抓取與幾何、運動與夾爪、診斷與紀錄；保留主畫面網格，增加 pointer 拖曳與數字 ROI 替代。
- README.md、.gitignore、依賴盤點工具與回歸測試已更新。完整拆分證據見 docs/V2_MIGRATION_AUDIT.md；測試與限制見 docs/V2_VERIFICATION.md。

## Open Decisions / Risks

- 未做實際相機、串口、Free mode 或實物抓放驗收；fixed wait、active/ready 及軟體測試不證明實際到位或物件結果。
- 原工作站 operator_data/、config/v0.yaml、標定與權重保留；舊站點因安全指紋政策更新須在現場確認後重驗。Git 排除 operator_data/，乾淨副本從空站點開始。
- Impeccable 偵測器仍有既有主畫面小字／風格提示，沒有宣稱完成全面無障礙驗收；見驗證文件。
- 本機 Git 已初始化，未設定遠端、未上傳；使用者尚未要求發布。

## Verification Status

- 修改前基線：pytest 771 passed；Node 24 passed。
- 最終完整 pytest：808 passed、0 failed、0 errors、0 skipped，294.59 秒；2 則第三方棄用警告。development_runs/v2_native_20261102/full_verified.log/xml。
- Node：28 passed、0 failed、0 skipped；node_final.log。
- AST：58 個 src 模組、442 個 import；42 個正式入口可達模組；V1 禁用業務 import 0；docs/V2_DEPENDENCY_AUDIT.json。
- Edge headless：4 分頁、4 旋轉拖曳、示教／ROI 儲存、桌面更新、數字 ROI／Esc 通過；8 張截圖，頁面錯誤 0。1440×900 及 1050／1920 寬均 scrollHeight=clientHeight=900。資料來自受控替身，沒有實體運動。
- Python compileall 通過。憑證格式掃描匹配 0；本機資料 receipt 14 份的修改時間早於本次工作，原 config/v0.yaml 與修改前 SHA-256 一致。
- 交付白名單 154 份來源，SHA-256／ZIP CRC 通過；乾淨解包不含 operator_data 或退役檔，CLI help、5 項 Python 定向與 28 項 Node 通過。Git 僅保留本機整理提交，無遠端、未上傳。

## Next Step

- 本輪軟體整理與驗證完成；使用 README.md 啟動純 V2 工作台。使用者未要求上傳，不得自行設定遠端或發布。
- 日後現場啟用時先重驗站點、重測桌面與確認空手；再依現場驗收結果決定是否交付硬體使用。
