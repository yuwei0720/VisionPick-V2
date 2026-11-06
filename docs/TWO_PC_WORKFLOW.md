# 雙電腦：實機測試與結果回傳

倉庫：<https://github.com/yuwei0720/VisionPick-V2>。出發標籤 `pre-hardware-20261106` 是「實機驗收前基準」，不是實機通過標記。實機使用 `hardware/acceptance-01` 分支；這一輪由實機電腦修改，原電腦閱讀／分析。

## 1. 另一台電腦第一次設定

1. 安裝 GitHub Desktop 並登入有此倉庫寫入權限的帳號；另安裝 Python、專案套件與設備驅動。
2. `File → Clone repository → URL`，貼上倉庫網址，選空資料夾，再按 `Clone`。不需要另建倉庫或 Fork。
3. 上方 `Current branch` 選 `hardware/acceptance-01`，再按 `Repository → Show in Explorer` 開啟程式目錄。
4. 先讀根目錄 AGENTS.md、HANDOFF.md、README.md；安裝方式見 README。requirements-lock.txt 是舊環境參考，不保證新電腦相容。
5. 本機設定可完整複製為 `operator_data/local_config.yaml`，用 `python run_v0.py ui --config operator_data/local_config.yaml` 選用；這不是疊加設定，這個命令預設不開設備與運動。
6. 核對 COM 埠、模型／韌體、標定、桌面及站點；相機／手臂相對幾何改變時重新標定。初次每類只放一件。金鑰用環境變數。

operator_data/ 已被 Git 忽略，需另行備份。既有啟動器預設仍用 config/v0.yaml，要使用本機設定須明確帶 --config。不要只搬移控制器持物旗標就宣稱空手。

## 2. 每次開始：選分支，再更新

1. 先停止工作台任務，確認 `Current branch` 是測試分支。
2. Changes 有上次的未提交修改時，先檢查並 Commit，不按 Discard changes。
3. 按 `Fetch origin`；有更新再按 `Pull origin`。衝突時保留檔案並請協作者處理，不用 Force push。
4. 修改後先做受影響部分的離線檢查；在 Changes 勾選相關檔案、填 Summary、按 `Commit to hardware/acceptance-01`。
5. 在 History 右鍵該提交 → `Copy SHA`，填入本批受測程式提交編號。測試中再改程式，須另作提交並開新批次。
6. 操作者確認現場後再做實機測試。Git 更新不會自動啟動測試或機械臂。

## 3. 每批結束：失敗、中止也保存

1. 等任務停止、檔案寫入完成，建立 `test_records/HW_YYYYMMDD_001/`，把 `test_records/TEMPLATE.md` 複製為其中的 `summary.md` 並填寫。
2. 依 run ID 將本批 `runs/v2/<run_id>/`、實際設定／標定快照、環境資料與人工照片複製到倉庫外的批次資料夾，保留來源。自訂 runs-dir 時以實際路徑為準。
3. 檢查不含金鑰及不適合公開的影像，再壓成 ZIP。ZIP 不加入 Git 原始碼歷史。
4. PowerShell 執行 `Get-FileHash -Algorithm SHA256 -LiteralPath '完整ZIP路徑'`，把 Hash 填入摘要。可請現場 AI 整理及計算，但不能讓 AI 推測人工觀察結果。
5. 在 Desktop 勾選摘要與 HANDOFF.md 的更新，填說明，按 `Commit to… → Push origin`。Commit 只存本機，Push 才送到 GitHub。
6. GitHub 網頁右側 `Releases → Draft a new release`；建立唯一標籤 `evidence-HW_YYYYMMDD_001`，Target 選 hardware/acceptance-01，確認該分支目前就是剛推送的摘要提交，再附上 ZIP、勾 `This is a pre-release`，標題寫「實測資料 HW_...（非驗收通過宣告）」後發布。
7. 將 Release 網址填回 summary.md，再 Commit／Push。網頁確認看得到附件，這批才算完成回傳；補測另開新編號，不覆寫原批次。

目前倉庫公開，摘要與 Releases 附件皆公開。若附件上傳受帳號政策限制，保留本機 ZIP，摘要明寫「附件尚未上傳」及雜湊，不宣稱已回傳。一般 Git 檔案超過 100 MiB 會被阻擋，附件另受 GitHub 當前額度限制。

## 4. 原電腦查看中途結果

建議另用 Desktop 克隆一份到 `VisionPick_驗收檢視`，只需一次；原開發目錄保留 main。

1. 檢視用副本選 `hardware/acceptance-01`，按 `Fetch origin → Pull origin`。
2. 開 test_records/ 的新批次 summary.md，讀取結果、受測提交與證據連結。此副本也會取得分支的實驗程式，不要因此啟動設備。
3. 點 Release 連結下載 ZIP 到倉庫外資料目錄，以 Get-FileHash 核對摘要雜湊後解壓分析，不覆蓋原始證據。
4. 分析另存 `test_records/<批次>/analysis.md`。要提交時先和實機電腦約定只由一台寫入分支，遵守先 Pull、後 Commit／Push。

**Pull 不會自動下載 Releases 附件。**摘要與程式按 Pull，原始資料按連結下載；本版未新增自動封存／下載程式。

## 5. 本輪確認後，合併並更新原電腦

1. 實機電腦更新 HANDOFF.md，列通過／失敗／未測項與待辦，Commit／Push。
2. GitHub `Pull requests → New pull request`：base 選 main，compare 選 hardware/acceptance-01；確認沒有混入另一台專屬的 COM 埠、標定或憑證。
3. PR 附批次連結，審查並解決衝突後 Merge。合併不會讓未測項變成通過。
4. 原電腦原開發目錄處理本機修改後選 main，按 Fetch／Pull 取得合併後程式；原始證據仍另外保存。
5. 下一輪從更新後 main 開新分支，例如 hardware/acceptance-02；保留既有標籤與紀錄。

## 貼給另一台電腦 AI

> 請先讀 AGENTS.md、HANDOFF.md 與 docs/TWO_PC_WORKFLOW.md，回報分支與提交編號。這台是實機驗收電腦，請在 hardware/acceptance-01 接續。先核對環境／設定與原始碼，每次只改一個邏輯單元，不把軟體通過當實機通過。每批先固定受測程式提交，依 test_records/TEMPLATE.md 整理成功、失敗、中止及人工觀察，保留 run、設定與標定證據，協助製作 ZIP／SHA-256。摘要、交接、附件都上傳才算回傳成功；設備動作與無法判定的現場結果由操作者確認。

官方按鈕參考：[克隆](https://docs.github.com/en/desktop/adding-and-cloning-repositories/cloning-and-forking-repositories-from-github-desktop)、[同步](https://docs.github.com/en/desktop/working-with-your-remote-repository-on-github-or-github-enterprise/syncing-your-branch-in-github-desktop)、[附件發布](https://docs.github.com/en/repositories/releasing-projects-on-github/managing-releases-in-a-repository)。
