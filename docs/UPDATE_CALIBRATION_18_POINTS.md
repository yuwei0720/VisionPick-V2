# 使用本次18點替換標定

## 已準備的檔案

- 新副本：`calibration/calibration_points_18.csv`。保留標題，排除原檔第2–4行（三筆資料），留下原第4–21筆，共18筆。
- 舊設定備份：`docs/calibration_analysis_20261007/v0.before_18.yaml`。
- 原壓縮檔與原CSV未修改，目前`config/v0.yaml`也未修改。

此副本依你回憶的18點批次整理；數字下降本身不能證明採點來源，也不等於完成精度驗證。

## 命令

先停止正在執行的辨識／抓取流程，使用原採點的Python環境，在PowerShell切換到專案根目錄：

```powershell
Set-Location -LiteralPath 'E:\下載\VisionPick_DEV9_PlaceUI_Update\VisionPick_DEV9_Final'
```

先計算，不寫入設定：

```powershell
python tools/solve_calibration.py calibration/calibration_points_18.csv
```

本次檔案應顯示`Fit RMSE = 7.465 mm; max = 12.233 mm`。若點數、檔案內容或結果不同，先核對輸入檔。

替換矩陣：

```powershell
python tools/solve_calibration.py calibration/calibration_points_18.csv --update-config
```

這會更新既有`config/v0.yaml`中的`calibration.T_robot_from_camera`、`calibration.rmse_mm`，並將`calibration.ready_for_motion`設為`false`。不需要手動複製矩陣，也不會由這條命令移動手臂。執行後重新啟動程式以載入新設定。

## 精度驗證與啟用

另採5–10個未參與新舊標定的點，包含不同平面位置與實際盒面／接觸高度，另存`calibration/calibration_holdout.csv`。使用相同座標原點、開口與姿態；不要追加到18點檔，也不要拿排除的前三點替代獨立驗證。

有驗證檔後，先評估、不更新設定：

```powershell
python tools/solve_calibration.py calibration/calibration_points_18.csv --holdout calibration/calibration_holdout.csv
```

查看`Holdout RMSE`、`max`、XYZ方向偏差與逐點誤差，確認符合實際夾取容差。採點擬合RMSE低於設定的10 mm門檻，並不代表所有抓取位置都已通過。

目前保持`ready_for_motion: false`。完成獨立點驗證及人工控制下的低速實機對點檢查，確認位置、姿態與既有放置位置適用後，才在`config/v0.yaml`將`calibration.ready_for_motion`改為`true`，重新啟動程式。

## 還原舊設定

如需還原，先停止流程，再執行：

```powershell
Copy-Item -LiteralPath 'docs/calibration_analysis_20261007/v0.before_18.yaml' -Destination 'config/v0.yaml' -Force
```

此命令會還原整份舊設定，包含原本的動作啟用狀態；重新啟動程式前核對設定。
