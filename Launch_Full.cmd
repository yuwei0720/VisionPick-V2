@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"
title VisionPick - Full Hardware Console
echo [SAFETY] 危險時請使用手臂實體急停。
echo [SAFETY] 繼續前請清空手臂整段移動路徑。
echo [INFO] 設備： myCobot280 M5 + RealSense D405
echo [INFO] 模式：完整硬體（相機、手臂與AI）
if exist ".venv-console\Scripts\python.exe" (
  ".venv-console\Scripts\python.exe" run_v0.py ui --hardware --mode full --llm api --enable-motion
) else (
  python run_v0.py ui --hardware --mode full --llm api --enable-motion
)
pause
