@echo off
chcp 65001 >nul
setlocal EnableExtensions
cd /d "%~dp0"
title VisionPick - 智慧抓取操作台

echo.
echo ================================================================
echo   VisionPick 智慧抓取操作台
echo   Target: myCobot280 M5 + RealSense D405
echo ================================================================
echo.
echo   [1] 離線示範（不使用設備）
echo   [2] 相機測試（手臂不會移動）
echo   [3] 完整硬體（允許手臂移動）
echo   [Q] 結束
echo.
choice /C 123Q /N /M "選擇模式："
if errorlevel 4 goto :eof
if errorlevel 3 goto full
if errorlevel 2 goto camera
goto demo

:python
if exist ".venv-console\Scripts\python.exe" (
  set "VP_PY=.venv-console\Scripts\python.exe"
) else (
  set "VP_PY=python"
)
%VP_PY% run_v0.py %*
exit /b %ERRORLEVEL%

:demo
call :python ui --demo --mode offline --llm mock
pause
exit /b %ERRORLEVEL%

:camera
call :python ui --hardware --mode realsense_only --llm api
pause
exit /b %ERRORLEVEL%

:full
echo.
echo [SAFETY] 繼續前請清空手臂整段移動路徑。
echo [SAFETY] 危險時請使用手臂實體急停。
call :python ui --hardware --mode full --llm api --enable-motion
pause
exit /b %ERRORLEVEL%
