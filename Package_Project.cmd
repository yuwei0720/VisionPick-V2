@echo off
chcp 65001 >nul
setlocal EnableExtensions
cd /d "%~dp0"
if exist ".venv-console\Scripts\python.exe" (
    ".venv-console\Scripts\python.exe" -X utf8 -B "package_project.py" %*
) else (
    python -X utf8 -B "package_project.py" %*
)
set "VP_PACKAGE_EXIT=%ERRORLEVEL%"
if not "%VP_PACKAGE_EXIT%"=="0" echo [ERROR] Packaging failed. See the message above.
if "%~1"=="" pause
exit /b %VP_PACKAGE_EXIT%
