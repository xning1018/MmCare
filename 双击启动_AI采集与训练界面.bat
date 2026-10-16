@echo off
cd /d "%~dp0"
if exist "C:\Program Files\Python311\python.exe" (
    "C:\Program Files\Python311\python.exe" "scripts\collect_and_train.py"
) else (
    python "scripts\collect_and_train.py"
)
if errorlevel 1 pause
