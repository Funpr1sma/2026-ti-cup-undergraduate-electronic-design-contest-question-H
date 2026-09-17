@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv313\Scripts\python.exe" (
  echo Python 3.13 environment is not installed.
  echo Run install_and_run.bat first.
  pause
  exit /b 1
)
".venv313\Scripts\python.exe" check_environment.py
if errorlevel 1 (
  pause
  exit /b 1
)
".venv313\Scripts\python.exe" ball_video_receiver.py
endlocal
