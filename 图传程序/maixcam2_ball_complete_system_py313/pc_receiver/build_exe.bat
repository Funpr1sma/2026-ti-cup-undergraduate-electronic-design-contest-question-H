@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv313\Scripts\python.exe" (
  echo Python 3.13 environment is not installed.
  echo Run install_and_run.bat first.
  pause
  exit /b 1
)
call ".venv313\Scripts\activate.bat"
python check_environment.py
if errorlevel 1 (
  pause
  exit /b 1
)
python -m pip install --only-binary=:all: "pyinstaller>=6.12,<7"
if errorlevel 1 goto :build_error

rem --onedir is more reliable and starts faster than --onefile for Qt + FFmpeg.
python -m PyInstaller ^
  --noconfirm ^
  --clean ^
  --windowed ^
  --onedir ^
  --name MaixCAM2_Ball_Receiver ^
  --collect-all av ^
  ball_video_receiver.py
if errorlevel 1 goto :build_error

echo.
echo Built application:
echo   dist\MaixCAM2_Ball_Receiver\MaixCAM2_Ball_Receiver.exe
pause
exit /b 0

:build_error
echo [ERROR] EXE build failed.
pause
exit /b 1
