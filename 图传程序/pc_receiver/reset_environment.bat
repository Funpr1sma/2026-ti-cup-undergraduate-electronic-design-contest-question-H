@echo off
setlocal
cd /d "%~dp0"
if exist ".venv313" (
  rmdir /s /q ".venv313"
  echo Removed .venv313.
) else (
  echo .venv313 does not exist.
)
echo Run install_and_run.bat to reinstall.
pause
endlocal
