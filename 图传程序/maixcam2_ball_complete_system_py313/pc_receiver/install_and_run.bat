@echo off
setlocal EnableExtensions
cd /d "%~dp0"

set "VENV=.venv313"
set "PY313="

echo ============================================================
echo  MaixCAM2 Ball Receiver - Python 3.13 x64 installer
echo ============================================================

where py >nul 2>nul
if not errorlevel 1 (
    py -3.13 -c "import struct,sys; raise SystemExit(0 if sys.version_info[:2]==(3,13) and struct.calcsize('P')==8 else 1)" >nul 2>nul
    if not errorlevel 1 set "PY313=py -3.13"
)

if not defined PY313 (
    where python >nul 2>nul
    if not errorlevel 1 (
        python -c "import struct,sys; raise SystemExit(0 if sys.version_info[:2]==(3,13) and struct.calcsize('P')==8 else 1)" >nul 2>nul
        if not errorlevel 1 set "PY313=python"
    )
)

if not defined PY313 (
    echo [ERROR] 64-bit CPython 3.13 was not found.
    echo Install Python 3.13 x64 and enable the Python Launcher or PATH option.
    pause
    exit /b 1
)

if exist "%VENV%\Scripts\python.exe" (
    "%VENV%\Scripts\python.exe" -c "import struct,sys; raise SystemExit(0 if sys.version_info[:2]==(3,13) and struct.calcsize('P')==8 else 1)" >nul 2>nul
    if errorlevel 1 (
        echo Removing an incompatible virtual environment...
        rmdir /s /q "%VENV%"
    )
)

if not exist "%VENV%\Scripts\python.exe" (
    echo Creating Python 3.13 virtual environment...
    %PY313% -m venv "%VENV%"
    if errorlevel 1 goto :venv_error
)

call "%VENV%\Scripts\activate.bat"

echo Updating pip...
python -m pip install --upgrade pip setuptools wheel
if errorlevel 1 goto :install_error

echo Installing Python 3.13 binary wheels...
python -m pip install --only-binary=:all: --upgrade -r requirements.txt
if errorlevel 1 goto :install_error

echo Checking the environment...
python check_environment.py
if errorlevel 1 goto :install_error

echo Starting receiver...
python ball_video_receiver.py
exit /b %errorlevel%

:venv_error
echo [ERROR] Failed to create the Python 3.13 virtual environment.
pause
exit /b 1

:install_error
echo.
echo [ERROR] Dependency installation or verification failed.
echo Confirm that Python is 64-bit and that the computer can access pypi.org.
echo You can delete the .venv313 folder and run this file again.
pause
exit /b 1
