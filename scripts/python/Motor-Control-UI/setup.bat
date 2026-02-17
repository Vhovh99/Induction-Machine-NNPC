@echo off
REM Setup script for Motor Control UI on Windows
REM This script sets up the development environment and installs dependencies

echo.
echo ========================================
echo Motor Control UI - Setup Script
echo ========================================
echo.

REM Check Python version
echo Checking Python version...
python --version
echo.

REM Create virtual environment if it doesn't exist
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
    echo. Virtual environment created
) else (
    echo. Virtual environment already exists
)

echo.
echo Activating virtual environment...
call venv\Scripts\activate.bat

echo.
echo Upgrading pip...
python -m pip install --upgrade pip

echo.
echo Installing dependencies from requirements.txt...
pip install -r requirements.txt

echo.
echo ========================================
echo Setup Complete!
echo ========================================
echo.
echo To activate the environment, run:
echo   venv\Scripts\activate.bat
echo.
echo To start the application, run:
echo   python main.py
echo.
pause
