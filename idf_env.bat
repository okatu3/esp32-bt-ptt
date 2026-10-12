@echo off
rem ESP-IDF 5.5.1 + ESP-ADF environment for this project (works without changing the PowerShell execution policy).
rem Usage:  idf_env.bat build
rem         idf_env.bat -p COM12 flash monitor
set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf5.5_py3.9_env
set ADF_PATH=C:\Users\user\esp-adf
call C:\Espressif\frameworks\esp-idf-v5.5.1\export.bat >nul || exit /b 1
cd /d "%~dp0"
idf.py %*
