@echo off
REM Windows 노트북용 실행 스크립트. 가상환경을 활성화한 뒤 사용합니다.
setlocal
set "ROOT_DIR=%~dp0.."
cd /d "%ROOT_DIR%"
if "%PYTHON_BIN%"=="" set "PYTHON_BIN=python"
%PYTHON_BIN% run_server.py
