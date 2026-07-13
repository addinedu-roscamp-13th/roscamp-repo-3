# Windows PowerShell 노트북용 실행 스크립트. 가상환경을 활성화한 뒤 사용합니다.
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root
if ($env:PYTHON_BIN) {
    & $env:PYTHON_BIN run_server.py
} else {
    & python run_server.py
}
