@echo off
setlocal
if "%~1"=="" (
  echo Drag a joint plan JSON onto this file to build a model-compensated plan offline.
  echo No hardware or driver is started by this helper.
  pause
  exit /b 2
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0make_comp_plan.ps1" -Plan "%~1"
pause
