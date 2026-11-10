@echo off
setlocal
if "%~1"=="" (
  echo Drag a slaihand_joint_plan JSON onto this file.
  echo It will ask you to type RUN to confirm supports / clearance / power-cut.
  pause
  exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0one_click_compensation.ps1" -PlanPath "%~1"
pause
