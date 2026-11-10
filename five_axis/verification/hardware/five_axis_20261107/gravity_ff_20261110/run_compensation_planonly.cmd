@echo off
setlocal
if "%~1"=="" (
  echo Drag a slaihand_joint_plan JSON onto this file.
  echo Offline only: validates the plan and emits the model-compensated plan.
  echo No driver is started and no motion happens.
  pause
  exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0one_click_compensation.ps1" -PlanPath "%~1" -PlanOnly
pause
