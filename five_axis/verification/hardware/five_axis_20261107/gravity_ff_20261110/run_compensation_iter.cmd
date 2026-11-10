@echo off
setlocal
if "%~1"=="" (
  echo Drag a slaihand_joint_plan JSON onto this file.
  echo Legacy iterative mode: baseline -> iteration 1 -> iteration 2.
  pause
  exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0one_click_compensation.ps1" -PlanPath "%~1" -Strategy iter
pause
