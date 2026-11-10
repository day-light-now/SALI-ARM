@echo off
setlocal
if "%~1"=="" (
  echo Drag a plan JSON onto this file for a dry run (admission/artifact/driver only, no motion).
  pause
  exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0one_click_compensation.ps1" -PlanPath "%~1" -DryRun
pause
