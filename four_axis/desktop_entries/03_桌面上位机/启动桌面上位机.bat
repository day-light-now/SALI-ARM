@echo off
chcp 65001 >nul
set "SLAI_PY=C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe"
cd /d "D:\CodexData\Projects\机械臂\slaihand_ros2_ws"
"%SLAI_PY%" -m slai_console.launch
if errorlevel 1 pause
