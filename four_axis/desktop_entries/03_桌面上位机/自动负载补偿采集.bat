@echo off
chcp 65001 >nul
cd /d "D:\CodexData\Projects\机械臂\slaihand_ros2_ws"
"C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe" -m slai_console.auto_load_console
if errorlevel 1 pause
