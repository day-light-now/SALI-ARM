@echo off
chcp 65001 >nul
"C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe" -u "C:\Users\ASUS\Documents\机械臂\slaihand_ros2_ws\verification\launch_gravity_demo.py"
if errorlevel 1 pause
