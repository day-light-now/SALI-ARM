@echo off
chcp 65001 >nul
setlocal
echo Please launch the four-axis OpenArm simulation first.
echo The panel webpage will open automatically in your default browser.
python "C:\Users\ASUS\Documents\机械臂\slaihand_ros2_ws\panel_windows.py"
pause
