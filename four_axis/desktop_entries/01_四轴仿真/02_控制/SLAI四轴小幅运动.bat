@echo off
wsl.exe -d Ubuntu-24.04 -u xuyu -- bash -lc "source /opt/ros/jazzy/setup.bash; source /home/xuyu/slaihand_ros2_ws/install/setup.bash; ros2 run slaihand_bringup send_small_test.py"
pause
