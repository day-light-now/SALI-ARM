cmake_minimum_required(VERSION 3.8)
project(slaihand_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch web DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/send_small_test.py scripts/offline_panel.py scripts/panel_bridge.py scripts/commission_supported_trial.py scripts/control_panel.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/joint_waypoint_plan.py scripts/point_to_point.py scripts/control_panel_model.py scripts/control_panel_http.py scripts/control_panel_gravity.py scripts/control_panel_connection.py DESTINATION lib/${PROJECT_NAME})
# Provisional load declaration reader and J5 motion admission artifact are pure
# modules shared by the candidate launch and offline validation.
install(FILES scripts/load_declaration.py scripts/j5_motion_artifact.py scripts/five_axis_trajectory_groups.py scripts/five_axis_diagnostics.py scripts/five_axis_margin.py DESTINATION lib/${PROJECT_NAME})
# Legacy J5 read-only/controller modules stay in the source tree for historical
# evidence, but are deliberately NOT installed: the unified panel/C++ driver is
# the only production CANUSB owner. j5_calibration.py remains installable as the
# calibration data schema.
install(FILES scripts/calibration_core.py scripts/calibration_capture.py scripts/calibration_session.py scripts/drag_teaching.py scripts/supported_capture.py scripts/j5_calibration.py DESTINATION lib/${PROJECT_NAME})
ament_package()
