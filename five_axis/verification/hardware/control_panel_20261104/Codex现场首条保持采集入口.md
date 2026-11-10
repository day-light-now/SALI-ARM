# 2026-11-05 首条保持采集入口

目标：alpha0原设置下，用户执行现有受保护客户端的当前位置10秒保持，只读采10个静止窗口反馈后归档。无轨迹、无自动复位、无增益或前馈应用。默认无execute只做预览。

capture_supported_hold.py 复用commission_supported_trial.py作为唯一运动客户端；先验证四轴未使能和运行身份，保持ready后采样，客户端结束确认后写入raw。用户仍需现场确认净空、支撑防护和切电条件，采样时机械臂无手或支架外力接触；做不到时不把样本标free。

现场工具两处补正：Recorder缺少snapshot方法，已补回实际spin/新鲜快照入口；STATIC_SPEED_RAD_S从.01改.03，因为J3/J4零速度最近反馈码为±.010989，旧门槛会永远排除静止。位置稳定门槛、状态、年龄、窗口和alpha0门槛均保留。

验证：test_calibration_core.py 34/34；verify_capture_supported_hold.py在隔离域177、新建PTY下真实Recorder/原客户端/ROS驱动归档通过，client_exit0、10样本、停止stage8。真实机械臂只读预览通过：[-2.535,-6.513,-20.474,25.551]°、身份匹配、未请求使能。日志/tmp/codex-hold-capture-check.log。

原始数据与hold_client结果保存在用户~/.ros/slaihand/calibration。首条样本不构成全域标定；暂定质量、J3模型偏置仍需实测验证。仅user执行带execute的实机入口。
