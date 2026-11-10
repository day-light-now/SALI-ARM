# 历史实验：OpenArm 五轴适配（已停用）

> 当前已切回四轴。以下是未完整验证的旧实验记录，不代表当前程序。
> 请使用桌面四轴入口，阅读 `OpenArm四轴适配说明.md`。五轴 xacro 不可直接启动。

当前完成离线控制适配：五轴控制器 → SLAI 五轴硬件接口 → OpenArm MIT 指令编码 → 虚拟电机反馈。
这一步不包含真实 CAN 发送，也不包含动力学或重力补偿。

## 使用

双击桌面 `启动SLAI五轴OpenArm仿真.bat`，然后双击 `SLAI五轴小幅运动.bat`。
RViz 中 J1–J4 显示已确认的 SLAI 网格；J5 显示 `link5_virtual` 坐标轴，没有虚构电机外形。
J5 不改变前四个 link 的姿态；它只改变自己的虚拟子坐标系。
控制接口始终使用 `slaihand_joint1` 到 `slaihand_joint5`。
旧四轴桌面入口使用同一个最新启动文件，现在也会进入五轴；建议使用新五轴入口。

## 电机与编码配置

| 轴 | 计划实物电机 | 当前离线 OpenArm 编码 profile |
|---|---|---|
| J1/J2 | DM-J10010L-2EC | DM10010L |
| J3/J4 | DM-J8009P-2EC | DM8009 |
| J5（虚拟） | DM-J4340-2EC | DM4340 |

上述 profile 是上游库的离线测试配置。8009P 与 8009 等型号的 2EC 固件参数必须实测确认，
不能仅凭名字把库内 pMax/vMax/tMax 当成实际电机规格。
目前 MIT 包 kp、kd、tau 全为 0，包只在内存中生成；虚拟电机直接跟随位置目标，
因此这不是 MIT 阻抗闭环或电机动力学仿真。
编码器 profile 的位置编码范围也不是机械臂限位。

## 复用及修改

- 上游 CAN 库保持原样复制，实际编译并链接 `OpenArmCAN::openarm_can`。
- 复用 OpenArm ROS 硬件接口的生命周期和读写结构，改成 SLAI 五轴位置接口。
- 去掉七轴和夹爪依赖，按名称检查五个关节，读取 J1–J4 的 URDF 限位。
- 接口激活时保持当前状态，回零由显式轨迹完成。
- 映射公式：`motor = sign * joint + zero`；`joint = sign * (motor - zero)`。
- `sim_sign=1`、`sim_zero_offset=0` 仅为仿真默认值，不写入实际电机映射表。
- `transport` 只接受 `fake`；实现中没有 CAN socket，也没有电机使能调用。
- 可用 `backend:=generic` 切回 ROS 通用假硬件进行对照。

来源：

- https://github.com/enactic/openarm_ros2
  - 本地引用 commit：`4e837e1d0dae692ff67b560b69d8d281d7a8d4ed`
- https://github.com/enactic/openarm_can
  - 本地引用 commit：`f340d4b808fb177e1f297af54eb55fd51c6c7c10`

版权及 Apache-2.0 许可保留在源码和 LICENSE 中。

## J5 的占位假设

J5 的位置暂与 link4 原点重合，轴向暂为局部 +Z，零位暂为 0。
这些只定义虚拟坐标系，不代表实际安装面、轴方向或电机编码器零点。
J5 用 continuous 表示尚未确定机械限位；0.5 rad/s 为演示速度。
有新 CAD 后替换 J5 的 origin、axis、网格和真实限位，五轴轨迹接口无需改名。

实机 CAN ID、方向、零偏均仍为 null。真实 CAN 驱动接入、反馈超时/故障处理、
增益标定及物理限位验证留待单轴实测阶段。

## 构建与验证

Windows 源码目录为本文件所在目录。Linux 编译目录 `/home/xuyu/slaihand_ros2_ws`。
运行 `build_workspace.sh` 同步并编译。运行 `colcon test --packages-select slaihand_hardware`
检查符号/零偏映射和三种 OpenArm MIT 编码；启动控制器后运行 `verify_fake_control.py`
验证逐轴、五轴联合及回零轨迹。
