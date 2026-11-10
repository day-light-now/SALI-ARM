# OpenArm 四轴适配与交接（2026-10-07）

按用户最新决定，当前正式配置只有 J1–J4。四轴桌面入口使用四轴描述。
旧五轴说明和 URDF 是历史实验，不能作为当前运行状态的依据。

## J2 正方向修正（2026-10-07）

用户确认 J2 正角度旋转方向反了：正式四轴 URDF 将 J2 局部轴从 -Z 改为 +Z。
零位、origin、网格和其他轴不变。限位保持 -10° 到 +190°，新的正向朝原来的负向运动。
这不是对旧物理范围的等价重命名，而是根据用户碰撞反馈修正允许运动的物理方向。
不同时取反限位；否则物理扇区又会回到原来的碰撞侧。
旧五轴和旧独立 MuJoCo 模型未同步，均不是当前四轴运行入口。

验证：编译通过；面板/描述单元测试 4 项通过；canonical 和展开 URDF 校验通过（缺惯量警告仍在）；
check_urdf 通过；live J2 +5° 的 ROS TF 与新 +Z 旋转一致、与旧 -Z 不一致。
CAD Viewer 网格加载报 Invalid typed array length，未完成全限位视觉扫轴，不保证机械无碰撞。
实机电机方向和编码器零偏仍未知，没有改变实机映射，也没有发实机指令。

## J3 旋转中心修正（2026-10-07）

用户修正 CAD 后，读取当前 slaihand_four_axis.SLDASM 中的新坐标系 J3（旧 J3_cs 不存在）。
使用现有 CAD 提取脚本重新导出 link 网格；CAD J3_axis 与新 J3 Z 轴共线。
新轴相对旧轴横向移动约 57.4507 mm，另有沿轴约 135.0533 mm 的原点变化。
正式模型同步更新 link3.obj、J3 origin、J4 origin，不改 J2 +Z、其他轴方向或限位。
零位世界坐标网格最大误差约 2.15e-8 m，说明零位装配保持；新 J3 轴到 CAD 轴线误差小于 1e-8 m。

源数据与脚本在 verification/cad_j3_corrected、extract_corrected_cad.py、prepare_j3_mesh.py。
修改前 URDF 和 link3.obj 备份在 verification/before_j3_correction，可恢复。
verification/review_j3_correction.py 校验正式 J3/J4 变换与新 CAD 一致，并输出 ±90°几何对比图。
该图确认绕新中心旋转的连接外观，不代替网格碰撞检测或完整全轴视觉扫轴。
CAD Viewer 先前对 OBJ 加载失败，未宣称完成其全行程检查。

## 技能

已使用 find-skills 搜索 ROS 2 技能，并使用现有 urdf 技能核验描述。
候选 ros2-robotics / ros2-cli 源码获取遇到网络问题及 API 限流，未安装未核验的技能。
URDF 技能要求保留 CAD 坐标真值、验证展开描述、不伪造惯量，因此本轮不改几何或补猜测惯量。

## 复用边界

- 上游 https://github.com/enactic/openarm_can ，commit f340d4b808fb177e1f297af54eb55fd51c6c7c10，源码未改，实际编译链接。
- ROS 接口结构参考 https://github.com/enactic/openarm_ros2 ，commit 4e837e1d0dae692ff67b560b69d8d281d7a8d4ed。
- 未运行上游七轴硬件驱动，未照搬默认 ID 或激活自动回零。
- 插件 `slaihand_hardware/OpenArmJointFake` 仅接受 transport=fake。
- 关节数、名字、限位、编码 profile 和仿真速度从描述读取；支持四/五轴，但正式配置为四轴。
- 映射 motor = sign * joint + zero；仿真 sign=1、zero=0 不代表实际标定。
- J1/J2 使用 DM10010L，J3/J4 使用 DM8009 离线编码 profile；8009P-2EC 参数未实测核验。
- MIT 包仅存内存，kp/kd/tau=0；反馈是限速位置跟随，不是物理或阻抗仿真。

## 本轮验证

1. 四个 ROS 包编译通过，展开描述包含四个控制关节及正确插件。
2. 映射/编码单元测试：1 test，0 errors，0 failures。
3. ROS_DOMAIN_ID=47 隔离运行，假硬件、状态广播及轨迹控制器成功激活。
4. verify_fake_control.py 的 11 段全部通过：逐轴运动与归零、联合运动与归零；反馈误差要求小于 0.002 rad。
5. cadgen 校验展开 URDF：0 errors，四条 missing_inertial 警告；5 links、4 movable joints、5 个网格引用解析成功。
6. 桌面小幅运动对应 send_small_test.py 完成，返回成功。

隔离测试进程已停止；用户正常启动时使用默认 ROS domain，不受测试节点干扰。

验证范围是离线编码、ROS 轨迹和反馈，不包含实机 CAN、真实机械限位或动力学。
未完成 CAD Viewer 全限位视觉扫轴；不能据此声称全行程无碰撞。
编译有 Jazzy 旧 on_init 接口弃用警告；未影响当前测试，未来应迁移新版接口。

## 文件与扩展

正式描述：src/slaihand_description/urdf/slaihand_four_axis.urdf.xacro。
硬件实现：src/slaihand_hardware/src/openarm_joint_fake.cpp。
控制器：src/slaihand_bringup/config/slaihand_controllers.yaml。
真实电机映射表维持 null；没有新增实机值。

J5 建模完成后确认安装变换、轴向、零位、限位及惯量，再新增 joint5 配置、控制器列表与轨迹维度。
旧五轴 xacro 含旧插件名，仅历史留档，不应启动；未来从当前四轴配置扩展。
# J4 最新 CAD 校正记录（2026-10-07）

读取活动 `slaihand_four_axis.SLDASM` 的“坐标系1”和 `J4_axis`。
J4 父坐标变换 xyz 为 `0.04125 1.25982557719e-13 0.1395` 米，
rpy 为 `-3.14159265359 -1.57079632679 0` 弧度。
同时重新导出 link4 网格；J1/J2/J3 坐标、网格、方向均未改变。
J4 保留 local -Z 正向和 -90°～+50°限位，CAD 轴线残差约 6.86e-18 米。
最新 CAD 的 link4 零位几何与旧版最大双向最近顶点差约 3.859 mm，
以新装配体为准，不宣称零位几何完全不变。旧版备份位于
`verification/before_j4_correction`，新 CAD 捕获位于 `verification/cad_j4_corrected`。
比较图 `verification/j4_pivot_comparison.png` 展示 J4 两端姿态，不是碰撞检查。
URDF 校验、check_urdf、四包构建、四项离线单元测试通过。
控制面板集成测试（运动、取消、限位拒绝、回零）通过，结束于零位。
CAD Viewer 0.7.6 加载 OBJ 报 Invalid typed array length，未完成查看器全轴扫限位。
已用 CAD 轴线数值检查和 J4 端点几何比较图替代部分检查；完整视觉扫限位仍待确认。
仍缺惯性和碰撞模型，不能用于已验证的重力补偿或实机限位安全结论。
