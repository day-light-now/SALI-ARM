# C 阶段验收：采集与分析门槛需修正

2026-11-05。实现负责人仍为 DeepSeek。Codex 本轮只检查源码并新增无 ROS 初始化的离线反例，没有实机动作、参数应用或生产源码改写。

现有 test_calibration_core.py 19/19 与 fake_calibration_check.py 通过；这不足以通过现场采集验收。下列反例均由生产 Recorder._build()/calibration_core.analyze() 直接运行得出。

## 原样重现

```bash
python3 /mnt/d/CodexData/Projects/机械臂/slaihand_ros2_ws/verification/hardware/control_panel_20261104/codex_c_repro.py
```

输出：

```text
moving: PROVISIONAL_SCALE usable=12 scale=1.0
disabled: PROVISIONAL_SCALE usable=12 scale=0.0
mixed_versions: PROVISIONAL_SCALE usable=12 scale=1.0
negative_scale: PROVISIONAL_SCALE usable=12 scale=-1.0
cached_snapshot_age_s=0.20; returned_feedback_age=[0.01,0.01,0.01,0.01]
```

## 1. P1：纯静态数据资格未落实

validate_sample 只验证速度是有限四元列表，exclusion_reason 只看 support/contact。motion=up、速度 .8 rad/s 的记录仍进纯重力拟合；stage=8、全轴 effort=0 仍得尺度0；当前schema连阶段/电机状态都不是必填。

修复：采集保留不合格原始样本，但拟合必须要求实际新鲜四轴一致状态、使能/保持条件、无外部接触与支撑，且在明确持续窗口中速度与姿态稳定；不能只相信用户 --motion static。记录新反馈样本数、窗口跨度、速度/位置变化、motor_state、control_state和当前前馈状态。失能反馈仅用于几何位置检查，不能当作电机静态保持力矩。几何记录与力矩拟合资格分开。非静态及状态未知样本明示排除原因。

负/零尺度应是方向/状态/模型异常或不可辨识，不能称“可用尺度”；留出误差明显失败时同样应把资格与失败原因写明。保留诊断数值可以，但不能只靠永不输出CALIBRATED来代替输入资格。

## 2. P1：新鲜度漏算与非同步位置

Recorder._build 允许 JointState/DynamicJointState 缓存0.25秒，却原样复制消息中的 feedback_age=.01。反例两条消息均已缓存.20秒，仍返回.01，而实际至少.21秒，超过工具声明.06秒。

修复：反馈总年龄至少加本地接收后经过时间；优先从同一 DynamicJointState 快照取 position/velocity/effort，校验四轴状态一致；若必须混合话题，验证时间对齐。持续静止窗口不得用同一缓存/receipt反复累计。生产回调覆盖停流、重复快照、单轴缺帧、乱序及跨话题偏差。

## 3. P1：版本一致性和来源校验不完整

core.analyze 完全不检查样本之间的模型/映射/重力/偏置/负载版本，provenance最终是空对象。混合mapping hash反例仍正常拟合。CLI只比较header与当前磁盘文件，既不强制header完整，又未逐样本对header/当前模型核对；record的哈希也只来自仓库文件，不证明正在运行的驱动使用同一配置。负载标识缺失。

修复：建立版本一致性检查，必填合法哈希、单位、gravity向量、J3偏置、负载标识和采集时间；逐条对header及所选分析模型匹配，不一致分组或拒绝，不悄悄混拟合。运行中实际配置身份须有可验证来源；不能取得时标UNKNOWN，禁止该批作为可信驱动标定。不要仅因为当前文件恰好相同就当作部署已匹配。

## 4. 操作单和原始证据保护

- A标题“不需要运动”与要求J4两种不同角度容易误导，改为“工具不发运动；姿态调整由用户独立执行”。J2力矩符号判别需要使能稳定保持、自由无接触，失能手掰不能取得重力保持力矩。两姿态只是暂定模型候选判别，需记录实际角度、PD误差、反馈量化和摩擦不确定性；单条约0.2Nm符号不足以证明物理安装。
- 沿用用户已有安装声明，不再列为重复确认步骤；只在真实矛盾时询问。额外pi物理一致性仍待现场证据，离线模型对照不替代实物核验。
- 不让用户在--count12自动计时中一边掰动一边被当静态采样。提供逐姿态明确触发/稳定后采集与反馈，保留来向。未知支撑不能默认free。
- run_preview --save-report 必须拒绝与 raw 解析到同一路径（包括符号链接）；原始创建用独占模式与唯一文件名，不可覆盖；异常退出也应收尾保留原始只读证据。chmod0444只是权限设置，不是不可篡改证明。
- 静态数据通常可在足够激励下辨识某些重力基参数，但不能普遍断言“只能拟合全局尺度”。本工具主动限制只做全局尺度是合理范围选择，请据实说明，不把它当成一般不可辨识定理。

## 交付要求

集中修上述采集/拟合门槛，扩充真实Recorder回调与导入/预览失败路径测试，原样复跑reviewer反例。可保持低风险只读设计，不需要改驱动/增益/alpha来通过。继续alpha0，不先做D阶段扫描和Kp/Kd页。Codex复核通过后再引导现场采集；不安排用户先按当前操作单收集12条。
