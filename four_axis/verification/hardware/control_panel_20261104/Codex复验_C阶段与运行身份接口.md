# C 返工复验：剩余边界与运行身份接口

2026-11-05。DeepSeek 继续实现。原 codex_c_repro.py 已重跑，四类均 INSUFFICIENT；35项测试通过。此前支撑/静态资格、版本分组、负尺度及证据保护工作保留。本轮尚不能关闭全部采集门槛，以下精准修复与身份接口一起完成，不再等待权限确认。

## 三项剩余反例（生产函数）

1. Recorder._build() 使用 total_age=max([local_age]+driver_ages)，并非文档描述的相加。测试先构造RecorderCase._recorder，再将四轴feedback_age设.035、dynamic.at设time.monotonic()-.035。实际至少.070秒，函数仍返回capture_age_s=.035。必须每轴total=driver_age+local_receipt_age，校验非负和有限，比较原始精度后再显示舍入；保存原始驱动年龄、本地经过时间及每轴总年龄，不再混用字段含义。考虑订阅消息传输时间未知的局限，如无法界定只作下界说明。
2. _window_aggregate() 只取最后一帧stage/motor_state。五帧静止数据中第一帧stage8/motor_state全0、其余stage5/全1，聚合仍报告stage5/全1。必须全窗逐帧满足条件并记录证据；换command_id、失能、停流或身份变化应重置窗口，不拼接成连续稳定。增加最大采样间隙和窗口内freshness/FF状态一致性校验；当前只限制总跨度不证明持续。
3. driver_ff 只验证alpha是否0，不检查age_s、stage、command_id或全窗状态。diverse_samples()每条driver_ff.age_s=120，仍PROVISIONAL_SCALE。必须用当前驱动会话下新鲜前馈状态；缺失/过期/非零/阶段或命令不符应拒绝拟合并保留诊断。默认alpha0时话题当前1Hz刷新，设计合理的新鲜度阈值或调整明确的发布频率，不能机械用.06秒导致正常消息必拒。

以上反例没有初始化ROS或访问串口。精确复算：

```python
from test_calibration_core import RecorderCase, CollectStaticCase, diverse_samples, gravity
from calibration_capture import _window_aggregate
import calibration_core as c
import time
r=RecorderCase()._recorder(time.monotonic())
for row in r.dynamic['axes'].values(): row.update(feedback_age=.035)
r.dynamic['at']=time.monotonic()-.035
print(r._build()['capture_age_s'])  # 当前 .035，期望拒绝
x=CollectStaticCase()
rows=[x._snapshot(float(i),i) for i in range(5)]
rows[0]['stage']=8;rows[0]['motor_state']=[0]*4
print(_window_aggregate(rows)['motor_state'])  # 当前全1，不能证明全窗使能
rows=diverse_samples()
for row in rows: row['driver_ff']['age_s']=120.
print(c.analyze(rows,gravity)['status'])  # 当前PROVISIONAL_SCALE，期望INSUFFICIENT
```

## 运行身份接口：在现有授权内继续实现

不必另请“是否可改驱动”许可。仅做必要的只读元数据发布与采集消费，不新增硬件写通路，不使能，不自动部署实机。

- 用版本化结构化消息（现有可用标准消息封装JSON也可），包含真实加载后配置：URDF/model、mapping、policy、质量档案/紧凑重力模型哈希，mass status，gravity_base，J3固定偏置，load_id，实际kp/kd/alpha/ramp、策略版本等。元数据需从驱动实际接受的参数/模型生成并一致性校验，不能用当前仓库文件冒充运行值。
- 驱动启动/重新configure生成boot/session ID；消息含schema、启动标识、配置hash及序号/时间依据。静态身份可RELIABLE+TRANSIENT_LOCAL；单个latched消息不证明发布者仍活着，用同一会话的周期状态/心跳验证活性。重启、重复发布者、消息旧会话或配置不符要回UNKNOWN并重置采样窗。
- 身份核对不是质量/COM物理标定；verified只表示采集数据与运行配置匹配，不允许因此把暂定质量改成CALIBRATED。load_id是声明，需要记录用户提供来源。
- 沿用四轴唯一命令入口、alpha0默认、180秒会话及原保护。可只在启动配置身份里记录load_id，不必为此新增运行时修改机制。
- 测试必须含正确匹配、模型/质量/负载不符、旧latched、驱动重启、断流、同窗command_id变化、零/非零alpha时效、UNKNOWN只诊断。通过后提供实际可执行的只读预览入口。

只补上述缺口，避免扩大成新控制系统或调参页。修完集中交付；Codex再确认是否可以进入用户现场数据采集。
