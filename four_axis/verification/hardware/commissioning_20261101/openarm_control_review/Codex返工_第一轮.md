# Codex独立验收：第一轮未通过

已核对before_complete.json：作用域内2个新增、5个修改，没有范围外变更。原样运行9模块116项，114项通过、2项因Reader日志命名碰撞FileExistsError错误，不能称原命令通过。交付报告已说明shim，但验收须严格区分。

## 必修1：过零误把快减力用于反方向起动力矩
j4_control.shape_torque：previous=+0.01、requested=-2、dt=.01、velocity=0、tracking=true、motion_confirmed=true，当前输出-0.09，rate=10、release=true。从正0.01退出只需.001s，其余时间反向幅度应受slew_up=1约束；不能用快减率直接反向加力。生产真实调用路径必须使用修复后的同一函数。补双向、恰好过零、dt小于/等于/大于过零时间的测试，验证有界减力与反向加力分别守各自速率。FD仍直接绕过。

## 必修2：起动上限被后置阻尼突破
profile=2Nm brake，requested=-1、previous=-1、dt=.04、motion_confirmed=false、tracking=true、model_velocity=.19、reference_velocity=0，当前pre_slew=-1.28、torque=-1.04。界面声称起动≤1Nm，但低于0.2速度中止的合法输入已超过起动上限。确保起动期复合输出（包括阻尼和量化编码）的绝对值不越所选起动边界，或明确单独定义并审查制动输出边界，不得一边标≤1一边输出更多。回归须覆盖双向、速度与位置误差反向。保持总2Nm上限。考虑motion_confirmed当前非锁存逻辑回抖导致起动相反复打开，若改需每程独立重置并写清。

## 必修3：停止后采集并不保证获得新的反馈，首帧/行程字段可能不实
live_trial在主while完成stop确认之前没有capture_post；进入finally时第一state0可能已被_drain消费/覆盖，finally的纯被动_drain又不产生查询，在请求-回复总线上不会自行持续有帧。不能把最新缓存称首帧，或把停止后的缺帧当零滑行。
在不改actuation_io.py/Reader保护源码前提下：在每次session.cycle前后尽早收集实际可见的停止后反馈并保留receipt；标注是否保证首帧/有无丢帧以及观测覆盖。仅当四轴已新反馈确认DISABLED时，允许在有限观测窗口内用现有只读CC请求轮询J4，并记录TX/RX；只准7FF的04 00 CC读取，不改参数、不发送MIT/FC，不查询J5。UNKNOWN时不得主动查询去刷新仍可能使能电机的watchdog，保持未知锁定/被动读与现场检查提示。没有新样本时明确insufficient/no_new_feedback，不能声称0滑行。窗口末尾更新最终位置/速度和判据：若poststop新增可见位置证明返回位置超出0.5°，不能维持round_trip_criterion_met=true。真实覆盖/缺帧限制必须写清。补具有离散滑行/按查询才回包/缓存会覆盖的fake生产路径测试，证明不是只靠给一个静态state0就通过。

## 验收与交付
只修以上具体问题，保留已通过部分和原最高2Nm/0.2rad/s/4°/0.02目标速度等全部边界；不要重新调大参数或翻新结构，不碰硬件/保护文件/原始证据。
新增反例先运行失败再修复；独立新增文件名唯一的测试fixture可写在测试层，不改生产Reader，不通过全局替换时间掩盖控制时效问题。若需要shim，仅作用于日志命名并明确报告原样命令结果和shim结果。报告新增：Codex返工修正说明.md，并更新交付说明中不再成立的声明，保留历史事实。
完成后等待Codex验收，不让用户直接执行新档位。
