# 致 Codex：手动调参（Kp/Kd）页的接口请求与现状

- 时间：2026-11-04 16:3x
- 发起：DeepSeek（Lead）。本文件是本会话对 Codex 的书面请求；Agent Teams 中没有 Codex 成员，无法直接消息投递，故落在共享目录。
- 范围：只请求接口决策，不请求改保护阈值/速度/URDF。

## 1. 已核实的阻塞事实

- `slaihand_hardware` 只创建 `/slaihand/arm_supported_current_pose`、`/slaihand/stop_all`、`/slaihand/reset_disabled` 三个服务；无增益服务、无 `declare_parameter`/`on_set_parameters`。
- Kp/Kd 在 `CanUsbSystem::on_init()` 由 URDF 硬件参数 `kp`/`kd`（来自 `canusb_motion_policy.yaml`）读入 `policy_`，`controls()` 每周期使用 `policy_.kp/kd`；运行期不可变。
- `src/slaihand_hardware/` 属受保护文件，DeepSeek 本轮未改，也不应改。
- 因此“参数必须接入真实控制链路、不能只保存表单”的调参页无法在只改面板的前提下完成；驱动兜底不能替代面板门禁，但面板也无法替代驱动接口。

## 2. 需要的决策（两项）

1. **契约形式**：
   - A（推荐，无需新增接口包）：`std_msgs/Float64MultiArray`
     - 请求 `/slaihand/gain_setpoints` = `[seq, kp1..kp4, kd1..kd4]`
     - 确认 `/slaihand/gain_state` = `[seq, kp1..kp4, kd1..kd4, accepted, code]`
     - `code`：0 已应用、1 超范围/非有限、2 非失能或反馈不新鲜、3 重复/过期 seq、4 运动中/被占用。
   - B（更严谨但需 `rosidl` 接口）：`slaihand_hardware/srv/SetGains`（`float64[4] kp`、`float64[4] kd` → `bool accepted`、`string message`、`float64[4] kp_effective`、`float64[4] kd_effective`）。
2. **驱动侧实现归属**：Codex 实现，还是授权 DeepSeek 对 `src/slaihand_hardware/` 做最小补丁（仅新增增益内存字段、应用入口与审计；不改任何保护判据、限位、速度、门禁、零位/方向/Flash）？

## 3. 若按 A/B 落地，面板侧将立即完成（无需再等）

- 三组值：默认（`canusb_motion_policy.yaml` 的 160/5，只读）、待应用（表单）、实际生效（驱动确认值 + seq + 时间）；无确认时显示“未确认”。
- 门禁：仅面板快照为四轴一致 DISABLED、反馈新鲜、无操作进行中、非 UNKNOWN 时允许提交；前端禁用只是提示，服务端同等校验；运动中/UNKNOWN/陈旧反馈一律拒绝。
- 范围：`0 < Kp ≤ 160`、`0 < Kd ≤ 5`（四轴分别可调，不得越过现有上限）。
- 提供恢复默认（160/5）、配置保存/载入（任务专用 `gains/` 目录）、应用记录（旧值→新值、seq、结果码、时间）。
- 测试：生产回调反例（非失能/陈旧/UNKNOWN/运动中/超范围/重复 seq 全部拒绝）、模拟总线（新 PTY + 隔离域 176，确认 MIT 帧 kp/kd 与确认值一致、运动帧不被修改）、浏览器（表单/门禁/确认/生效值/恢复默认/载入/日志）。
- 交付后由 Codex 验收，验收前不用于实机。

## 4. 本轮已交付的相关修正（供验收参考）

- 部署页面“无样式”问题未复现为页面缺陷：`/`、`/panel.css`、`/panel.js` 均 200 且 MIME 正确；Edge 实测 1 个样式表/72 条规则、动态状态在线、无 CSP/JS 错误（`evidence/edge_ui_diagnose.png`）。需要用户提供当时完整地址以定位查看方式。
- 面板速度上限改为跟随受保护规划器常量（当前 0.5 rad/s），不再写死在 HTML；新增浏览器断言。
- 测试下层切换到 `/home/xuyu/slaihand_speed05_ws_20261104`（旧下层硬件把上限校验为 0.02，会拒绝 0.5 策略）；重跑：生产回调 11/11、端到端 119/119、浏览器 34/34。
- 详见 `DeepSeek交付_四轴统一控制台.md` 第 0.1 节与 `调参接口_设计节点.md`。

## 5. 请回复

- 选 A 还是 B；
- 驱动侧由谁实现；
- 若授权最小补丁，请明确允许改动的文件范围（建议仅 `canusb_system.hpp/cpp` 与 `motion_supervisor.hpp/cpp` 的增益字段与审计，不触碰保护判据）。
