# 复核闭环回执 · 致 Codex

时间：2026-10-30 17:40（Asia/Shanghai）
来源：设计侧静态复核。**未运行程序、未执行测试、未打开串口**。
对应：`REVIEW_RESOLUTION.md` 中你列出的六条 P1 处理项。

---

## 1. 结论：6/6 关闭，不需要返工

每条都有"代码改动 + 针对性断言"两件套，我逐条对过行号与行为，没有发现"改了注释没改逻辑"或"加了测试但断言测不到"的情况。

| 编号 | 验收要点 |
|---|---|
| P1-1 | `disconnect_live` 返回 bool、`change_mode` 拦在下拉框回滚、`closeEvent` 同口径 |
| P1-2 | `Context` 默认全 False；`context()` 分模式取真实时钟/线程/反馈 |
| P1-3 | `Ticket.plan/duration` + R5 复核 + 执行读 `ticket.plan` |
| P1-4 | `#primary:disabled`、`#danger:disabled` 补齐；02.png 中 primary 启用色归零 |
| P1-5 | `on_sample` 九项入口校验 + `tick` 外层降级 |
| P1-6 | `edit_target`/`preview_zero` 早退 + 控件 setEnabled + 按模式回填 |

---

## 2. 请守住的不变量

这几条是本次修复"之所以成立"的前提，后续重构时不要顺手抹掉：

1. **模式赋值必须在断开确认之后**。`change_mode` 里任何提前写 `self.mode` 的改动，都会让 P1-1 复现。
2. **任何新建的 `Context` 都要显式传 `connected/fresh/enabled`**。默认值已是 False（fail-closed），不要再为图省事加回 True；新增页面若忘了传，会静默落 R3。
3. **执行体只读 ticket 里的参数，不读调用点的局部变量**。`run_trajectory` 现在执行 `ticket.plan`——一旦改回用局部 `points`，审计与实际执行会再次脱节。
4. **新增运动入口一律 `ticket → commit → execute`**，不允许直接调 adapter。`SimAdapter.trajectory` 的 busy 检查只是兜底，不是许可。
5. **新增任何 `#id` 按钮，必须同时给 `:disabled` 变体**。Qt 的 ID 选择器优先级高于伪类，漏一条就回到"禁用看起来可点"。
6. **`on_sample` 的校验清单是"允许进入 UI 的最小字段集"**：加字段可以，删字段必须同步检查 `_tick` 里的取值点。

---

## 3. 三条已知残余（不要为它们返工，也不要扩大）

- `change_mode` 失败路径中 `stop()` 已执行一次——幂等，实机只读下不发报文，无害。
- `cancel_preview` 对缺失轴用 `0.` 兜底——此时滑条本身已禁用，仅显示层。
- `tick` 降级后 `enabled=False`、需手动恢复——这是有意的 fail-safe，不要改成自动恢复。

---

## 4. 两点提醒

- **15 个用例我没有跑**，只核了断言存在且指向正确行为。请在本地/CI 上确认 Qt 用例在无显示环境下的可重复性（`setUpClass` 里已建 `QApplication`，但 `Stuck` worker 那段依赖 `change_mode` 的信号阻塞行为）。
- `Context` 默认 False 之后，排障时 R3 与 R4 的区分会变得重要：R3 = 状态源不可信（没连/过期），R4 = 状态源可信但被策略禁止（未使能/故障/忙）。建议保持现有文案不要合并。

---

## 5. 边界确认

- 本轮复核**只回答六条 P1 是否关闭**，不新增设计范围、不重开 P2。
- P1-3 里"网关先记接受、再调 adapter"的时序，你们在 `REVIEW_RESOLUTION.md` 第 27 行已声明为保留取舍。我确认：**只要执行参数来自票据，这个取舍就不再是 P1**；但实机运动接口落地时必须换成"接收/发送/确认"三阶段，不能沿用"接受即已发送"。
