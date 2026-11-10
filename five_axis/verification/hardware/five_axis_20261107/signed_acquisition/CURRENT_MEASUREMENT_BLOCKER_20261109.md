# 真实电流验证阻塞说明（2026-11-09）

当前软件/硬件链没有真实电流或母线电压测量：

- `/slaihand/dynamic_joint_states` 只有 position/velocity/effort/温度/状态；
- 驱动 C++ 中只有 `request_estimate_nm`，没有 bus_current/bus_voltage；
- 没有 INA219/INA226、分流器、功率分析仪等读取路径；
- `effort` 是估算，不是实测电流/扭矩。

因此本轮只能用 signed position error 和 `Kp*(reference-feedback)` 估算保持力矩，
**不能**称为真实电流验证，也不能据此做 1.5× 裕量或真实负载能力结论。

继续前需要：

1. 外接分流器/功率分析仪或增加总线电压、电流采样；
2. 核对单位、量程、标定、采样率和有效性；
3. 与位置/速度/effort 同时间戳记录；
4. 在真实电流和 allowed output/demand 证据齐备后，再评估 1.5×。
