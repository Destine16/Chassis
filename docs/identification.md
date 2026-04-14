# 辨识与整定

## 1. PRBS 单轮辨识

用途：
- 辨识单轮 `u_raw -> wheel_speed_radps`

入口：
- `g_sysid_arm`
- `g_sysid_*`

详细步骤：
- [sysid_gdb.md](./sysid_gdb.md)
- [sysid_identify.md](./sysid_identify.md)

## 2. FFID 单轮前馈辨识

用途：
- 拟合 `u_ff = kS * sign(w_ref) + kV * w_ref`

入口：
- `g_ffid_arm`
- `g_ffid_*`

方式：
- 单轮闭环稳态速度平台
- 架空测试
- 记录维持稳态速度所需电流

离线脚本：
- [tools/ffid_dump_to_csv.py](../tools/ffid_dump_to_csv.py)
- [tools/ffid_identify.py](../tools/ffid_identify.py)

当前输出：
- `summary.json`
- `overview.png`
- `step_summary.png`
- `fit_scatter.png`
- `residuals.png`

## 3. PID 整定

工具：
- [pid_tune.md](./pid_tune.md)
- [tools/pid_tune.py](../tools/pid_tune.py)

说明：
- 当前工程已经能用 FFID 得到的 `kS/kV` 作为速度前馈
- PID 主要继续负责误差修正和扰动抑制
