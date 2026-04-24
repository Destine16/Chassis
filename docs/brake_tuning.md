# 回中主动刹车整定

## 目标

遥控器回中后，让底盘更快、更稳地停下。

当前实现是轮级阻尼刹车：

```text
brake_current = -K_brake * wheel_speed
```

它只在车体命令接近 0、但轮子仍在转时介入。

## 固件参数

默认参数位于：

```text
application/robot_def.h
```

当前默认值：

```text
APP_CFG_BRAKE_K_DEFAULT = 1030.0
APP_CFG_BRAKE_CURRENT_LIMIT = 4000.0
APP_CFG_BRAKE_ENTER_CMD_V_MPS = 0.03
APP_CFG_BRAKE_ENTER_CMD_WZ_RADPS = 0.06
APP_CFG_BRAKE_ENTER_WHEEL_RADPS = 0.50
APP_CFG_BRAKE_EXIT_WHEEL_RADPS = 0.25
```

运行中可通过 GDB 临时修改：

```gdb
set var g_ctrl_chassis_brake_param.k_brake = 1000.0f
set var g_ctrl_chassis_brake_param.enable = 1
```

## 采集流程

当前实验只测 forward 直行制动：

```text
K_brake = 0, 200, 400, 600, 800, 1000, 1200
每个 K 测 5 次 forward
```

每组动作：

```text
左摇杆向前推，保持 1~2 秒
快速松手回中
等待底盘完全停稳
重复 5 次
Ctrl+C 结束采集
```

采集命令示例：

```bash
.venv/bin/python tools/brake_capture.py -p /dev/ttyACM0 --k-brake 1000 --direction forward
```

脚本行为：
- 按 `Enter` 开始采集
- 按 `Ctrl+C` 结束并保存 CSV
- 每个 CSV 会记录 `k_brake`、方向、四轮轮速、IMU、flags、反解车体速度

## 分析命令

离散点分析：

```bash
.venv/bin/python tools/brake_analyze.py data/brake/*.csv -o data/brake/analysis_forward_k0_1200
```

连续优化：

```bash
.venv/bin/python tools/brake_optimize.py data/brake/*.csv \
  -o data/brake/optimize_forward_k0_1200_strict005 \
  --max-yaw-equiv-ratio 0.05
```

`--max-yaw-equiv-ratio 0.05` 用于剔除旋转占比偏大的片段。

## 当前结果

离散点综合评分结果：

```text
K=1000  score=1.429  stop_time=0.482s  stop_distance=0.174m
K=1200  score=1.545  stop_time=0.540s  stop_distance=0.203m
K=800   score=1.588  stop_time=0.537s  stop_distance=0.226m
```

连续优化结果：

```text
非严格筛选拟合推荐约 K = 1029
严格旋转筛选拟合推荐约 K = 1189
```

工程取值：

```text
当前固件默认 K_brake = 1030
```

这个值接近非严格筛选的连续拟合结果；如果实车验证时觉得刹车过硬，可先回退到 `1000`。

## 输出文件

原始采集：

```text
data/brake/brake_k*_forward_*.csv
```

离散分析：

```text
data/brake/analysis_forward_k0_1200/summary.json
data/brake/analysis_forward_k0_1200/brake_tuning_summary.png
```

连续优化：

```text
data/brake/optimize_forward_k0_1200_strict005/optimization_summary.json
data/brake/optimize_forward_k0_1200_strict005/k_brake_optimization.png
```

## 注意事项

- 当前结论来自 forward 直行制动实验。
- 旋转、横移可以先共用同一个 `K_brake`。
- 如果后续发现旋转回中拖滞或回弹，再补测旋转制动。
- 搬车或调整位置前应结束采集，避免把非落地制动数据混入 CSV。
