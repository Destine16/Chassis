# 底盘工程

基于 `STM32F405 + M3508 + DR16/DBUS + CAN1 + USB CDC + BMI088` 的四轮全向底盘控制工程。

当前系统边界：
- 底盘移动只由遥控器控制
- 电控通过 `USB CDC` 向导航发送轮速惯导观测

## 快速概览

- 车体系：`+vx = 前`、`+vy = 左`、`+wz = 逆时针`
- 底盘：`45°` 四全向轮
- 轮半径：`81.5 mm`
- 轮对角线：`425 mm`
- 四轮 ESC ID：
  - `front_left = 2`
  - `front_right = 1`
  - `rear_left = 3`
  - `rear_right = 4`
- 遥控器：`USART3_RX(PB11)`，`100000 / 9bit / Even / 1 stop bit`
- 导航观测链：`USB CDC @ 200 Hz`

## 主链路

```text
DR16/DBUS
-> RC 服务层
-> SAFE / MANUAL 仲裁
-> 底盘控制
-> 逆运动学 / 轮速同比缩放 / 速度前馈 + 速度 PID
-> CAN1 -> C620 -> M3508
```

## 文档入口

- 架构：[docs/architecture.md](docs/architecture.md)
- 控制与遥控映射：[docs/control.md](docs/control.md)
- IMU 与车体系：[docs/imu.md](docs/imu.md)
- USB CDC 协议：[docs/telemetry.md](docs/telemetry.md)
- 辨识与整定：[docs/identification.md](docs/identification.md)

已有专项文档：
- [docs/sysid_gdb.md](docs/sysid_gdb.md)
- [docs/sysid_identify.md](docs/sysid_identify.md)
- [docs/pid_tune.md](docs/pid_tune.md)

## 关键脚本

- USB CDC 本机监视：
  - [tools/nav_cdc_monitor.py](tools/nav_cdc_monitor.py)
- PRBS dump 转 CSV：
  - [tools/sysid_dump_to_csv.py](tools/sysid_dump_to_csv.py)
- PRBS 模型辨识：
  - [tools/sysid_identify.py](tools/sysid_identify.py)
- FFID dump 转 CSV：
  - [tools/ffid_dump_to_csv.py](tools/ffid_dump_to_csv.py)
- FFID 前馈拟合：
  - [tools/ffid_identify.py](tools/ffid_identify.py)
- PID 整定：
  - [tools/pid_tune.py](tools/pid_tune.py)

分析脚本优先用项目内虚拟环境执行：

```bash
.venv/bin/python tools/...
```

## 编译

```bash
cmake -S . -B build/Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Debug -j
```

发布构建默认关闭实验代码（`PRBS / FFID / wheeltest`）。如需重新打开：

```bash
cmake -S . -B build/Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCHASSIS_ENABLE_EXPERIMENTS=ON
```

生成文件：

```text
build/Debug/Chassis.elf
build/Debug/Chassis.bin
```
