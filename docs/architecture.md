# 架构概览

## 系统边界

- 底盘移动只由遥控器控制
- 电控通过 `USB CDC` 向导航发送轮速惯导观测
- 当前没有导航接管底盘的 `AUTO` 模式

## 主控制链

```text
DR16/DBUS
-> RC 服务层
-> SAFE / MANUAL 仲裁
-> 底盘控制
-> 逆运动学 / 轮速同比缩放 / 速度前馈 + 速度 PID / 回中主动刹车
-> CAN1 -> C620 -> M3508
```

## 观测链

```text
电机反馈 + BMI088
-> telemetryTask
-> USB CDC
-> 导航
```

## 主要目录

```text
application/
  cmd/         RC / 仲裁
  chassis/     底盘控制、运动学、辨识
  debug/       统一调试快照
  robot*.c     初始化与任务调度

modules/
  imu/         BMI088 驱动
  motor/       CAN 电机驱动与反馈
  remote/      DBUS 接收
  master_machine/ USB CDC 协议与发送
  algorithm/   PID / 数学基础
```

## 当前关键设计

- RC 接收：`USART3 IRQ -> byte ring buffer -> task 侧流式拼 DBUS 帧`
- 控制坐标系：`+vx=前`、`+vy=左`、`+wz=逆时针`
- IMU 输出：电控侧已映射成车体系
- 轮速输出：已做电机安装方向修正，导航侧不再额外反向
- 回中主动刹车：轮级阻尼形式，当前默认 `K_brake=1030`

## 当前大文件拆分

`application/chassis/sysid.c` 已按职责拆成：
- `sysid.c`：模式调度入口
- `sysid_shared.c`：共享状态、全局变量、通用 helper
- `sysid_prbs.c`：PRBS 单轮辨识
- `sysid_ffid.c`：稳态速度平台前馈辨识
- `sysid_wheeltest.c`：单轮闭环测试
