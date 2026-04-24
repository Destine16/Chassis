# IMU 模块

## 硬件

- 传感器：`BMI088`
- 接口：`SPI1`
- 采样方式：运行期 `EXTI + SPI DMA`

## 输出语义

电控发出的 IMU 数据已经是底盘车体系，不是芯片 raw 轴。

车体系定义：
- `+x = 前`
- `+y = 左`
- `+z = 上`
- `+gyro_z = 逆时针`

## 当前轴映射

代码中的实际映射是：

```text
chassis_x = raw_y
chassis_y = -raw_x
chassis_z = raw_z
```

同一套映射同时作用于：
- `gyro_x/y/z`
- `acc_x/y/z`

## 当前行为

- 不再做上电陀螺零偏标定
- 拿到第一笔真实样本后即开始输出
- USB CDC 发送基础换轴后的未标定 IMU 数据
- 静止时 `gyro_x/y/z` 可能带有小 bias
- 长期 bias 更适合由导航侧估计

## 导航侧约定

- 不要再对 `gyro/acc` 做额外换轴或取反
- `acc_x/y/z` 仍包含重力项，不是去重力后的线加速度

## 静止水平标定

如果底盘静止放平时 `acc_x/acc_y` 仍有明显常值，可以用 USB CDC 采集静止数据，计算 roll/pitch 小角度修正矩阵：

```bash
.venv/bin/python tools/imu_static_level_calibrate.py -p /dev/ttyACM0
```

脚本输出的矩阵含义是：

```text
corrected_vec = rotation_matrix_chassis_level * current_chassis_vec
```

当前固件不会应用这个矩阵；USB CDC 仍发送基础换轴后的 `current_chassis_vec`。如果导航侧需要修正 roll/pitch 安装偏差，应在导航侧对收到的 `gyro` 和 `acc` 同时左乘这个矩阵。

这一步只修正 roll/pitch 安装偏差；yaw 外参仍应由导航侧结合轮速里程计或运动轨迹标定。

## yaw 外参标定

电控侧 USB CDC 帧已经同时包含四轮轮速和 IMU 数据，因此可以在本工程主机工具里先完成 yaw 外参拟合：

```bash
.venv/bin/python tools/imu_yaw_calibrate.py -p /dev/ttyACM0
```

默认流程是按 Enter 开始采集，动作做完后按 `Ctrl+C` 停止，脚本会自动分析并保存已采集数据。

实验动作建议：

- 静止 2~3 秒
- 前进加速、减速，重复几次
- 后退加速、减速，重复几次
- 左移和右移也各重复几次

脚本会用四轮轮速反算底盘 `vx/vy`，对速度求导得到轮速里程计加速度，再拟合：

```text
R_yaw * R_level * imu_from_elec ~= wheel_odom_acc
```

输出的 `rotation_matrix_total_yaw_level` 可由导航侧直接左乘到收到的 `gyro` 和 `acc`。
