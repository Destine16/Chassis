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
- 静止时 `gyro_x/y/z` 可能带有小 bias
- 长期 bias 更适合由导航侧估计

## 导航侧约定

- 不要再对 `gyro/acc` 做额外换轴或取反
- `acc_x/y/z` 仍包含重力项，不是去重力后的线加速度
