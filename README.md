# 底盘工程

本项目是一个基于 `STM32F405 + M3508 + DR16/DBUS + CAN1 + USB CDC + BMI088` 的四轮全向底盘控制工程。

当前系统边界已经收敛为：
- 底盘移动只由遥控器控制
- 电控通过 `USB CDC` 向导航发送轮速惯性里程计观测

## 当前硬件

- MCU：`STM32F405`
- 底盘电机：4 个 `M3508 + hexroll`
- 总减速比：`268:17`
- 轮半径：`81.5 mm`
- 轮对角线距离：`425 mm`
- 四个轮电机都在 `CAN1`
- ESC ID：`1~4`
- 轮位与 ESC ID 对应：
  - `front_left = 2`
  - `front_right = 1`
  - `rear_left = 3`
  - `rear_right = 4`
- 遥控器：`USART3_RX(PB11)`，`100000 / 9bit / Even`
- IMU：板载 `BMI088`
  - `SPI1 = PA5 / PA6 / PA7`
  - `ACC_CS = PC4`
  - `GYRO_CS = PB1`
  - `ACC_INT = PB0`
  - `GYRO_INT = PC5`
- 导航链路：`USB OTG FS CDC`

## 当前架构

主链路是：

```text
DR16/DBUS
-> RC 服务层
-> SAFE / MANUAL 仲裁
-> 底盘控制
-> 逆运动学 / 轮速同比缩放 / 速度 PID
-> CAN1 -> C620 -> M3508
```

同时，观测链路是：

```text
电机反馈 + BMI088
-> telemetryTask
-> USB CDC
-> 导航
```

## 主要功能

- 四轮 `X` 布局全向底盘控制
- `SAFE / MANUAL` 两态仲裁
- DR16/DBUS 遥控器输入
- `USART3 + DMA + IDLE + 双缓冲` 接收
- 四轮逆运动学解算
- 四轮正运动学反算
- 逆解后的轮速同比缩放
- 四轮共用一套速度 PID 参数
- `CAN1 0x200` 下发四个 `C620` 电流命令
- 电机离线直接压到 `SAFE`
- USB CDC 输出轮速 + IMU 原始观测
- USB CDC 发送链使用固定帧 FIFO，发送忙时不再直接丢最新帧
- BMI088 运行期 `EXTI + SPI DMA` 非阻塞采样
- 单轮 `PRBS` 系统辨识模式

## 目录结构

```text
application/
  robot.c
  robot_task.c
  cmd/srv_rc.c
  cmd/srv_arbiter.c
  chassis/ctrl_chassis.c
  chassis/omni_chassis_kinematics.c
  robot_def.h

bsp/
  can/
  usart/
  time/

modules/
  algorithm/
  daemon/
  imu/                           BMI088 最小驱动
  master_machine/                USB CDC 观测帧协议
  motor/
  remote/
```

## 控制策略

- 遥控器输入采用“只取最新值”，不堆积历史命令
- 当前没有导航接管底盘的 `AUTO` 模式
- `MANUAL` 模式摇杆映射：
  - `ch3 -> vx`
  - `ch2 -> vy`
  - `ch0 -> wz`
- 底盘执行顺序：
  - 命令斜坡
  - 车体速度限幅
  - 四轮逆运动学
  - 轮速同比缩放
  - 逐轮速度 PID
  - 电流输出

当前运动学按 `45°` 四全向轮模型处理：
- 轮对角线 `425 mm`
- 底盘中心到轮心距离 `212.5 mm`
- `vx / vy` 使用 `sin/cos(45°)` 投影

## BMI088 说明

当前 IMU 驱动只保留底盘需要的最小功能：
- 三轴角速度
- 三轴加速度
- 上电陀螺零偏标定
- 运行期 `EXTI + SPI DMA` 采样

上电后会先做一轮简单陀螺零偏标定：
- 累计 `300` 个 gyro 样本做均值
- 大约需要静止 `1.3 s`
- 标定完成前，IMU 会先标成未就绪
- 标定完成后，输出的是“减过零偏”的角速度

上电注意事项：
- 上电后先不要动底盘
- 等 `1 s` 多一点再开始转动和测试

## 单轮系统辨识

当前工程已经接入一套最小单轮 `PRBS` 辨识模式，目标是辨识：

```text
u_raw -> wheel_speed_radps
```

也就是直接给单个轮子施加原始电流命令，再记录该轮的实际输出轴角速度。

当前设计约束：
- 一次只辨识一个轮子
- 建议轮子架空
- 建议遥控器保持 `SAFE`，避免实验结束后恢复正常控制
- 辨识数据先写入板上 RAM，实验结束后再通过 `gdb` dump

当前默认 `PRBS` 参数：
- 幅值：`+2500 / -2500`
- bit 时间：`40 ms`
- bit 数：`127`
- 总时长：约 `5.08 s`

采样缓冲区：
- 放在 `CCMRAM`
- 每条样本保存：
  - `u_raw`
  - `speed_radps`
- 采样率固定为 `1000 Hz`
- 当前容量：`6000` 点
  - 对应最长约 `6.0 s`
  - 足够覆盖当前默认 `5.08 s` 的一次实验

调试器可直接修改的全局变量：
- `g_sysid_arm`
- `g_sysid_running`
- `g_sysid_hold_zero`
- `g_sysid_wheel_id`
- `g_sysid_amplitude_raw`
- `g_sysid_bit_period_ms`
- `g_sysid_total_bits`

其中：
- `g_sysid_arm = 1` 会启动一次实验
- `g_sysid_running = 1` 表示实验正在进行
- `g_sysid_hold_zero = 1` 表示实验结束后继续保持四轮零输出
  - 需要恢复正常底盘控制时，手动把它清成 `0`
- `g_sysid_wheel_id` 使用当前统一轮序：
  - `0 = front_left`
  - `1 = front_right`
  - `2 = rear_left`
  - `3 = rear_right`

实验结果保存在：
- `g_sysid_meta`
- `g_sysid_buffer`

`g_sysid_meta` 会记录：
- `sample_hz`
- `sample_count`
- `capacity`
- `wheel_id`
- `amplitude_raw`
- `bit_period_ms`
- `total_bits`

离线导出时：
- 不单独存每条样本时间戳
- 直接按 `sample_hz = 1000` 还原时间轴
- 即 `t = sample_idx / 1000.0`
- 当前 `u_raw` 记录的是“上一拍已经实际发出的电流命令”
  - 这样能和同一条样本里的 `speed_radps` 更好对齐

详细的 `gdb` 操作步骤见：
- [docs/sysid_gdb.md](docs/sysid_gdb.md)

离线解析脚本见：
- [tools/sysid_dump_to_csv.py](tools/sysid_dump_to_csv.py)

## 导航观测帧协议

当前电控通过 `USB CDC` 周期发送一类固定长度观测帧给导航。
当前默认发送频率为 `200 Hz`。

帧格式：

```text
SOF0    1B   0xA5
SOF1    1B   0x5A
SEQ     1B   递增序号
LEN     1B   固定 46
PAYLOAD 46B
CRC16   2B
```

总长度固定为 `52` 字节。

`PAYLOAD` 字段如下，全部为小端：

```text
t_ms      u32   时间戳(ms)
w_fl      f32   左前轮实际角速度(rad/s)
w_fr      f32   右前轮实际角速度(rad/s)
w_rl      f32   左后轮实际角速度(rad/s)
w_rr      f32   右后轮实际角速度(rad/s)
gyro_x    f32   IMU角速度x(rad/s)
gyro_y    f32   IMU角速度y(rad/s)
gyro_z    f32   IMU角速度z(rad/s)
acc_x     f32   IMU加速度x(m/s^2)
acc_y     f32   IMU加速度y(m/s^2)
acc_z     f32   IMU加速度z(m/s^2)
flags     u16   状态位
```

`flags` 当前定义：
- `bit0`：`imu_valid`
- `bit1`：`wheel_fl_valid`
- `bit2`：`wheel_fr_valid`
- `bit3`：`wheel_rl_valid`
- `bit4`：`wheel_rr_valid`
- `bit5`：`imu_fault`
- `bit6`：`motor_fault`
- `bit7`：`chassis_safe_mode`
- `bit8`：`rc_offline`

## 当前待定事项

当前工程已经能形成完整的底盘控制与观测上报主链，但还有几项后续建议收尾：

- `BMI088` 轴向映射 / 符号修正
  - 目前直接发送芯片原始 `xyz`
  - 后续需要结合实车坐标系确认 `gyro/acc` 的轴向和正负号
- 导航侧联调
  - 确认 `USB CDC` 帧格式解析正确
  - 确认轮序固定为 `FL / FR / RL / RR`
- 实车验证
  - 验证 `vx / vy / wz` 三个单动作方向
  - 验证 BMI088 上电零偏标定完成后的 `gyro_z` 方向
- PID 实车整定
  - 当前已经支持在线改共享速度 PID 参数
  - 还需要结合实车继续细调 `kp / kd / ki`
- 可选增强
  - 如果后续需要更强的链路诊断，可再补 `USB CDC` 丢包统计和 IMU/电机观测统计

## 编译

```bash
cmake -S . -B build/Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Debug -j
```

生成文件：

```text
build/Debug/Chassis.elf
```

## 上板前需要确认

- 四个底盘电机的 ID 与轮序是否一致
- `vx / vy / wz` 正方向是否与实车一致
- `BMI088` 三轴方向是否与定义的车体坐标系一致
- 轮速和 IMU 数据在导航侧的解析是否与当前 `USB CDC` 帧格式一致
- 共用速度 PID 参数与电流限幅是否已按实车调过
