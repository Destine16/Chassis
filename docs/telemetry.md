# USB CDC 观测帧

## 链路

- 发送方式：`USB CDC`
- 默认周期：`5 ms`
- 默认频率：`200 Hz`

## 帧格式

```text
A5 5A | seq | len | payload(46B) | crc16
```

总长度：
- `52 bytes`

说明：
- 小端序
- `len` 当前固定为 `46`

## Payload

```text
t_ms      u32   时间戳(ms)
w_fl      f32   左前轮角速度(rad/s)
w_fr      f32   右前轮角速度(rad/s)
w_rl      f32   左后轮角速度(rad/s)
w_rr      f32   右后轮角速度(rad/s)
gyro_x    f32   车体系角速度 x(rad/s)
gyro_y    f32   车体系角速度 y(rad/s)
gyro_z    f32   车体系角速度 z(rad/s)
acc_x     f32   车体系加速度 x(m/s^2)
acc_y     f32   车体系加速度 y(m/s^2)
acc_z     f32   车体系加速度 z(m/s^2)
flags     u16   状态位
```

## flags

当前定义：
- `bit0`：`imu_valid`
- `bit1`：`wheel_fl_valid`
- `bit2`：`wheel_fr_valid`
- `bit3`：`wheel_rl_valid`
- `bit4`：`wheel_rr_valid`
- `bit5`：`imu_fault`
- `bit6`：`motor_fault`
- `bit7`：`chassis_safe_mode`
- `bit8`：`rc_offline`

## CRC

当前实现：
- `CRC16-CCITT-FALSE`
- `poly = 0x1021`
- `init = 0xFFFF`
- `refin = false`
- `refout = false`
- `xorout = 0x0000`

覆盖范围：
- `A5 5A + seq + len + payload`

## 本机验证

脚本：
- [tools/nav_cdc_monitor.py](../tools/nav_cdc_monitor.py)

示例：

```bash
.venv/bin/python tools/nav_cdc_monitor.py -p /dev/ttyACM0
```
