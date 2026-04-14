# 控制模块

## 车体系定义

- `+vx = 前`
- `+vy = 左`
- `+wz = 逆时针`

## 遥控器映射

- 左摇杆前后：`ch3 -> vx`
- 左摇杆左右：`ch2 -> vy`
- 右摇杆左右：`ch0 -> wz`
- 右摇杆前后：当前未使用

模式拨杆：
- `S1` 上：`SAFE`
- `S1` 中/下：`MANUAL`

## 当前控制链

```text
命令斜坡
-> 车体速度限幅
-> 四轮逆运动学
-> 轮速同比缩放
-> 逐轮速度前馈 + 速度 PID
-> 电流输出
```

## 电机方向修正

当前按实物安装写死：
- `front_left = -1`
- `front_right = +1`
- `rear_left = -1`
- `rear_right = +1`

这层用于把抽象轮速方向转换成实际电机正方向。

## 速度环

四轮共用一套速度 PID 参数：
- `g_ctrl_chassis_speed_pid_param.kp`
- `g_ctrl_chassis_speed_pid_param.ki`
- `g_ctrl_chassis_speed_pid_param.kd`

运行中可直接通过调试器在线修改。

## 前馈

当前已接入：

```text
u = u_ff + u_pid
u_ff = kS * sign(w_ref) + kV * w_ref
```

当前默认参数：
- `enable = 1`
- `kS = 425.378`
- `kV = 1.14785`

可在线修改：
- `g_ctrl_chassis_speed_ff_param.enable`
- `g_ctrl_chassis_speed_ff_param.k_s`
- `g_ctrl_chassis_speed_ff_param.k_v`

说明：
- 前馈在抽象轮速方向域计算
- 电机方向修正在最终发电流前统一处理
