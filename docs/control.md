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
-> 遥控回中主动刹车
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

## 回中主动刹车

遥控器回中后，如果车体命令已经接近 0，但轮子仍在转，控制器会进入轮级阻尼刹车：

```text
brake_current = -K_brake * wheel_speed
```

当前默认参数：
- `enable = 1`
- `K_brake = 1030`
- `current_limit = 4000`
- `enter_cmd_v = 0.03 m/s`
- `enter_cmd_wz = 0.06 rad/s`
- `enter_wheel_speed = 0.50 rad/s`
- `exit_wheel_speed = 0.25 rad/s`

可在线修改：
- `g_ctrl_chassis_brake_param.enable`
- `g_ctrl_chassis_brake_param.k_brake`
- `g_ctrl_chassis_brake_param.current_limit`

说明：
- 主动刹车只在遥控命令接近 0 时介入
- 介入时会重置三轴命令斜坡和四轮速度 PID，避免零速附近残留积分继续推轮子
- 刹车电流仍在抽象轮速方向域计算，最终发电流前统一做电机方向修正
- 当前 `K_brake` 来自 forward 落地制动实验；旋转和横移可先共用，后续如发现手感差异再单独补测
