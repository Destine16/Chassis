#ifndef USER_CONTROL_CTRL_CHASSIS_H
#define USER_CONTROL_CTRL_CHASSIS_H

#include "common_def.h"

/*
 * 四个底盘轮共用一套速度环参数。
 * 调试时可直接在调试器里修改这个全局变量，下一控制周期自动生效。
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} ctrl_chassis_speed_pid_param_t;

extern ctrl_chassis_speed_pid_param_t g_ctrl_chassis_speed_pid_param;

/*
 * 轮速前馈参数：
 * u_ff = kS * sign(speed_ref) + kV * speed_ref
 *
 * enable 仅控制是否把前馈叠加进速度环，不影响参数本身。
 */
typedef struct
{
    uint32_t enable;
    float k_s;
    float k_v;
} ctrl_chassis_speed_ff_param_t;

extern ctrl_chassis_speed_ff_param_t g_ctrl_chassis_speed_ff_param;

/*
 * 遥控回中主动刹车参数。
 * k_brake 默认可通过调试器在线修改；单位为 C620 电流给定原始量 / (rad/s)。
 */
typedef struct
{
    uint32_t enable;
    float k_brake;
    float current_limit;
    float enter_cmd_v_mps;
    float enter_cmd_wz_radps;
    float enter_wheel_speed_radps;
    float exit_wheel_speed_radps;
} ctrl_chassis_brake_param_t;

typedef struct
{
    uint32_t active;
    float k_brake;
    float max_abs_wheel_speed_radps;
    float brake_current_cmd[COMMON_WHEEL_COUNT];
} ctrl_chassis_brake_state_t;

extern ctrl_chassis_brake_param_t g_ctrl_chassis_brake_param;

/*
 * 当前底盘按“电机输出轴朝外、四轮对称安装”写死每轮方向。
 * 返回值用于把抽象轮速方向转换成实际电机正方向。
 */
float ctrl_chassis_motor_sign(chassis_wheel_id_t wheel_id);

/* 底盘控制核心：final_cmd -> 轮速参考 -> 速度 PID -> 电流给定 -> CAN 发送。 */
void ctrl_chassis_init(void);
void ctrl_chassis_execute(app_mode_t mode, const chassis_cmd_t *final_cmd);
void ctrl_chassis_execute_single_wheel_speed(chassis_wheel_id_t wheel_id, float speed_ref_radps);
void ctrl_chassis_stop(void);
ctrl_chassis_brake_state_t ctrl_chassis_get_brake_state(void);

#endif /* USER_CONTROL_CTRL_CHASSIS_H */
