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

/* 底盘控制核心：final_cmd -> 轮速参考 -> 速度 PID -> 电流给定 -> CAN 发送。 */
void ctrl_chassis_init(void);
void ctrl_chassis_execute(app_mode_t mode, const chassis_cmd_t *final_cmd);
void ctrl_chassis_execute_single_wheel_speed(chassis_wheel_id_t wheel_id, float speed_ref_radps);
void ctrl_chassis_stop(void);

#endif /* USER_CONTROL_CTRL_CHASSIS_H */
