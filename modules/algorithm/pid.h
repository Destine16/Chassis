#ifndef USER_CONTROL_PID_H
#define USER_CONTROL_PID_H

/* 简单位置式 PID，当前用于轮速环。 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float integral_limit;
    float output_limit;
} pid_controller_t;

/* 初始化 PID 参数并清空内部状态。 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit);
void pid_reset(pid_controller_t *pid);

/* ref 和 feedback 单位需一致，输出单位由调用方自行定义。 */
float pid_update(pid_controller_t *pid, float ref, float feedback, float dt_s);

#endif /* USER_CONTROL_PID_H */
