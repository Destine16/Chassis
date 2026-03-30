#include "pid.h"

#include "common_math.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void pid_reset(pid_controller_t *pid)
{
    /* 模式切换到 SAFE 时应调用，防止积分残留。 */
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float pid_update(pid_controller_t *pid, float ref, float feedback, float dt_s)
{
    float error = ref - feedback;
    float derivative = 0.0f;
    float output;

    /* 微分项直接对误差差分，第一版先保持简单实现。 */
    if (dt_s > 0.0f)
        derivative = (error - pid->previous_error) / dt_s;

    /* 积分和输出都做对称限幅，避免长时间饱和。 */
    pid->integral += error * dt_s;
    pid->integral = common_clampf(pid->integral, -pid->integral_limit, pid->integral_limit);

    output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    output = common_clampf(output, -pid->output_limit, pid->output_limit);
    pid->previous_error = error;
    return output;
}
