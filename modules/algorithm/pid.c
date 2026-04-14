#include "pid.h"

#include <math.h>
#include <stdbool.h>

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
    pid->zero_ref_threshold = 0.0f;
    pid->zero_feedback_threshold = 0.0f;
    pid->integral_separation_error = 0.0f;
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
    float integral_candidate = pid->integral;
    float unsaturated_output;
    float output;
    bool zero_ref = fabsf(ref) <= pid->zero_ref_threshold;
    bool near_stop = fabsf(feedback) <= pid->zero_feedback_threshold;
    bool allow_integration = false;

    /* 微分项直接对误差差分，第一版先保持简单实现。 */
    if (dt_s > 0.0f)
        derivative = (error - pid->previous_error) / dt_s;

    /*
     * 零速区直接清积分，避免遥控回中后残余积分继续推轮子。
     * 若反馈也已经接近 0，同时把 previous_error 清掉，降低零速附近误差微分抖动。
     */
    if (zero_ref)
    {
        pid->integral = 0.0f;
        integral_candidate = 0.0f;
        if (near_stop)
            pid->previous_error = 0.0f;
    }
    else if ((dt_s > 0.0f) &&
             ((pid->integral_separation_error <= 0.0f) ||
              (fabsf(error) <= pid->integral_separation_error)))
    {
        /* 误差进入可控范围后才允许积分，避免大误差阶段盲目堆积分。 */
        integral_candidate += error * dt_s;
        integral_candidate = common_clampf(integral_candidate, -pid->integral_limit, pid->integral_limit);
        allow_integration = true;
    }

    unsaturated_output = (pid->kp * error) + (pid->ki * integral_candidate) + (pid->kd * derivative);
    output = common_clampf(unsaturated_output, -pid->output_limit, pid->output_limit);

    /*
     * 简单 anti-windup：
     * - 若输出未饱和，接受新的积分
     * - 若输出已饱和，但当前误差会把输出往“离饱和更远”的方向拉，接受积分
     * - 其余情况冻结积分，避免 windup
     */
    if (allow_integration)
    {
        bool saturated_high = (unsaturated_output > pid->output_limit);
        bool saturated_low = (unsaturated_output < -pid->output_limit);
        bool drives_further_into_saturation =
            (saturated_high && (error > 0.0f)) ||
            (saturated_low && (error < 0.0f));

        if (!drives_further_into_saturation)
            pid->integral = integral_candidate;
    }

    pid->previous_error = error;
    return output;
}
