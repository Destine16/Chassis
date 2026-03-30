#include "srv_motor.h"

#include <string.h>

#include "robot_def.h"
#include "bsp_time.h"
#include "drv_can_motor.h"

typedef struct
{
    motor_feedback_t feedback[COMMON_WHEEL_COUNT];
    wheel_targets_t targets;
} srv_motor_context_t;

static srv_motor_context_t g_motor_ctx;

void srv_motor_init(void)
{
    memset(&g_motor_ctx, 0, sizeof(g_motor_ctx));
}

void srv_motor_poll_driver_feedback(void)
{
    drv_m3508_feedback_t raw_feedback[COMMON_WHEEL_COUNT];
    uint32_t mask = drv_can_motor_fetch_pending(raw_feedback);
    uint32_t index = 0U;

    while (index < COMMON_WHEEL_COUNT)
    {
        if ((mask & (1UL << index)) != 0U)
        {
            /* 只更新本次收到反馈的轮子，其余轮子维持上一拍状态。 */
            g_motor_ctx.feedback[index].ecd = raw_feedback[index].ecd;
            g_motor_ctx.feedback[index].rpm = raw_feedback[index].rpm;
            g_motor_ctx.feedback[index].current = raw_feedback[index].current;
            g_motor_ctx.feedback[index].temperature = raw_feedback[index].temperature;
            g_motor_ctx.feedback[index].wheel_speed_radps = raw_feedback[index].wheel_speed_radps;
            g_motor_ctx.feedback[index].online = true;
            g_motor_ctx.feedback[index].last_update_ms = bsp_time_get_ms();
        }
        index++;
    }
}

void srv_motor_refresh_online(uint32_t now_ms)
{
    uint32_t index = 0U;

    while (index < COMMON_WHEEL_COUNT)
    {
        /* 电机反馈超时后只标记离线，不在这里清空反馈值。 */
        if ((now_ms - g_motor_ctx.feedback[index].last_update_ms) > APP_CFG_MOTOR_TIMEOUT_MS)
            g_motor_ctx.feedback[index].online = false;
        index++;
    }
}

motor_feedback_t srv_motor_get_feedback(chassis_wheel_id_t wheel_id)
{
    return g_motor_ctx.feedback[wheel_id];
}

void srv_motor_get_feedback_all(motor_feedback_t feedback_out[COMMON_WHEEL_COUNT])
{
    memcpy(feedback_out, g_motor_ctx.feedback, sizeof(g_motor_ctx.feedback));
}

void srv_motor_set_targets(const float speed_ref[COMMON_WHEEL_COUNT], const float current_cmd[COMMON_WHEEL_COUNT])
{
    uint32_t index = 0U;

    /* 保存目标值主要用于遥测和调试，不参与反馈解析。 */
    while (index < COMMON_WHEEL_COUNT)
    {
        g_motor_ctx.targets.wheel_speed_ref_radps[index] = speed_ref[index];
        g_motor_ctx.targets.current_cmd[index] = current_cmd[index];
        index++;
    }
}

void srv_motor_get_targets(wheel_targets_t *targets_out)
{
    *targets_out = g_motor_ctx.targets;
}

void srv_motor_zero_targets(void)
{
    memset(&g_motor_ctx.targets, 0, sizeof(g_motor_ctx.targets));
}
