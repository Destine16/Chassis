#include "srv_arbiter.h"

#include <string.h>

#include "bsp_time.h"
#include "srv_rc.h"
#include "srv_watchdog.h"
#include "state_machine.h"

typedef struct
{
    state_machine_t mode_sm;
    arbiter_state_t state;
    chassis_cmd_t final_cmd;
} srv_arbiter_context_t;

static srv_arbiter_context_t g_arbiter_ctx;

void srv_arbiter_init(void)
{
    memset(&g_arbiter_ctx, 0, sizeof(g_arbiter_ctx));
    state_machine_init(&g_arbiter_ctx.mode_sm, (uint32_t)APP_MODE_SAFE, bsp_time_get_ms());
    g_arbiter_ctx.state.requested_mode = APP_MODE_SAFE;
    g_arbiter_ctx.state.resolved_mode = APP_MODE_SAFE;
}

void srv_arbiter_step(void)
{
    rc_state_t rc = srv_rc_get_state();
    chassis_cmd_t final_cmd = {0};
    app_mode_t requested = srv_rc_get_requested_mode();
    app_mode_t resolved = APP_MODE_SAFE;
    uint32_t now_ms = bsp_time_get_ms();
    uint32_t flags = 0U;
    uint32_t watchdog_flags = srv_watchdog_get_flags();

    if (!rc.online)
    {
        /* 遥控器离线时直接压到 SAFE，优先级最高。 */
        flags |= SRV_ARBITER_FLAG_RC_OFFLINE;
        requested = APP_MODE_SAFE;
    }
    if ((watchdog_flags & (SRV_WATCHDOG_FLAG_MOTOR1_OFFLINE |
                           SRV_WATCHDOG_FLAG_MOTOR2_OFFLINE |
                           SRV_WATCHDOG_FLAG_MOTOR3_OFFLINE |
                           SRV_WATCHDOG_FLAG_MOTOR4_OFFLINE)) != 0U)
    {
        /* 任意底盘电机离线都直接压到 SAFE，避免残余三轮继续输出。 */
        requested = APP_MODE_SAFE;
    }

    if (requested == APP_MODE_SAFE)
    {
        resolved = APP_MODE_SAFE;
    }
    else if (requested == APP_MODE_MANUAL)
    {
        /* MANUAL 模式直接采用遥控器命令。 */
        resolved = APP_MODE_MANUAL;
        final_cmd = srv_rc_get_manual_cmd();
    }
    else
    {
        /* 当前底盘移动只由遥控器给定，非 SAFE 状态统一落到 MANUAL。 */
        resolved = APP_MODE_MANUAL;
        final_cmd = srv_rc_get_manual_cmd();
    }

    /* 状态机仅记录模式切换时刻，具体行为仍由仲裁逻辑决定。 */
    if (state_machine_transition(&g_arbiter_ctx.mode_sm, (uint32_t)resolved, now_ms))
        g_arbiter_ctx.state.last_transition_ms = now_ms;

    g_arbiter_ctx.state.requested_mode = requested;
    g_arbiter_ctx.state.resolved_mode = resolved;
    g_arbiter_ctx.state.flags = flags;
    g_arbiter_ctx.final_cmd = final_cmd;
}

arbiter_state_t srv_arbiter_get_state(void)
{
    return g_arbiter_ctx.state;
}

chassis_cmd_t srv_arbiter_get_final_cmd(void)
{
    return g_arbiter_ctx.final_cmd;
}
