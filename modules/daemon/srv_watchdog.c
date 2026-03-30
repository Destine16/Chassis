#include "srv_watchdog.h"

#include "bsp_time.h"
#include "srv_motor.h"
#include "srv_rc.h"

static uint32_t g_watchdog_flags = 0U;

void srv_watchdog_init(void)
{
    g_watchdog_flags = 0U;
}

void srv_watchdog_update(void)
{
    motor_feedback_t motors[COMMON_WHEEL_COUNT];
    rc_state_t rc;
    uint32_t now_ms = bsp_time_get_ms();

    /* 先刷新各模块 online 标志，再汇总成统一故障位。 */
    srv_rc_refresh_online(now_ms);
    srv_motor_refresh_online(now_ms);

    rc = srv_rc_get_state();
    srv_motor_get_feedback_all(motors);

    g_watchdog_flags = 0U;

    /* 故障位按固定顺序写入，便于上位机直接解码。 */
    if (!rc.online)
        g_watchdog_flags |= SRV_WATCHDOG_FLAG_RC_OFFLINE;
    if (!motors[0].online)
        g_watchdog_flags |= SRV_WATCHDOG_FLAG_MOTOR1_OFFLINE;
    if (!motors[1].online)
        g_watchdog_flags |= SRV_WATCHDOG_FLAG_MOTOR2_OFFLINE;
    if (!motors[2].online)
        g_watchdog_flags |= SRV_WATCHDOG_FLAG_MOTOR3_OFFLINE;
    if (!motors[3].online)
        g_watchdog_flags |= SRV_WATCHDOG_FLAG_MOTOR4_OFFLINE;
}

uint32_t srv_watchdog_get_flags(void)
{
    return g_watchdog_flags;
}
