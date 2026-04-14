#include "sysid_internal.h"

#include "ctrl_chassis.h"
#include "srv_motor.h"

void wheeltest_begin(void)
{
    g_wheeltest_rt.active = true;
    g_wheeltest_rt.wheel_id = sysid_clamp_wheel_id(g_wheeltest_wheel_id);
    g_wheeltest_active = 1U;
    g_wheeltest_hold_zero = 0U;
    ctrl_chassis_stop();
}

bool wheeltest_task_step(void)
{
    motor_feedback_t feedback[COMMON_WHEEL_COUNT];

    if ((!g_wheeltest_rt.active) && (g_wheeltest_hold_zero != 0U))
    {
        ctrl_chassis_stop();
        return true;
    }

    if (!g_wheeltest_rt.active)
        return false;

    srv_motor_get_feedback_all(feedback);

    if (!feedback[g_wheeltest_rt.wheel_id].online)
    {
        wheeltest_finish_and_zero_output();
        return true;
    }

    if (g_wheeltest_enable == 0U)
    {
        wheeltest_finish_and_zero_output();
        return true;
    }

    ctrl_chassis_execute_single_wheel_speed((chassis_wheel_id_t)g_wheeltest_rt.wheel_id,
                                            g_wheeltest_speed_ref_radps);
    return true;
}
