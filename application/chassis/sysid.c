#include "robot_def.h"
#include "sysid.h"

#if (APP_CFG_ENABLE_EXPERIMENTS != 0U)

#include "sysid_internal.h"

bool sysid_task_step(void)
{
    if ((!g_ffid_rt.running) && (!g_sysid_rt.running) && (!g_wheeltest_rt.active) && (g_ffid_arm != 0U))
        ffid_begin();

    if ((!g_sysid_rt.running) && (!g_ffid_rt.running) && (!g_wheeltest_rt.active) && (g_sysid_arm != 0U))
        sysid_begin();

    if ((!g_wheeltest_rt.active) && (!g_ffid_rt.running) && (!g_sysid_rt.running) && (g_wheeltest_enable != 0U))
        wheeltest_begin();

    if (wheeltest_task_step())
        return true;

    if (ffid_task_step())
        return true;

    if (sysid_prbs_task_step())
        return true;

    return false;
}

#else

bool sysid_task_step(void)
{
    return false;
}

#endif
