#include "srv_debug.h"

#include <string.h>

#include "bsp_time.h"
#include "srv_arbiter.h"
#include "srv_chassis.h"
#include "srv_motor.h"
#include "srv_rc.h"
#include "srv_watchdog.h"

debug_snapshot_t g_debug_snapshot;

void srv_debug_init(void)
{
    memset(&g_debug_snapshot, 0, sizeof(g_debug_snapshot));
}

void srv_debug_refresh(void)
{
    g_debug_snapshot.t_ms = bsp_time_get_ms();
    g_debug_snapshot.rc = srv_rc_get_state();
    g_debug_snapshot.arbiter = srv_arbiter_get_state();
    g_debug_snapshot.watchdog_flags = srv_watchdog_get_flags();
    g_debug_snapshot.final_cmd = srv_chassis_get_final_cmd();
    g_debug_snapshot.limited_cmd = srv_chassis_get_limited_cmd();
    g_debug_snapshot.actual_cmd = srv_chassis_get_actual_cmd();
    g_debug_snapshot.wheel_targets = srv_chassis_get_wheel_targets();
    srv_motor_get_feedback_all(g_debug_snapshot.motors);
    g_debug_snapshot.pid_param = g_ctrl_chassis_speed_pid_param;
    g_debug_snapshot.ff_param = g_ctrl_chassis_speed_ff_param;
    drv_dbus_get_stats(&g_debug_snapshot.dbus);
    drv_can_motor_get_stats(&g_debug_snapshot.can);
    drv_nav_proto_get_stats(&g_debug_snapshot.nav_tx);
    g_debug_snapshot.sysid_meta = g_sysid_meta;
    g_debug_snapshot.ffid_meta = g_ffid_meta;
    g_debug_snapshot.sysid_running = g_sysid_running;
    g_debug_snapshot.sysid_hold_zero = g_sysid_hold_zero;
    g_debug_snapshot.ffid_running = g_ffid_running;
    g_debug_snapshot.ffid_hold_zero = g_ffid_hold_zero;
    g_debug_snapshot.wheeltest_active = g_wheeltest_active;
    g_debug_snapshot.wheeltest_hold_zero = g_wheeltest_hold_zero;
    g_debug_snapshot.wheeltest_wheel_id = g_wheeltest_wheel_id;
    g_debug_snapshot.wheeltest_speed_ref_radps = g_wheeltest_speed_ref_radps;
}

debug_snapshot_t srv_debug_get_snapshot(void)
{
    return g_debug_snapshot;
}
