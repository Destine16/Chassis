#ifndef USER_SERVICE_SRV_DEBUG_H
#define USER_SERVICE_SRV_DEBUG_H

#include "common_def.h"
#include "ctrl_chassis.h"
#include "drv_can_motor.h"
#include "drv_dbus.h"
#include "drv_nav_proto.h"
#include "sysid.h"

typedef struct
{
    uint32_t t_ms;
    rc_state_t rc;
    arbiter_state_t arbiter;
    uint32_t watchdog_flags;
    chassis_cmd_t final_cmd;
    chassis_cmd_t limited_cmd;
    chassis_cmd_t actual_cmd;
    wheel_targets_t wheel_targets;
    motor_feedback_t motors[COMMON_WHEEL_COUNT];
    ctrl_chassis_speed_pid_param_t pid_param;
    drv_dbus_stats_t dbus;
    drv_can_motor_stats_t can;
    drv_nav_proto_stats_t nav_tx;
    sysid_meta_t sysid_meta;
    uint32_t sysid_running;
    uint32_t sysid_hold_zero;
    uint32_t wheeltest_active;
    uint32_t wheeltest_hold_zero;
    uint32_t wheeltest_wheel_id;
    float wheeltest_speed_ref_radps;
} debug_snapshot_t;

extern debug_snapshot_t g_debug_snapshot;

void srv_debug_init(void);
void srv_debug_refresh(void);
debug_snapshot_t srv_debug_get_snapshot(void);

#endif /* USER_SERVICE_SRV_DEBUG_H */
