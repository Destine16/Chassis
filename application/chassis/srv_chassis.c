#include "srv_chassis.h"

#include <string.h>

typedef struct
{
    chassis_cmd_t final_cmd;
    chassis_cmd_t limited_cmd;
    chassis_cmd_t actual_cmd;
    wheel_targets_t wheel_targets;
} srv_chassis_context_t;

static srv_chassis_context_t g_chassis_ctx;

void srv_chassis_init(void)
{
    memset(&g_chassis_ctx, 0, sizeof(g_chassis_ctx));
}

void srv_chassis_set_final_cmd(const chassis_cmd_t *cmd)
{
    g_chassis_ctx.final_cmd = *cmd;
}

void srv_chassis_set_limited_cmd(const chassis_cmd_t *cmd)
{
    g_chassis_ctx.limited_cmd = *cmd;
}

void srv_chassis_set_actual_cmd(const chassis_cmd_t *cmd)
{
    g_chassis_ctx.actual_cmd = *cmd;
}

void srv_chassis_set_wheel_targets(const wheel_targets_t *targets)
{
    g_chassis_ctx.wheel_targets = *targets;
}

chassis_cmd_t srv_chassis_get_final_cmd(void)
{
    return g_chassis_ctx.final_cmd;
}

chassis_cmd_t srv_chassis_get_limited_cmd(void)
{
    return g_chassis_ctx.limited_cmd;
}

chassis_cmd_t srv_chassis_get_actual_cmd(void)
{
    return g_chassis_ctx.actual_cmd;
}

wheel_targets_t srv_chassis_get_wheel_targets(void)
{
    return g_chassis_ctx.wheel_targets;
}

void srv_chassis_stop(void)
{
    /* SAFE 模式下直接把底盘相关快照全部清零。 */
    memset(&g_chassis_ctx, 0, sizeof(g_chassis_ctx));
}
