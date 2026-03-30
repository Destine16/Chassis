#include "srv_nav.h"

#include <string.h>

#include "robot_def.h"
#include "bsp_time.h"

static nav_state_t g_nav_state;

void srv_nav_init(void)
{
    memset(&g_nav_state, 0, sizeof(g_nav_state));
}

void srv_nav_update_from_driver(const drv_nav_cmd_t *cmd)
{
    /* 驱动层仍用整数单位，服务层统一换算成控制侧浮点单位。 */
    g_nav_state.vx_mps = (float)cmd->vx_mmps / 1000.0f;
    g_nav_state.vy_mps = (float)cmd->vy_mmps / 1000.0f;
    g_nav_state.wz_radps = (float)cmd->wz_mradps / 1000.0f;
    g_nav_state.seq = cmd->seq;
    g_nav_state.valid = true;
    g_nav_state.online = true;
    g_nav_state.last_update_ms = bsp_time_get_ms();
}

void srv_nav_refresh_online(uint32_t now_ms)
{
    /* 第一版超时只拉低 online，valid 仍保留给上层做降级判断。 */
    if (bsp_time_is_expired(now_ms, g_nav_state.last_update_ms, APP_CFG_NAV_TIMEOUT_MS))
        g_nav_state.online = false;
}

nav_state_t srv_nav_get_state(void)
{
    return g_nav_state;
}
