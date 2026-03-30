#include "srv_rc.h"

#include <string.h>

#include "robot_def.h"
#include "bsp_time.h"
#include "common_math.h"

static rc_state_t g_rc_state;

/* 将 DBUS 拨杆原始编码转为可读枚举。 */
static rc_switch_pos_t srv_rc_decode_switch(uint8_t value)
{
    if (value == RC_SWITCH_UP)
        return RC_SWITCH_UP;
    if (value == RC_SWITCH_MID)
        return RC_SWITCH_MID;
    if (value == RC_SWITCH_DOWN)
        return RC_SWITCH_DOWN;
    return RC_SWITCH_UNKNOWN;
}

static float srv_rc_normalize_channel(uint16_t raw)
{
    float centered = (float)raw - (float)APP_CFG_RC_VALUE_CENTER;

    /* 先减中值、再做死区和归一化。 */
    centered = common_apply_deadband(centered, APP_CFG_RC_DEADBAND_RAW);
    return common_clampf(centered / APP_CFG_RC_VALUE_SPAN, -1.0f, 1.0f);
}

void srv_rc_init(void)
{
    memset(&g_rc_state, 0, sizeof(g_rc_state));
    g_rc_state.s1 = RC_SWITCH_UNKNOWN;
    g_rc_state.s2 = RC_SWITCH_UNKNOWN;
}

void srv_rc_update_from_dbus(const drv_dbus_frame_t *frame)
{
    uint32_t now_ms = bsp_time_get_ms();
    float activity;
    uint32_t index = 0U;

    while (index < 4U)
    {
        /* 同时保存原始值和归一化值，调试时更容易定位方向和中值问题。 */
        g_rc_state.ch_raw[index] = frame->ch[index];
        g_rc_state.ch_norm[index] = srv_rc_normalize_channel(frame->ch[index]);
        index++;
    }

    g_rc_state.s1 = srv_rc_decode_switch(frame->s1);
    g_rc_state.s2 = srv_rc_decode_switch(frame->s2);

    /* 键鼠数据先缓存；当前底盘模式里暂不使用这一路输入。 */
    g_rc_state.mouse.x = frame->mouse_x;
    g_rc_state.mouse.y = frame->mouse_y;
    g_rc_state.mouse.z = frame->mouse_z;
    g_rc_state.mouse.press_l = frame->mouse_press_l;
    g_rc_state.mouse.press_r = frame->mouse_press_r;
    g_rc_state.key_bits = frame->key_bits;
    g_rc_state.online = true;
    g_rc_state.last_update_ms = now_ms;

    activity = common_absf(g_rc_state.ch_norm[0]) +
               common_absf(g_rc_state.ch_norm[2]) +
               common_absf(g_rc_state.ch_norm[3]);

    /*
     * 人工接管状态仍然保留，便于后续扩展更高层模式；
     * 当前底盘逻辑不会再把它当成导航接管降级条件。
     */
    if (activity >= APP_CFG_RC_TAKEOVER_THRESHOLD)
    {
        if (g_rc_state.takeover_since_ms == 0U)
            g_rc_state.takeover_since_ms = now_ms;

        g_rc_state.takeover_active = (now_ms - g_rc_state.takeover_since_ms) >= APP_CFG_RC_TAKEOVER_HOLD_MS;
    }
    else
    {
        g_rc_state.takeover_since_ms = 0U;
        g_rc_state.takeover_active = false;
    }
}

void srv_rc_refresh_online(uint32_t now_ms)
{
    if (bsp_time_is_expired(now_ms, g_rc_state.last_update_ms, APP_CFG_RC_TIMEOUT_MS))
    {
        g_rc_state.online = false;
        g_rc_state.takeover_active = false;
        g_rc_state.takeover_since_ms = 0U;
    }
}

rc_state_t srv_rc_get_state(void)
{
    return g_rc_state;
}

app_mode_t srv_rc_get_requested_mode(void)
{
    /* 遥控器离线时，模式请求直接视为 SAFE。 */
    if (!g_rc_state.online)
        return APP_MODE_SAFE;

    /* 当前只用左拨杆 S1 选模式：上 SAFE，中/下都视为 MANUAL。 */
    if (g_rc_state.s1 == RC_SWITCH_UP)
        return APP_MODE_SAFE;
    if ((g_rc_state.s1 == RC_SWITCH_MID) || (g_rc_state.s1 == RC_SWITCH_DOWN))
        return APP_MODE_MANUAL;

    return APP_MODE_SAFE;
}

chassis_cmd_t srv_rc_get_manual_cmd(void)
{
    chassis_cmd_t cmd;

    /*
     * 当前 MANUAL 模式的底盘映射为：
     * - ch3: 左摇杆纵向 -> vx 前后
     * - ch2: 左摇杆横向 -> vy 横移
     * - ch0: 右摇杆横向 -> wz 自转
     * ch1 和键鼠输入暂不参与底盘控制。
     */
    cmd.vx_mps = g_rc_state.ch_norm[3] * APP_CFG_RC_SIGN_VX * APP_CFG_MAX_VX_MPS;
    cmd.vy_mps = g_rc_state.ch_norm[2] * APP_CFG_RC_SIGN_VY * APP_CFG_MAX_VY_MPS;
    cmd.wz_radps = g_rc_state.ch_norm[0] * APP_CFG_RC_SIGN_WZ * APP_CFG_MAX_WZ_RADPS;
    return cmd;
}
