#include "sysid.h"

#include <string.h>

#include "robot_def.h"

#if (APP_CFG_ENABLE_EXPERIMENTS == 0U)

volatile uint32_t g_sysid_arm = 0U;
volatile uint32_t g_sysid_running = 0U;
volatile uint32_t g_sysid_hold_zero = 0U;
volatile uint32_t g_sysid_wheel_id = APP_CFG_SYSID_DEFAULT_WHEEL_ID;
volatile int32_t g_sysid_amplitude_raw = APP_CFG_SYSID_DEFAULT_AMPLITUDE_RAW;
volatile uint32_t g_sysid_bit_period_ms = APP_CFG_SYSID_DEFAULT_BIT_PERIOD_MS;
volatile uint32_t g_sysid_total_bits = APP_CFG_SYSID_DEFAULT_TOTAL_BITS;

volatile uint32_t g_ffid_arm = 0U;
volatile uint32_t g_ffid_running = 0U;
volatile uint32_t g_ffid_hold_zero = 0U;
volatile uint32_t g_ffid_wheel_id = APP_CFG_FFID_DEFAULT_WHEEL_ID;
volatile float g_ffid_speed_max_radps = APP_CFG_FFID_DEFAULT_SPEED_MAX_RADPS;
volatile uint32_t g_ffid_speed_level_count = APP_CFG_FFID_DEFAULT_SPEED_LEVEL_COUNT;
volatile uint32_t g_ffid_hold_duration_ms = APP_CFG_FFID_DEFAULT_HOLD_DURATION_MS;
volatile uint32_t g_ffid_settle_skip_ms = APP_CFG_FFID_DEFAULT_SETTLE_SKIP_MS;

volatile uint32_t g_wheeltest_enable = 0U;
volatile uint32_t g_wheeltest_active = 0U;
volatile uint32_t g_wheeltest_hold_zero = 0U;
volatile uint32_t g_wheeltest_wheel_id = APP_CFG_SYSID_DEFAULT_WHEEL_ID;
volatile float g_wheeltest_speed_ref_radps = 0.0f;

sysid_meta_t g_sysid_meta;
ffid_meta_t g_ffid_meta;
sysid_sample_t g_sysid_buffer[1];
sysid_sample_t *const g_ffid_buffer = g_sysid_buffer;

void sysid_init(void)
{
    memset(&g_sysid_meta, 0, sizeof(g_sysid_meta));
    memset(&g_ffid_meta, 0, sizeof(g_ffid_meta));
    g_sysid_meta.sample_hz = APP_CFG_CONTROL_HZ;
    g_ffid_meta.sample_hz = APP_CFG_CONTROL_HZ;
}

#endif
