#include "sysid_internal.h"

#include <string.h>

#include "ctrl_chassis.h"
#include "robot_def.h"

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
volatile float g_wheeltest_speed_ref_radps = 10.0f;

sysid_meta_t g_sysid_meta;
ffid_meta_t g_ffid_meta;
sysid_sample_t __attribute__((section(".ccmram"))) g_sysid_buffer[APP_CFG_SYSID_SAMPLE_CAPACITY];
sysid_sample_t *const g_ffid_buffer = g_sysid_buffer;

sysid_runtime_t g_sysid_rt;
ffid_runtime_t g_ffid_rt;
wheeltest_runtime_t g_wheeltest_rt;

uint32_t sysid_clamp_wheel_id(uint32_t wheel_id)
{
    if (wheel_id >= COMMON_WHEEL_COUNT)
        return (uint32_t)APP_CFG_SYSID_DEFAULT_WHEEL_ID;
    return wheel_id;
}

int16_t sysid_clamp_amplitude(int32_t amplitude_raw)
{
    if (amplitude_raw < 0)
        amplitude_raw = -amplitude_raw;
    if (amplitude_raw > (int32_t)APP_CFG_C620_CURRENT_LIMIT)
        amplitude_raw = (int32_t)APP_CFG_C620_CURRENT_LIMIT;
    if (amplitude_raw == 0)
        amplitude_raw = APP_CFG_SYSID_DEFAULT_AMPLITUDE_RAW;
    return (int16_t)amplitude_raw;
}

uint32_t sysid_compute_bit_period_samples(uint32_t bit_period_ms)
{
    uint32_t samples = (bit_period_ms * APP_CFG_CONTROL_HZ) / 1000U;

    if (samples == 0U)
        samples = 1U;
    return samples;
}

uint32_t ffid_compute_duration_samples(uint32_t duration_ms)
{
    uint32_t samples = (duration_ms * APP_CFG_CONTROL_HZ) / 1000U;

    if (samples == 0U)
        samples = 1U;
    return samples;
}

float ffid_clamp_speed_max(float speed_max_radps)
{
    if (speed_max_radps < 0.0f)
        speed_max_radps = -speed_max_radps;
    if (speed_max_radps <= 0.0f)
        speed_max_radps = APP_CFG_FFID_DEFAULT_SPEED_MAX_RADPS;
    if (speed_max_radps > APP_CFG_MAX_WHEEL_SPEED_RADPS)
        speed_max_radps = APP_CFG_MAX_WHEEL_SPEED_RADPS;
    return speed_max_radps;
}

uint32_t ffid_clamp_speed_level_count(uint32_t speed_level_count)
{
    if (speed_level_count == 0U)
        return APP_CFG_FFID_DEFAULT_SPEED_LEVEL_COUNT;
    if (speed_level_count > 16U)
        return 16U;
    return speed_level_count;
}

int16_t ffid_round_current_cmd(float current_cmd)
{
    if (current_cmd >= 0.0f)
        return (int16_t)(current_cmd + 0.5f);
    return (int16_t)(current_cmd - 0.5f);
}

void sysid_init(void)
{
    memset(&g_sysid_rt, 0, sizeof(g_sysid_rt));
    memset(&g_ffid_rt, 0, sizeof(g_ffid_rt));
    memset(&g_wheeltest_rt, 0, sizeof(g_wheeltest_rt));
    memset(&g_sysid_meta, 0, sizeof(g_sysid_meta));
    memset(&g_ffid_meta, 0, sizeof(g_ffid_meta));
    g_sysid_meta.sample_hz = APP_CFG_CONTROL_HZ;
    g_sysid_meta.capacity = APP_CFG_SYSID_SAMPLE_CAPACITY;
    g_sysid_meta.wheel_id = APP_CFG_SYSID_DEFAULT_WHEEL_ID;
    g_sysid_meta.amplitude_raw = APP_CFG_SYSID_DEFAULT_AMPLITUDE_RAW;
    g_sysid_meta.bit_period_ms = APP_CFG_SYSID_DEFAULT_BIT_PERIOD_MS;
    g_sysid_meta.total_bits = APP_CFG_SYSID_DEFAULT_TOTAL_BITS;
    g_ffid_meta.sample_hz = APP_CFG_CONTROL_HZ;
    g_ffid_meta.capacity = APP_CFG_SYSID_SAMPLE_CAPACITY;
    g_ffid_meta.wheel_id = APP_CFG_FFID_DEFAULT_WHEEL_ID;
    g_ffid_meta.speed_max_radps = APP_CFG_FFID_DEFAULT_SPEED_MAX_RADPS;
    g_ffid_meta.speed_level_count = APP_CFG_FFID_DEFAULT_SPEED_LEVEL_COUNT;
    g_ffid_meta.hold_duration_ms = APP_CFG_FFID_DEFAULT_HOLD_DURATION_MS;
    g_ffid_meta.settle_skip_ms = APP_CFG_FFID_DEFAULT_SETTLE_SKIP_MS;
    g_sysid_hold_zero = 0U;
    g_ffid_hold_zero = 0U;
    g_wheeltest_hold_zero = 0U;
}

void sysid_finish_and_zero_output(void)
{
    g_sysid_rt.running = false;
    g_sysid_rt.pending_stop = false;
    g_sysid_rt.current_u_raw = 0;
    g_sysid_rt.last_applied_u_raw = 0;
    g_sysid_running = 0U;
    g_sysid_arm = 0U;
    g_sysid_hold_zero = 1U;
    ctrl_chassis_stop();
}

void ffid_finish_and_zero_output(void)
{
    g_ffid_rt.running = false;
    g_ffid_rt.pending_stop = false;
    g_ffid_rt.current_step_id = (int16_t)FFID_STEP_DONE;
    g_ffid_rt.current_speed_ref_radps = 0.0f;
    g_ffid_rt.last_applied_u_raw = 0;
    g_ffid_rt.last_applied_step_id = (int16_t)FFID_STEP_DONE;
    g_ffid_running = 0U;
    g_ffid_arm = 0U;
    g_ffid_hold_zero = 1U;
    g_ctrl_chassis_speed_ff_param.enable = g_ffid_rt.saved_ff_enable;
    ctrl_chassis_stop();
}

void wheeltest_finish_and_zero_output(void)
{
    g_wheeltest_rt.active = false;
    g_wheeltest_active = 0U;
    g_wheeltest_enable = 0U;
    g_wheeltest_hold_zero = 1U;
    ctrl_chassis_stop();
}

