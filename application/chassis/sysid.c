#include "sysid.h"

#include <string.h>

#include "ctrl_chassis.h"
#include "drv_can_motor.h"
#include "robot_def.h"
#include "srv_motor.h"

typedef struct
{
    bool running;
    bool pending_stop;
    uint16_t lfsr;
    uint32_t wheel_id;
    uint32_t bit_period_samples;
    uint32_t samples_left_in_bit;
    uint32_t bits_completed;
    int16_t current_u_raw;
    int16_t last_applied_u_raw;
} sysid_runtime_t;

typedef struct
{
    bool active;
    uint32_t wheel_id;
} wheeltest_runtime_t;

volatile uint32_t g_sysid_arm = 0U;
volatile uint32_t g_sysid_running = 0U;
volatile uint32_t g_sysid_hold_zero = 0U;
volatile uint32_t g_sysid_wheel_id = APP_CFG_SYSID_DEFAULT_WHEEL_ID;
volatile int32_t g_sysid_amplitude_raw = APP_CFG_SYSID_DEFAULT_AMPLITUDE_RAW;
volatile uint32_t g_sysid_bit_period_ms = APP_CFG_SYSID_DEFAULT_BIT_PERIOD_MS;
volatile uint32_t g_sysid_total_bits = APP_CFG_SYSID_DEFAULT_TOTAL_BITS;
volatile uint32_t g_wheeltest_enable = 0U;
volatile uint32_t g_wheeltest_active = 0U;
volatile uint32_t g_wheeltest_hold_zero = 0U;
volatile uint32_t g_wheeltest_wheel_id = APP_CFG_SYSID_DEFAULT_WHEEL_ID;
volatile float g_wheeltest_speed_ref_radps = 10.0f;

sysid_meta_t g_sysid_meta;
sysid_sample_t __attribute__((section(".ccmram"))) g_sysid_buffer[APP_CFG_SYSID_SAMPLE_CAPACITY];

static sysid_runtime_t g_sysid_rt;
static wheeltest_runtime_t g_wheeltest_rt;

static uint32_t sysid_clamp_wheel_id(uint32_t wheel_id)
{
    if (wheel_id >= COMMON_WHEEL_COUNT)
        return (uint32_t)APP_CFG_SYSID_DEFAULT_WHEEL_ID;
    return wheel_id;
}

static int16_t sysid_clamp_amplitude(int32_t amplitude_raw)
{
    if (amplitude_raw < 0)
        amplitude_raw = -amplitude_raw;
    if (amplitude_raw > (int32_t)APP_CFG_C620_CURRENT_LIMIT)
        amplitude_raw = (int32_t)APP_CFG_C620_CURRENT_LIMIT;
    if (amplitude_raw == 0)
        amplitude_raw = APP_CFG_SYSID_DEFAULT_AMPLITUDE_RAW;
    return (int16_t)amplitude_raw;
}

static uint32_t sysid_compute_bit_period_samples(uint32_t bit_period_ms)
{
    uint32_t samples = (bit_period_ms * APP_CFG_CONTROL_HZ) / 1000U;

    if (samples == 0U)
        samples = 1U;
    return samples;
}

static int16_t sysid_next_prbs_level(int16_t amplitude_raw)
{
    uint16_t lsb;

    if (g_sysid_rt.lfsr == 0U)
        g_sysid_rt.lfsr = 0x5Au;

    lsb = (uint16_t)(g_sysid_rt.lfsr & 1U);
    g_sysid_rt.lfsr >>= 1U;
    if (lsb != 0U)
        g_sysid_rt.lfsr ^= 0xB400u;

    return ((g_sysid_rt.lfsr & 1U) != 0U) ? amplitude_raw : (int16_t)-amplitude_raw;
}

static void sysid_finish_and_zero_output(void)
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

static void wheeltest_finish_and_zero_output(void)
{
    g_wheeltest_rt.active = false;
    g_wheeltest_active = 0U;
    g_wheeltest_enable = 0U;
    g_wheeltest_hold_zero = 1U;
    ctrl_chassis_stop();
}

static void wheeltest_begin(void)
{
    g_wheeltest_rt.active = true;
    g_wheeltest_rt.wheel_id = sysid_clamp_wheel_id(g_wheeltest_wheel_id);
    g_wheeltest_active = 1U;
    g_wheeltest_hold_zero = 0U;
    ctrl_chassis_stop();
}

static void sysid_begin(void)
{
    int16_t amplitude_raw = sysid_clamp_amplitude(g_sysid_amplitude_raw);

    g_sysid_rt.running = true;
    g_sysid_rt.pending_stop = false;
    g_sysid_rt.lfsr = 0x5Au;
    g_sysid_rt.wheel_id = sysid_clamp_wheel_id(g_sysid_wheel_id);
    g_sysid_rt.bit_period_samples = sysid_compute_bit_period_samples(g_sysid_bit_period_ms);
    g_sysid_rt.samples_left_in_bit = g_sysid_rt.bit_period_samples;
    g_sysid_rt.bits_completed = 0U;
    g_sysid_rt.current_u_raw = sysid_next_prbs_level(amplitude_raw);
    g_sysid_rt.last_applied_u_raw = 0;

    g_sysid_meta.sample_hz = APP_CFG_CONTROL_HZ;
    g_sysid_meta.sample_count = 0U;
    g_sysid_meta.capacity = APP_CFG_SYSID_SAMPLE_CAPACITY;
    g_sysid_meta.wheel_id = g_sysid_rt.wheel_id;
    g_sysid_meta.amplitude_raw = amplitude_raw;
    g_sysid_meta.bit_period_ms = g_sysid_bit_period_ms;
    g_sysid_meta.total_bits = (g_sysid_total_bits == 0U) ? APP_CFG_SYSID_DEFAULT_TOTAL_BITS : g_sysid_total_bits;

    g_sysid_arm = 0U;
    g_sysid_running = 1U;
    g_sysid_hold_zero = 0U;

    ctrl_chassis_stop();
}

void sysid_init(void)
{
    memset(&g_sysid_rt, 0, sizeof(g_sysid_rt));
    memset(&g_wheeltest_rt, 0, sizeof(g_wheeltest_rt));
    memset(&g_sysid_meta, 0, sizeof(g_sysid_meta));
    g_sysid_meta.sample_hz = APP_CFG_CONTROL_HZ;
    g_sysid_meta.capacity = APP_CFG_SYSID_SAMPLE_CAPACITY;
    g_sysid_meta.wheel_id = APP_CFG_SYSID_DEFAULT_WHEEL_ID;
    g_sysid_meta.amplitude_raw = APP_CFG_SYSID_DEFAULT_AMPLITUDE_RAW;
    g_sysid_meta.bit_period_ms = APP_CFG_SYSID_DEFAULT_BIT_PERIOD_MS;
    g_sysid_meta.total_bits = APP_CFG_SYSID_DEFAULT_TOTAL_BITS;
    g_sysid_hold_zero = 0U;
    g_wheeltest_hold_zero = 0U;
}

bool sysid_task_step(void)
{
    motor_feedback_t feedback[COMMON_WHEEL_COUNT];
    float current_cmd[COMMON_WHEEL_COUNT] = {0.0f};
    float speed_ref[COMMON_WHEEL_COUNT] = {0.0f};
    int16_t current_raw[COMMON_WHEEL_COUNT] = {0};
    uint32_t sample_index;

    if ((!g_sysid_rt.running) && (g_sysid_arm != 0U))
        sysid_begin();

    if ((!g_wheeltest_rt.active) && (g_wheeltest_enable != 0U))
        wheeltest_begin();

    if ((!g_wheeltest_rt.active) && (g_wheeltest_hold_zero != 0U))
    {
        ctrl_chassis_stop();
        return true;
    }

    if (g_wheeltest_rt.active)
    {
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

    if ((!g_sysid_rt.running) && (g_sysid_hold_zero != 0U))
    {
        ctrl_chassis_stop();
        return true;
    }

    if (!g_sysid_rt.running)
        return false;

    if (g_sysid_rt.pending_stop)
    {
        sysid_finish_and_zero_output();
        return true;
    }

    srv_motor_get_feedback_all(feedback);

    if (!feedback[g_sysid_rt.wheel_id].online)
    {
        sysid_finish_and_zero_output();
        return true;
    }

    current_raw[g_sysid_rt.wheel_id] = g_sysid_rt.current_u_raw;
    current_cmd[g_sysid_rt.wheel_id] = (float)g_sysid_rt.current_u_raw;

    sample_index = g_sysid_meta.sample_count;
    if (sample_index < APP_CFG_SYSID_SAMPLE_CAPACITY)
    {
        g_sysid_buffer[sample_index].u_raw = g_sysid_rt.last_applied_u_raw;
        g_sysid_buffer[sample_index].reserved = 0;
        g_sysid_buffer[sample_index].speed_radps = feedback[g_sysid_rt.wheel_id].wheel_speed_radps;
        g_sysid_meta.sample_count = sample_index + 1U;
    }
    else
    {
        g_sysid_rt.pending_stop = true;
    }

    (void)drv_can_motor_send_chassis_currents(current_raw[0], current_raw[1], current_raw[2], current_raw[3]);
    srv_motor_set_targets(speed_ref, current_cmd);
    g_sysid_rt.last_applied_u_raw = g_sysid_rt.current_u_raw;

    if (g_sysid_rt.samples_left_in_bit > 0U)
        g_sysid_rt.samples_left_in_bit--;

    if (g_sysid_rt.samples_left_in_bit == 0U)
    {
        g_sysid_rt.bits_completed++;
        if (g_sysid_rt.bits_completed >= g_sysid_meta.total_bits)
        {
            g_sysid_rt.pending_stop = true;
        }
        else
        {
            g_sysid_rt.current_u_raw = sysid_next_prbs_level((int16_t)g_sysid_meta.amplitude_raw);
            g_sysid_rt.samples_left_in_bit = g_sysid_rt.bit_period_samples;
        }
    }

    return true;
}
