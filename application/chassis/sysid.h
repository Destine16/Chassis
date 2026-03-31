#ifndef CHASSIS_APPLICATION_SYSID_H
#define CHASSIS_APPLICATION_SYSID_H

#include <stdbool.h>
#include <stdint.h>

#include "common_def.h"

typedef struct
{
    int16_t u_raw;
    int16_t reserved;
    float speed_radps;
} sysid_sample_t;

typedef struct
{
    uint32_t sample_hz;
    uint32_t sample_count;
    uint32_t capacity;
    uint32_t wheel_id;
    int32_t amplitude_raw;
    uint32_t bit_period_ms;
    uint32_t total_bits;
} sysid_meta_t;

extern volatile uint32_t g_sysid_arm;
extern volatile uint32_t g_sysid_running;
extern volatile uint32_t g_sysid_hold_zero;
extern volatile uint32_t g_sysid_wheel_id;
extern volatile int32_t g_sysid_amplitude_raw;
extern volatile uint32_t g_sysid_bit_period_ms;
extern volatile uint32_t g_sysid_total_bits;
extern volatile uint32_t g_wheeltest_enable;
extern volatile uint32_t g_wheeltest_active;
extern volatile uint32_t g_wheeltest_hold_zero;
extern volatile uint32_t g_wheeltest_wheel_id;
extern volatile float g_wheeltest_speed_ref_radps;

extern sysid_meta_t g_sysid_meta;
extern sysid_sample_t g_sysid_buffer[];

void sysid_init(void);
bool sysid_task_step(void);

#endif /* CHASSIS_APPLICATION_SYSID_H */
