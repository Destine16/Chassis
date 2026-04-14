#ifndef CHASSIS_APPLICATION_SYSID_INTERNAL_H
#define CHASSIS_APPLICATION_SYSID_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "sysid.h"

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

typedef enum
{
    FFID_STEP_PRE_HOLD = 0,
    FFID_STEP_MID_HOLD = 30000,
    FFID_STEP_POST_HOLD = 30001,
    FFID_STEP_DONE = 30002,
} ffid_step_special_t;

typedef struct
{
    bool running;
    bool pending_stop;
    uint32_t saved_ff_enable;
    uint32_t wheel_id;
    uint32_t hold_samples;
    uint32_t speed_level_count;
    uint32_t total_steps;
    uint32_t current_step_index;
    uint32_t step_sample_index;
    int16_t current_step_id;
    float current_speed_ref_radps;
    int16_t last_applied_u_raw;
    int16_t last_applied_step_id;
} ffid_runtime_t;

extern sysid_runtime_t g_sysid_rt;
extern ffid_runtime_t g_ffid_rt;
extern wheeltest_runtime_t g_wheeltest_rt;

uint32_t sysid_clamp_wheel_id(uint32_t wheel_id);
int16_t sysid_clamp_amplitude(int32_t amplitude_raw);
uint32_t sysid_compute_bit_period_samples(uint32_t bit_period_ms);
uint32_t ffid_compute_duration_samples(uint32_t duration_ms);
float ffid_clamp_speed_max(float speed_max_radps);
uint32_t ffid_clamp_speed_level_count(uint32_t speed_level_count);
int16_t ffid_round_current_cmd(float current_cmd);

void sysid_begin(void);
void ffid_begin(void);
void wheeltest_begin(void);

void sysid_finish_and_zero_output(void);
void ffid_finish_and_zero_output(void);
void wheeltest_finish_and_zero_output(void);

bool sysid_prbs_task_step(void);
bool ffid_task_step(void);
bool wheeltest_task_step(void);

#endif /* CHASSIS_APPLICATION_SYSID_INTERNAL_H */
