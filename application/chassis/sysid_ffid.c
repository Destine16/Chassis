#include "sysid_internal.h"

#include "ctrl_chassis.h"
#include "robot_def.h"
#include "srv_motor.h"

static uint32_t ffid_total_steps(uint32_t speed_level_count)
{
    return (2U * speed_level_count) + 3U;
}

static int16_t ffid_compute_step_id(uint32_t step_index, uint32_t speed_level_count)
{
    if (step_index == 0U)
        return (int16_t)FFID_STEP_PRE_HOLD;
    if (step_index <= speed_level_count)
        return (int16_t)step_index;
    if (step_index == (speed_level_count + 1U))
        return (int16_t)FFID_STEP_MID_HOLD;
    if (step_index <= ((2U * speed_level_count) + 1U))
        return (int16_t)(-((int32_t)step_index - (int32_t)(speed_level_count + 1U)));
    return (int16_t)FFID_STEP_POST_HOLD;
}

static float ffid_step_speed_ref_radps(int16_t step_id, float speed_max_radps, uint32_t speed_level_count)
{
    if ((step_id == (int16_t)FFID_STEP_PRE_HOLD) ||
        (step_id == (int16_t)FFID_STEP_MID_HOLD) ||
        (step_id == (int16_t)FFID_STEP_POST_HOLD) ||
        (step_id == (int16_t)FFID_STEP_DONE))
    {
        return 0.0f;
    }

    if (speed_level_count == 0U)
        return 0.0f;

    return ((float)step_id * speed_max_radps) / (float)speed_level_count;
}

static void ffid_advance_step(ffid_runtime_t *rt)
{
    rt->step_sample_index = 0U;
    rt->current_step_index++;
    if (rt->current_step_index >= rt->total_steps)
    {
        rt->current_step_id = (int16_t)FFID_STEP_DONE;
        rt->pending_stop = true;
        rt->current_speed_ref_radps = 0.0f;
        return;
    }

    rt->current_step_id = ffid_compute_step_id(rt->current_step_index, rt->speed_level_count);
    rt->current_speed_ref_radps =
        ffid_step_speed_ref_radps(rt->current_step_id, g_ffid_meta.speed_max_radps, rt->speed_level_count);
}

void ffid_begin(void)
{
    float speed_max_radps = ffid_clamp_speed_max(g_ffid_speed_max_radps);
    uint32_t speed_level_count = ffid_clamp_speed_level_count(g_ffid_speed_level_count);

    g_ffid_rt.running = true;
    g_ffid_rt.pending_stop = false;
    g_ffid_rt.saved_ff_enable = g_ctrl_chassis_speed_ff_param.enable;
    g_ffid_rt.wheel_id = sysid_clamp_wheel_id(g_ffid_wheel_id);
    g_ffid_rt.hold_samples = ffid_compute_duration_samples(g_ffid_hold_duration_ms);
    g_ffid_rt.speed_level_count = speed_level_count;
    g_ffid_rt.total_steps = ffid_total_steps(speed_level_count);
    g_ffid_rt.current_step_index = 0U;
    g_ffid_rt.step_sample_index = 0U;
    g_ffid_rt.current_step_id = (int16_t)FFID_STEP_PRE_HOLD;
    g_ffid_rt.current_speed_ref_radps = 0.0f;
    g_ffid_rt.last_applied_u_raw = 0;
    g_ffid_rt.last_applied_step_id = (int16_t)FFID_STEP_PRE_HOLD;

    g_ffid_meta.sample_hz = APP_CFG_CONTROL_HZ;
    g_ffid_meta.sample_count = 0U;
    g_ffid_meta.capacity = APP_CFG_SYSID_SAMPLE_CAPACITY;
    g_ffid_meta.wheel_id = g_ffid_rt.wheel_id;
    g_ffid_meta.speed_max_radps = speed_max_radps;
    g_ffid_meta.speed_level_count = speed_level_count;
    g_ffid_meta.hold_duration_ms = g_ffid_hold_duration_ms;
    g_ffid_meta.settle_skip_ms = g_ffid_settle_skip_ms;

    g_ffid_arm = 0U;
    g_ffid_running = 1U;
    g_ffid_hold_zero = 0U;

    /* FFID 要测的是纯 PID 为维持稳态速度输出的电流，不应混入前馈。 */
    g_ctrl_chassis_speed_ff_param.enable = 0U;
    ctrl_chassis_stop();
}

bool ffid_task_step(void)
{
    motor_feedback_t feedback[COMMON_WHEEL_COUNT];
    uint32_t sample_index;
    wheel_targets_t targets;

    if ((!g_ffid_rt.running) && (g_ffid_hold_zero != 0U))
    {
        ctrl_chassis_stop();
        return true;
    }

    if (!g_ffid_rt.running)
        return false;

    if (g_ffid_rt.pending_stop)
    {
        ffid_finish_and_zero_output();
        return true;
    }

    srv_motor_get_feedback_all(feedback);

    if (!feedback[g_ffid_rt.wheel_id].online)
    {
        ffid_finish_and_zero_output();
        return true;
    }

    sample_index = g_ffid_meta.sample_count;
    if (sample_index < APP_CFG_SYSID_SAMPLE_CAPACITY)
    {
        g_ffid_buffer[sample_index].u_raw = g_ffid_rt.last_applied_u_raw;
        g_ffid_buffer[sample_index].reserved = g_ffid_rt.last_applied_step_id;
        g_ffid_buffer[sample_index].speed_radps =
            ctrl_chassis_motor_sign((chassis_wheel_id_t)g_ffid_rt.wheel_id) *
            feedback[g_ffid_rt.wheel_id].wheel_speed_radps;
        g_ffid_meta.sample_count = sample_index + 1U;
    }
    else
    {
        g_ffid_rt.pending_stop = true;
    }

    ctrl_chassis_execute_single_wheel_speed((chassis_wheel_id_t)g_ffid_rt.wheel_id,
                                            g_ffid_rt.current_speed_ref_radps);
    srv_motor_get_targets(&targets);
    g_ffid_rt.last_applied_u_raw = ffid_round_current_cmd(targets.current_cmd[g_ffid_rt.wheel_id]);
    g_ffid_rt.last_applied_step_id = g_ffid_rt.current_step_id;

    g_ffid_rt.step_sample_index++;
    if (g_ffid_rt.step_sample_index >= g_ffid_rt.hold_samples)
        ffid_advance_step(&g_ffid_rt);

    return true;
}
