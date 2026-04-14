#include "ctrl_chassis.h"

#include <math.h>
#include <string.h>

#include "robot_def.h"
#include "omni_chassis_kinematics.h"
#include "drv_can_motor.h"
#include "drv_m3508.h"
#include "pid.h"
#include "ramp.h"
#include "soft_limit.h"
#include "srv_chassis.h"
#include "srv_motor.h"

typedef struct
{
    pid_controller_t speed_pid[COMMON_WHEEL_COUNT];
    ramp_filter_t vx_ramp;
    ramp_filter_t vy_ramp;
    ramp_filter_t wz_ramp;
} ctrl_chassis_context_t;

static ctrl_chassis_context_t g_ctrl_ctx;

float ctrl_chassis_motor_sign(chassis_wheel_id_t wheel_id)
{
    /*
     * 结合当前实物安装前提写死：
     * - M3508 正方向按官方定义：面对输出轴时逆时针为正
     * - 四个电机输出轴都朝底盘外侧，对称安装
     *
     * 在这个前提下，底盘抽象轮坐标的正方向与实际电机正方向之间：
     * - 左侧两轮相反
     * - 右侧两轮相同
     */
    switch (wheel_id)
    {
    case CHASSIS_WHEEL_FRONT_LEFT:
    case CHASSIS_WHEEL_REAR_LEFT:
        return -1.0f;

    case CHASSIS_WHEEL_FRONT_RIGHT:
    case CHASSIS_WHEEL_REAR_RIGHT:
    default:
        return 1.0f;
    }
}

/*
 * 默认共享速度环参数采用当前单轮辨识后验证可用的 balanced 结果，
 * 上电即可直接进行遥控底盘测试；运行中仍可在调试器里在线修改。
 */
ctrl_chassis_speed_pid_param_t g_ctrl_chassis_speed_pid_param = {
    .kp = 116.84735568148633f,
    .ki = 320.5727636311219f,
    .kd = 0.17825148011568537f,
    .integral_limit = APP_CFG_SPEED_PID_INT_LIMIT,
    .output_limit = APP_CFG_SPEED_PID_OUT_LIMIT,
};

/*
 * 当前默认前馈来自单轮稳态速度平台实验的推荐对称模型：
 * u_ff = 425.378 * sign(w_ref) + 1.14785 * w_ref
 */
ctrl_chassis_speed_ff_param_t g_ctrl_chassis_speed_ff_param = {
    .enable = 1U,
    .k_s = 425.378f,
    .k_v = 1.14785f,
};

static float ctrl_chassis_compute_speed_ff(float speed_ref_radps)
{
    if (g_ctrl_chassis_speed_ff_param.enable == 0U)
        return 0.0f;

    if (fabsf(speed_ref_radps) <= APP_CFG_SPEED_PID_ZERO_REF_RADPS)
        return 0.0f;

    if (speed_ref_radps > 0.0f)
        return g_ctrl_chassis_speed_ff_param.k_s + (g_ctrl_chassis_speed_ff_param.k_v * speed_ref_radps);

    return -g_ctrl_chassis_speed_ff_param.k_s + (g_ctrl_chassis_speed_ff_param.k_v * speed_ref_radps);
}

static void ctrl_chassis_sync_speed_pid_params(void)
{
    uint32_t index = 0U;

    while (index < COMMON_WHEEL_COUNT)
    {
        g_ctrl_ctx.speed_pid[index].kp = g_ctrl_chassis_speed_pid_param.kp;
        g_ctrl_ctx.speed_pid[index].ki = g_ctrl_chassis_speed_pid_param.ki;
        g_ctrl_ctx.speed_pid[index].kd = g_ctrl_chassis_speed_pid_param.kd;
        g_ctrl_ctx.speed_pid[index].integral_limit = g_ctrl_chassis_speed_pid_param.integral_limit;
        g_ctrl_ctx.speed_pid[index].output_limit = g_ctrl_chassis_speed_pid_param.output_limit;
        g_ctrl_ctx.speed_pid[index].zero_ref_threshold = APP_CFG_SPEED_PID_ZERO_REF_RADPS;
        g_ctrl_ctx.speed_pid[index].zero_feedback_threshold = APP_CFG_SPEED_PID_ZERO_FB_RADPS;
        g_ctrl_ctx.speed_pid[index].integral_separation_error = APP_CFG_SPEED_PID_INT_SEP_ERR_RADPS;
        index++;
    }
}

void ctrl_chassis_init(void)
{
    uint32_t index = 0U;

    /* 四个轮子各自维护独立速度 PID。 */
    while (index < COMMON_WHEEL_COUNT)
    {
        pid_init(&g_ctrl_ctx.speed_pid[index],
                 g_ctrl_chassis_speed_pid_param.kp,
                 g_ctrl_chassis_speed_pid_param.ki,
                 g_ctrl_chassis_speed_pid_param.kd,
                 g_ctrl_chassis_speed_pid_param.integral_limit,
                 g_ctrl_chassis_speed_pid_param.output_limit);
        index++;
    }

    /* 对底盘三自由度命令分别做斜坡限制。 */
    ramp_init(&g_ctrl_ctx.vx_ramp, APP_CFG_CMD_RAMP_VX_MPS2);
    ramp_init(&g_ctrl_ctx.vy_ramp, APP_CFG_CMD_RAMP_VY_MPS2);
    ramp_init(&g_ctrl_ctx.wz_ramp, APP_CFG_CMD_RAMP_WZ_RADPS2);
}

void ctrl_chassis_stop(void)
{
    wheel_targets_t targets;
    uint32_t index = 0U;

    /* SAFE 模式下重置控制器内部状态，避免退出 SAFE 时残留积分。 */
    memset(&targets, 0, sizeof(targets));

    while (index < COMMON_WHEEL_COUNT)
    {
        pid_reset(&g_ctrl_ctx.speed_pid[index]);
        index++;
    }

    ramp_reset(&g_ctrl_ctx.vx_ramp, 0.0f);
    ramp_reset(&g_ctrl_ctx.vy_ramp, 0.0f);
    ramp_reset(&g_ctrl_ctx.wz_ramp, 0.0f);
    srv_motor_zero_targets();
    srv_chassis_stop();
    (void)drv_can_motor_send_chassis_currents(0, 0, 0, 0);
    srv_chassis_set_wheel_targets(&targets);
}

void ctrl_chassis_execute_single_wheel_speed(chassis_wheel_id_t wheel_id, float speed_ref_radps)
{
    wheel_targets_t targets;
    int16_t current_raw[COMMON_WHEEL_COUNT];
    float speed_ref[COMMON_WHEEL_COUNT] = {0.0f};
    float dt_s = 1.0f / (float)APP_CFG_CONTROL_HZ;
    uint32_t index = 0U;

    memset(&targets, 0, sizeof(targets));
    memset(current_raw, 0, sizeof(current_raw));
    ctrl_chassis_sync_speed_pid_params();

    if (wheel_id >= COMMON_WHEEL_COUNT)
    {
        ctrl_chassis_stop();
        return;
    }

    speed_ref[wheel_id] = speed_ref_radps;

    while (index < COMMON_WHEEL_COUNT)
    {
        motor_feedback_t feedback = srv_motor_get_feedback((chassis_wheel_id_t)index);
        float motor_sign = ctrl_chassis_motor_sign((chassis_wheel_id_t)index);
        float signed_feedback_wheel_speed = motor_sign * feedback.wheel_speed_radps;
        float ff_cmd = ctrl_chassis_compute_speed_ff(speed_ref[index]);
        float pid_cmd = pid_update(&g_ctrl_ctx.speed_pid[index],
                                   speed_ref[index],
                                   signed_feedback_wheel_speed,
                                   dt_s);
        float cmd = ff_cmd + pid_cmd;

        cmd = soft_limit_symmetric(cmd, APP_CFG_C620_CURRENT_LIMIT);
        current_raw[index] = drv_m3508_current_cmd_to_raw(motor_sign * cmd);
        targets.wheel_speed_ref_radps[index] = speed_ref[index];
        targets.current_cmd[index] = cmd;
        index++;
    }

    (void)drv_can_motor_send_chassis_currents(current_raw[0], current_raw[1], current_raw[2], current_raw[3]);
    srv_motor_set_targets(targets.wheel_speed_ref_radps, targets.current_cmd);
    srv_chassis_stop();
    srv_chassis_set_wheel_targets(&targets);
}

void ctrl_chassis_execute(app_mode_t mode, const chassis_cmd_t *final_cmd)
{
    chassis_cmd_t limited_cmd = *final_cmd;
    chassis_cmd_t actual_cmd = {0};
    wheel_targets_t targets;
    float actual_wheel_speed_radps[COMMON_WHEEL_COUNT];
    int16_t current_raw[COMMON_WHEEL_COUNT];
    float dt_s = 1.0f / (float)APP_CFG_CONTROL_HZ;
    uint32_t index = 0U;

    memset(&targets, 0, sizeof(targets));
    memset(actual_wheel_speed_radps, 0, sizeof(actual_wheel_speed_radps));
    memset(current_raw, 0, sizeof(current_raw));

    /* SAFE 模式直接清零所有输出。 */
    if (mode == APP_MODE_SAFE)
    {
        ctrl_chassis_stop();
        return;
    }

    /* 允许调试器直接改全局 PID 参数，下一拍自动同步到四个轮子。 */
    ctrl_chassis_sync_speed_pid_params();

    /* 先做命令斜坡，再做幅值限制。 */
    limited_cmd.vx_mps = ramp_apply(&g_ctrl_ctx.vx_ramp, limited_cmd.vx_mps, dt_s);
    limited_cmd.vy_mps = ramp_apply(&g_ctrl_ctx.vy_ramp, limited_cmd.vy_mps, dt_s);
    limited_cmd.wz_radps = ramp_apply(&g_ctrl_ctx.wz_ramp, limited_cmd.wz_radps, dt_s);
    soft_limit_chassis_cmd(&limited_cmd);

    /* 将车体速度命令解算成四个轮子的角速度参考。 */
    omni_chassis_kinematics_inverse(&limited_cmd, targets.wheel_speed_ref_radps);
    (void)omni_chassis_desaturate_wheel_speeds(targets.wheel_speed_ref_radps, APP_CFG_MAX_WHEEL_SPEED_RADPS);
    omni_chassis_kinematics_forward(targets.wheel_speed_ref_radps, &limited_cmd);

    while (index < COMMON_WHEEL_COUNT)
    {
        motor_feedback_t feedback = srv_motor_get_feedback((chassis_wheel_id_t)index);
        float motor_sign = ctrl_chassis_motor_sign((chassis_wheel_id_t)index);
        float signed_feedback_wheel_speed = motor_sign * feedback.wheel_speed_radps;
        float ff_cmd = ctrl_chassis_compute_speed_ff(targets.wheel_speed_ref_radps[index]);

        /* 底盘轮第一版只做速度外环，PID 输出直接作为 C620 电流给定。 */
        float pid_cmd = pid_update(&g_ctrl_ctx.speed_pid[index],
                                   targets.wheel_speed_ref_radps[index],
                                   signed_feedback_wheel_speed,
                                   dt_s);
        float current_cmd = ff_cmd + pid_cmd;

        current_cmd = soft_limit_symmetric(current_cmd, APP_CFG_C620_CURRENT_LIMIT);
        actual_wheel_speed_radps[index] = signed_feedback_wheel_speed;
        targets.current_cmd[index] = current_cmd;
        current_raw[index] = drv_m3508_current_cmd_to_raw(motor_sign * current_cmd);
        index++;
    }

    omni_chassis_kinematics_forward(actual_wheel_speed_radps, &actual_cmd);

    /* 所有电流给定统一在这里一次性下发，保证唯一最终输出口。 */
    (void)drv_can_motor_send_chassis_currents(current_raw[0], current_raw[1], current_raw[2], current_raw[3]);
    srv_motor_set_targets(targets.wheel_speed_ref_radps, targets.current_cmd);
    srv_chassis_set_final_cmd(final_cmd);
    srv_chassis_set_limited_cmd(&limited_cmd);
    srv_chassis_set_actual_cmd(&actual_cmd);
    srv_chassis_set_wheel_targets(&targets);
}
