#include "omni_chassis_kinematics.h"

#include "common_math.h"
#include "robot_def.h"

void omni_chassis_kinematics_inverse(const chassis_cmd_t *cmd, float wheel_speed_ref_radps[COMMON_WHEEL_COUNT])
{
    const float translation_coeff = APP_CFG_OMNI_45DEG_COEF / APP_CFG_OMNI_WHEEL_RADIUS_M;
    const float rotation_coeff = APP_CFG_OMNI_CENTER_TO_WHEEL_M / APP_CFG_OMNI_WHEEL_RADIUS_M;

    /*
     * 四轮 45° 全向轮逆运动学。
     * 当前保留现有轮序/正方向定义，只把 45° 投影系数和中心到轮心距离显式写入模型，
     * 避免把 vx/vy/wz 的物理量纲整体放大。
     */
    wheel_speed_ref_radps[CHASSIS_WHEEL_FRONT_LEFT] =
        ((cmd->vx_mps - cmd->vy_mps) * translation_coeff) - (cmd->wz_radps * rotation_coeff);
    wheel_speed_ref_radps[CHASSIS_WHEEL_FRONT_RIGHT] =
        ((cmd->vx_mps + cmd->vy_mps) * translation_coeff) + (cmd->wz_radps * rotation_coeff);
    wheel_speed_ref_radps[CHASSIS_WHEEL_REAR_LEFT] =
        ((cmd->vx_mps + cmd->vy_mps) * translation_coeff) - (cmd->wz_radps * rotation_coeff);
    wheel_speed_ref_radps[CHASSIS_WHEEL_REAR_RIGHT] =
        ((cmd->vx_mps - cmd->vy_mps) * translation_coeff) + (cmd->wz_radps * rotation_coeff);
}

void omni_chassis_kinematics_forward(const float wheel_speed_radps[COMMON_WHEEL_COUNT], chassis_cmd_t *cmd_out)
{
    const float translation_coeff = APP_CFG_OMNI_WHEEL_RADIUS_M / (2.0f * APP_CFG_OMNI_45DEG_COEF);
    const float rotation_coeff = APP_CFG_OMNI_WHEEL_RADIUS_M / (4.0f * APP_CFG_OMNI_CENTER_TO_WHEEL_M);

    /*
     * 由四轮实际角速度反算车体速度。
     * 与 inverse 使用同一套轮序和符号约定，避免调试时前后公式不一致。
     */
    cmd_out->vx_mps =
        translation_coeff * (wheel_speed_radps[CHASSIS_WHEEL_FRONT_LEFT] +
                             wheel_speed_radps[CHASSIS_WHEEL_FRONT_RIGHT] +
                             wheel_speed_radps[CHASSIS_WHEEL_REAR_LEFT] +
                             wheel_speed_radps[CHASSIS_WHEEL_REAR_RIGHT]) * 0.5f;

    cmd_out->vy_mps =
        translation_coeff * (-wheel_speed_radps[CHASSIS_WHEEL_FRONT_LEFT] +
                              wheel_speed_radps[CHASSIS_WHEEL_FRONT_RIGHT] +
                              wheel_speed_radps[CHASSIS_WHEEL_REAR_LEFT] -
                              wheel_speed_radps[CHASSIS_WHEEL_REAR_RIGHT]) * 0.5f;

    cmd_out->wz_radps =
        rotation_coeff * (-wheel_speed_radps[CHASSIS_WHEEL_FRONT_LEFT] +
                          wheel_speed_radps[CHASSIS_WHEEL_FRONT_RIGHT] -
                          wheel_speed_radps[CHASSIS_WHEEL_REAR_LEFT] +
                          wheel_speed_radps[CHASSIS_WHEEL_REAR_RIGHT]);
}

float omni_chassis_desaturate_wheel_speeds(float wheel_speed_radps[COMMON_WHEEL_COUNT], float max_abs_wheel_speed_radps)
{
    float max_found = 0.0f;
    float scale = 1.0f;
    uint32_t index = 0U;

    while (index < COMMON_WHEEL_COUNT)
    {
        float abs_speed = common_absf(wheel_speed_radps[index]);

        if (abs_speed > max_found)
            max_found = abs_speed;
        index++;
    }

    if ((max_found > max_abs_wheel_speed_radps) && (max_found > 0.0f))
    {
        scale = max_abs_wheel_speed_radps / max_found;
        index = 0U;
        while (index < COMMON_WHEEL_COUNT)
        {
            wheel_speed_radps[index] *= scale;
            index++;
        }
    }

    return scale;
}
