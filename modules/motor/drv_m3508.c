#include "drv_m3508.h"

#include <math.h>

#include "robot_def.h"
#include "common_math.h"

void drv_m3508_parse_feedback(const uint8_t raw_data[8], drv_m3508_feedback_t *feedback_out)
{
    feedback_out->ecd = (uint16_t)(((uint16_t)raw_data[0] << 8U) | raw_data[1]);
    feedback_out->rpm = (int16_t)(((uint16_t)raw_data[2] << 8U) | raw_data[3]);
    feedback_out->current = (int16_t)(((uint16_t)raw_data[4] << 8U) | raw_data[5]);
    feedback_out->temperature = raw_data[6];
    feedback_out->wheel_speed_radps = drv_m3508_motor_rpm_to_wheel_radps(feedback_out->rpm);
}

float drv_m3508_motor_rpm_to_wheel_radps(int16_t motor_rpm)
{
    /* 先把电机轴 rpm 转成 rad/s，再除以减速比得到车轮输出端速度。 */
    const float motor_radps = ((float)motor_rpm * 2.0f * 3.1415926535f) / 60.0f;
    return motor_radps / APP_CFG_M3508_GEAR_RATIO;
}

int16_t drv_m3508_current_cmd_to_raw(float current_cmd)
{
    return common_clamp_i16((int32_t)lroundf(current_cmd),
                            (int16_t)-APP_CFG_C620_CURRENT_LIMIT,
                            (int16_t)APP_CFG_C620_CURRENT_LIMIT);
}
