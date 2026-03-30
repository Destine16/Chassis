#ifndef USER_DRIVERS_DRV_M3508_H
#define USER_DRIVERS_DRV_M3508_H

#include <stdint.h>

/* 单个 M3508 反馈报文字段。 */
typedef struct
{
    uint16_t ecd;
    int16_t rpm;
    int16_t current;
    uint8_t temperature;
    float wheel_speed_radps;
} drv_m3508_feedback_t;

/* 解析 C620 回传的 8 字节标准反馈报文。 */
void drv_m3508_parse_feedback(const uint8_t raw_data[8], drv_m3508_feedback_t *feedback_out);

/* 将电机转速 rpm 转成减速箱输出端轮速 rad/s。 */
float drv_m3508_motor_rpm_to_wheel_radps(int16_t motor_rpm);

/* 将浮点电流给定裁剪并转换为 C620 发送原始量。 */
int16_t drv_m3508_current_cmd_to_raw(float current_cmd);

#endif /* USER_DRIVERS_DRV_M3508_H */
