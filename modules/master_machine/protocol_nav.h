#ifndef USER_MIDDLEWARE_PROTOCOL_NAV_H
#define USER_MIDDLEWARE_PROTOCOL_NAV_H

#include <stdint.h>

#include "robot_def.h"

/* 轮速惯性里程计观测标志位。 */
#define PROTOCOL_NAV_OBS_FLAG_IMU_VALID           (1U << 0)
#define PROTOCOL_NAV_OBS_FLAG_WHEEL_FL_VALID      (1U << 1)
#define PROTOCOL_NAV_OBS_FLAG_WHEEL_FR_VALID      (1U << 2)
#define PROTOCOL_NAV_OBS_FLAG_WHEEL_RL_VALID      (1U << 3)
#define PROTOCOL_NAV_OBS_FLAG_WHEEL_RR_VALID      (1U << 4)
#define PROTOCOL_NAV_OBS_FLAG_IMU_FAULT           (1U << 5)
#define PROTOCOL_NAV_OBS_FLAG_MOTOR_FAULT         (1U << 6)
#define PROTOCOL_NAV_OBS_FLAG_CHASSIS_SAFE_MODE   (1U << 7)
#define PROTOCOL_NAV_OBS_FLAG_RC_OFFLINE          (1U << 8)

/* MCU -> NAV 观测载荷：时间戳 + 四轮轮速 + 三轴角速度 + 三轴加速度 + 状态位。 */
typedef struct
{
    uint32_t t_ms;
    float w_fl;
    float w_fr;
    float w_rl;
    float w_rr;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    uint16_t flags;
} protocol_nav_observation_payload_t;

/* 将观测结构编码成 USB CDC 发送帧：A5 5A | seq | len | payload | crc16。 */
uint16_t protocol_nav_encode_observation(uint8_t seq, const protocol_nav_observation_payload_t *payload, uint8_t *buffer, uint16_t buffer_size);

#endif /* USER_MIDDLEWARE_PROTOCOL_NAV_H */
