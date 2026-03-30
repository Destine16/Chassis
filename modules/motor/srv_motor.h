#ifndef USER_SERVICE_SRV_MOTOR_H
#define USER_SERVICE_SRV_MOTOR_H

#include "common_def.h"

/* 电机服务层统一缓存反馈和控制目标。 */
void srv_motor_init(void);

/* 从驱动层拉取 ISR 缓存的最新反馈。 */
void srv_motor_poll_driver_feedback(void);
void srv_motor_refresh_online(uint32_t now_ms);
motor_feedback_t srv_motor_get_feedback(chassis_wheel_id_t wheel_id);
void srv_motor_get_feedback_all(motor_feedback_t feedback_out[COMMON_WHEEL_COUNT]);
void srv_motor_set_targets(const float speed_ref[COMMON_WHEEL_COUNT], const float current_cmd[COMMON_WHEEL_COUNT]);
void srv_motor_get_targets(wheel_targets_t *targets_out);
void srv_motor_zero_targets(void);

#endif /* USER_SERVICE_SRV_MOTOR_H */
