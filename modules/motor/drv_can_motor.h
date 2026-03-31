#ifndef USER_DRIVERS_DRV_CAN_MOTOR_H
#define USER_DRIVERS_DRV_CAN_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "drv_m3508.h"

typedef struct
{
    uint32_t tx_count;
    uint32_t tx_fail_count;
    uint32_t rx_frame_count;
    uint32_t rx_ignored_count;
    uint32_t rx_error_count;
    uint32_t last_std_id;
    uint8_t bus_off;
    uint8_t error_passive;
    uint8_t warning;
    uint8_t tec;
    uint8_t rec;
} drv_can_motor_stats_t;

/* 初始化 CAN 过滤器、启动 CAN、打开接收通知。 */
int drv_can_motor_init(void);

/* 取走 ISR 缓存的最新电机反馈掩码和数据。 */
uint32_t drv_can_motor_fetch_pending(drv_m3508_feedback_t feedback_out[4]);

/* 发送四个底盘轮的电流给定。 */
bool drv_can_motor_send_chassis_currents(int16_t i1, int16_t i2, int16_t i3, int16_t i4);
void drv_can_motor_get_stats(drv_can_motor_stats_t *stats_out);

#endif /* USER_DRIVERS_DRV_CAN_MOTOR_H */
