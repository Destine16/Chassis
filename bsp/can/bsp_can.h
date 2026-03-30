#ifndef USER_BSP_BSP_CAN_H
#define USER_BSP_BSP_CAN_H

#include "stm32f4xx_hal.h"

/* 当前工程只有一路底盘 CAN。 */
typedef enum
{
    BSP_CAN_PORT_CHASSIS = 0
} bsp_can_port_t;

/* 取得指定逻辑 CAN 口对应的 HAL 句柄。 */
void bsp_can_init(void);
CAN_HandleTypeDef *bsp_can_get_handle(bsp_can_port_t port);

/* 第一版直接配置“全接收”滤波，后续可按报文类型细分。 */
HAL_StatusTypeDef bsp_can_configure_accept_all(CAN_HandleTypeDef *hcan, uint32_t filter_bank);
HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef bsp_can_enable_rx_notification(CAN_HandleTypeDef *hcan, uint32_t active_interrupts);

/* 发送标准帧数据。 */
HAL_StatusTypeDef bsp_can_send_std(CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t dlc);

#endif /* USER_BSP_BSP_CAN_H */
