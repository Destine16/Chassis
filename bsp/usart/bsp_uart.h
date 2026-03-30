#ifndef USER_BSP_BSP_UART_H
#define USER_BSP_BSP_UART_H

#include "stm32f4xx_hal.h"

/* 逻辑串口端口号，当前仅保留 RC 接收链路。 */
typedef enum
{
    BSP_UART_PORT_RC = 0
} bsp_uart_port_t;

/* 获取逻辑串口对应的 HAL 句柄和接收 DMA 句柄。 */
UART_HandleTypeDef *bsp_uart_get_handle(bsp_uart_port_t port);
DMA_HandleTypeDef *bsp_uart_get_rx_dma_handle(bsp_uart_port_t port);

/* 阻塞发送接口，当前主要用于遥测回传。 */
HAL_StatusTypeDef bsp_uart_transmit(bsp_uart_port_t port, const uint8_t *data, uint16_t length, uint32_t timeout_ms);

#endif /* USER_BSP_BSP_UART_H */
