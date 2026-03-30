#include "bsp_uart.h"

#include "usart.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

UART_HandleTypeDef *bsp_uart_get_handle(bsp_uart_port_t port)
{
    (void)port;
    return &huart3;
}

DMA_HandleTypeDef *bsp_uart_get_rx_dma_handle(bsp_uart_port_t port)
{
    (void)port;
    return &hdma_usart3_rx;
}

HAL_StatusTypeDef bsp_uart_transmit(bsp_uart_port_t port, const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    (void)port;
    return HAL_UART_Transmit(bsp_uart_get_handle(port), (uint8_t *)data, length, timeout_ms);
}
