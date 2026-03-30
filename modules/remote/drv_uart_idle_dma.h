#ifndef USER_DRIVERS_DRV_UART_IDLE_DMA_H
#define USER_DRIVERS_DRV_UART_IDLE_DMA_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

/* 串口接收模式：导航先单缓冲，DBUS 使用双缓冲。 */
typedef enum
{
    DRV_UART_IDLE_DMA_MODE_SINGLE = 0,
    DRV_UART_IDLE_DMA_MODE_DOUBLE = 1
} drv_uart_idle_dma_mode_t;

/*
 * 通用 UART + DMA + IDLE 接收上下文。
 * ISR 只负责记录“哪块缓冲区收到了一帧、长度是多少”，解析放到任务里做。
 */
typedef struct
{
    UART_HandleTypeDef *huart;            /* 绑定的 UART 句柄，例如 huart1 或 huart3。 */
    DMA_HandleTypeDef *hdma;              /* 对应 UART RX 的 DMA 句柄。 */
    drv_uart_idle_dma_mode_t mode;        /* 当前接收模式：单缓冲或双缓冲。 */

    uint8_t *buffer0;                     /* 第 0 块接收缓冲区地址。 */
    uint8_t *buffer1;                     /* 第 1 块接收缓冲区地址，单缓冲模式下可为 NULL。 */
    uint16_t buffer_size;                 /* 每块缓冲区长度，单位为字节。 */

    TaskHandle_t notify_task;             /* 收到一帧后需要唤醒的任务句柄。 */
    uint32_t notify_value;                /* 唤醒任务时写入的通知位，例如 ROBOT_NOTIFY_RC_FRAME。 */

    volatile uint8_t latest_buffer_index; /* 最新完整帧位于哪块缓冲区，0 表示 buffer0，1 表示 buffer1。 */
    volatile uint16_t latest_length;      /* 最新完整帧的有效长度，单位为字节。 */
    volatile uint32_t latest_sequence;    /* 最新帧的递增序号，每成功登记一帧就加 1。 */
    volatile bool frame_ready;            /* 是否已有一帧等待任务取走。true 表示有新帧未消费。 */
    volatile bool paused;                 /* 单缓冲模式下 DMA 是否已暂停，双缓冲模式通常保持 false。 */
} drv_uart_idle_dma_t;

/* 初始化接收上下文，不启动 DMA。 */
void drv_uart_idle_dma_init(drv_uart_idle_dma_t *ctx,
                            UART_HandleTypeDef *huart,
                            DMA_HandleTypeDef *hdma,
                            drv_uart_idle_dma_mode_t mode,
                            uint8_t *buffer0,
                            uint8_t *buffer1,
                            uint16_t buffer_size,
                            TaskHandle_t notify_task,
                            uint32_t notify_value);

/* 启动或重启 DMA + IDLE 接收。 */
HAL_StatusTypeDef drv_uart_idle_dma_start(drv_uart_idle_dma_t *ctx);
HAL_StatusTypeDef drv_uart_idle_dma_restart(drv_uart_idle_dma_t *ctx);

/* 在 USARTx_IRQHandler 的 USER CODE 区里调用。 */
void drv_uart_idle_dma_irq_handler(drv_uart_idle_dma_t *ctx);

/* 任务侧取走最新一帧，不保留历史队列。 */
bool drv_uart_idle_dma_take_latest(drv_uart_idle_dma_t *ctx, const uint8_t **data, uint16_t *length, uint32_t *sequence);

#endif /* USER_DRIVERS_DRV_UART_IDLE_DMA_H */
