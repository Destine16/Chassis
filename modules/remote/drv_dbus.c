#include "drv_dbus.h"

#include <string.h>

#include "robot.h"
#include "robot_def.h"
#include "bsp_uart.h"

#define DRV_DBUS_BYTE_RING_SIZE    (128U)
#define DRV_DBUS_STREAM_BUFFER_LEN (APP_CFG_DBUS_FRAME_LEN * 4U)

static UART_HandleTypeDef *g_dbus_huart;
static drv_dbus_stats_t g_dbus_stats;
static uint8_t g_dbus_byte_ring[DRV_DBUS_BYTE_RING_SIZE];
static volatile uint16_t g_dbus_ring_head;
static volatile uint16_t g_dbus_ring_tail;
static volatile uint16_t g_dbus_ring_count;
static volatile uint32_t g_dbus_rx_byte_count;
static uint8_t g_dbus_stream_buffer[DRV_DBUS_STREAM_BUFFER_LEN];
static uint16_t g_dbus_stream_length;

/* DBUS 里鼠标和键盘字段使用小端序。 */
static uint16_t drv_dbus_read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static bool drv_dbus_channel_is_valid(uint16_t raw)
{
    return (raw >= APP_CFG_RC_VALUE_MIN) && (raw <= APP_CFG_RC_VALUE_MAX);
}

static bool drv_dbus_switch_is_valid(uint8_t value)
{
    return (value == RC_SWITCH_UP) || (value == RC_SWITCH_MID) || (value == RC_SWITCH_DOWN);
}

static void drv_dbus_ring_push_from_isr(uint8_t byte)
{
    if (g_dbus_ring_count >= DRV_DBUS_BYTE_RING_SIZE)
    {
        g_dbus_ring_tail = (uint16_t)((g_dbus_ring_tail + 1U) % DRV_DBUS_BYTE_RING_SIZE);
        g_dbus_ring_count--;
    }

    g_dbus_byte_ring[g_dbus_ring_head] = byte;
    g_dbus_ring_head = (uint16_t)((g_dbus_ring_head + 1U) % DRV_DBUS_BYTE_RING_SIZE);
    g_dbus_ring_count++;
    g_dbus_rx_byte_count++;
}

static bool drv_dbus_ring_pop(uint8_t *byte_out)
{
    bool has_data = false;

    __disable_irq();
    if (g_dbus_ring_count > 0U)
    {
        *byte_out = g_dbus_byte_ring[g_dbus_ring_tail];
        g_dbus_ring_tail = (uint16_t)((g_dbus_ring_tail + 1U) % DRV_DBUS_BYTE_RING_SIZE);
        g_dbus_ring_count--;
        has_data = true;
    }
    __enable_irq();

    return has_data;
}

static bool drv_dbus_decode_frame(const uint8_t *data, uint32_t sequence, drv_dbus_frame_t *frame_out)
{
    uint32_t index = 0U;

    frame_out->ch[0] = (uint16_t)((data[0] | (data[1] << 8U)) & 0x07FFU);
    frame_out->ch[1] = (uint16_t)(((data[1] >> 3U) | (data[2] << 5U)) & 0x07FFU);
    frame_out->ch[2] = (uint16_t)(((data[2] >> 6U) | (data[3] << 2U) | (data[4] << 10U)) & 0x07FFU);
    frame_out->ch[3] = (uint16_t)(((data[4] >> 1U) | (data[5] << 7U)) & 0x07FFU);
    frame_out->s1 = (uint8_t)(((data[5] >> 4U) & 0x0CU) >> 2U);
    frame_out->s2 = (uint8_t)((data[5] >> 4U) & 0x03U);

    frame_out->mouse_x = (int16_t)drv_dbus_read_u16_le(&data[6]);
    frame_out->mouse_y = (int16_t)drv_dbus_read_u16_le(&data[8]);
    frame_out->mouse_z = (int16_t)drv_dbus_read_u16_le(&data[10]);
    frame_out->mouse_press_l = data[12];
    frame_out->mouse_press_r = data[13];
    frame_out->key_bits = drv_dbus_read_u16_le(&data[14]);
    frame_out->sequence = sequence;

    while (index < 4U)
    {
        if (!drv_dbus_channel_is_valid(frame_out->ch[index]))
        {
            g_dbus_stats.invalid_channel_count++;
            return false;
        }
        index++;
    }

    if (!drv_dbus_switch_is_valid(frame_out->s1) || !drv_dbus_switch_is_valid(frame_out->s2))
    {
        g_dbus_stats.invalid_switch_count++;
        return false;
    }

    return true;
}

int drv_dbus_init(void)
{
    UART_HandleTypeDef *huart;

    memset(&g_dbus_stats, 0, sizeof(g_dbus_stats));
    g_dbus_stream_length = 0U;
    g_dbus_ring_head = 0U;
    g_dbus_ring_tail = 0U;
    g_dbus_ring_count = 0U;
    g_dbus_rx_byte_count = 0U;

    huart = bsp_uart_get_handle(BSP_UART_PORT_RC);
    if (huart == NULL)
        return -1;

    g_dbus_huart = huart;

    /* DBUS 改成纯字节流接收：不用 DMA 分片，不依赖 IDLE 分帧。 */
    ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    if (huart->hdmarx != NULL)
        (void)HAL_DMA_Abort(huart->hdmarx);

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;

    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE | USART_CR1_IDLEIE);
    ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
    if (huart->Init.Parity != UART_PARITY_NONE)
        ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

    return 0;
}

void drv_dbus_irq_handler(void)
{
    BaseType_t higher_priority_woken = pdFALSE;
    uint32_t sr;
    uint8_t byte = 0U;
    bool notify_task = false;

    if (g_dbus_huart == NULL)
        return;

    sr = g_dbus_huart->Instance->SR;

    if ((sr & USART_SR_RXNE) != 0U)
    {
        byte = (uint8_t)(g_dbus_huart->Instance->DR & 0x00FFU);
        drv_dbus_ring_push_from_isr(byte);
    }

    if ((sr & USART_SR_IDLE) != 0U)
    {
        __HAL_UART_CLEAR_IDLEFLAG(g_dbus_huart);
        notify_task = true;
    }

    if ((sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0U)
    {
        __HAL_UART_CLEAR_PEFLAG(g_dbus_huart);
        notify_task = true;
    }

    if (notify_task && (robot_get_rc_task() != NULL))
        xTaskNotifyFromISR(robot_get_rc_task(), ROBOT_NOTIFY_RC_FRAME, eSetBits, &higher_priority_woken);

    portYIELD_FROM_ISR(higher_priority_woken);
}

bool drv_dbus_read_latest(drv_dbus_frame_t *frame_out)
{
    uint8_t byte = 0U;

    for (;;)
    {
        while (g_dbus_stream_length >= APP_CFG_DBUS_FRAME_LEN)
        {
            uint32_t next_sequence = g_dbus_stats.valid_frame_count + 1U;

            if (drv_dbus_decode_frame(g_dbus_stream_buffer, next_sequence, frame_out))
            {
                g_dbus_stats.valid_frame_count++;
                g_dbus_stats.last_sequence = frame_out->sequence;
                memmove(g_dbus_stream_buffer,
                        &g_dbus_stream_buffer[APP_CFG_DBUS_FRAME_LEN],
                        (size_t)(g_dbus_stream_length - APP_CFG_DBUS_FRAME_LEN));
                g_dbus_stream_length = (uint16_t)(g_dbus_stream_length - APP_CFG_DBUS_FRAME_LEN);
                return true;
            }

            memmove(g_dbus_stream_buffer,
                    &g_dbus_stream_buffer[1],
                    (size_t)(g_dbus_stream_length - 1U));
            g_dbus_stream_length--;
        }

        if (!drv_dbus_ring_pop(&byte))
            break;

        if (g_dbus_stream_length >= sizeof(g_dbus_stream_buffer))
            g_dbus_stream_length = 0U;

        if (g_dbus_stream_length >= DRV_DBUS_STREAM_BUFFER_LEN)
            g_dbus_stream_length = 0U;

        g_dbus_stream_buffer[g_dbus_stream_length] = byte;
        g_dbus_stream_length++;
    }

    return false;
}

void drv_dbus_get_stats(drv_dbus_stats_t *stats_out)
{
    if (stats_out == NULL)
        return;

    *stats_out = g_dbus_stats;
}
