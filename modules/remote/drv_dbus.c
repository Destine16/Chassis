#include "drv_dbus.h"

#include "robot.h"
#include "robot_def.h"
#include "bsp_uart.h"
#include "drv_uart_idle_dma.h"

static drv_uart_idle_dma_t g_dbus_uart;
static uint8_t g_dbus_rx_buffer0[APP_CFG_DBUS_FRAME_LEN];
static uint8_t g_dbus_rx_buffer1[APP_CFG_DBUS_FRAME_LEN];

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

int drv_dbus_init(void)
{
    /* RC 必须使用 DMA + 双缓冲 + IDLE，中断只通知 rcTask。 */
    drv_uart_idle_dma_init(&g_dbus_uart,
                           bsp_uart_get_handle(BSP_UART_PORT_RC),
                           bsp_uart_get_rx_dma_handle(BSP_UART_PORT_RC),
                           DRV_UART_IDLE_DMA_MODE_DOUBLE,
                           g_dbus_rx_buffer0,
                           g_dbus_rx_buffer1,
                           APP_CFG_DBUS_FRAME_LEN,
                           robot_get_rc_task(),
                           ROBOT_NOTIFY_RC_FRAME);

    return (drv_uart_idle_dma_start(&g_dbus_uart) == HAL_OK) ? 0 : -1;
}

void drv_dbus_irq_handler(void)
{
    drv_uart_idle_dma_irq_handler(&g_dbus_uart);
}

bool drv_dbus_read_latest(drv_dbus_frame_t *frame_out)
{
    const uint8_t *data = NULL;
    uint16_t length = 0U;
    uint32_t sequence = 0U;
    uint32_t index = 0U;

    if (!drv_uart_idle_dma_take_latest(&g_dbus_uart, &data, &length, &sequence))
        return false;

    if (length != APP_CFG_DBUS_FRAME_LEN)
        return false;

    /*
     * 按 DR16 常见 DBUS 位域格式解包：
     * - ch0/ch1 = 右摇杆横/纵
     * - ch2/ch3 = 左摇杆横/纵
     * - s1/s2 = 左/右三段拨杆
     */
    frame_out->ch[0] = (uint16_t)((data[0] | (data[1] << 8U)) & 0x07FFU);
    frame_out->ch[1] = (uint16_t)(((data[1] >> 3U) | (data[2] << 5U)) & 0x07FFU);
    frame_out->ch[2] = (uint16_t)(((data[2] >> 6U) | (data[3] << 2U) | (data[4] << 10U)) & 0x07FFU);
    frame_out->ch[3] = (uint16_t)(((data[4] >> 1U) | (data[5] << 7U)) & 0x07FFU);
    frame_out->s1 = (uint8_t)(((data[5] >> 4U) & 0x0CU) >> 2U);
    frame_out->s2 = (uint8_t)((data[5] >> 4U) & 0x03U);

    /*
     * 鼠标和键盘字段当前只做底层解包缓存，不参与底盘控制。
     * 如果后续要加键鼠控，只需要在 srv_rc 层继续向上透传即可。
     */
    frame_out->mouse_x = (int16_t)drv_dbus_read_u16_le(&data[6]);
    frame_out->mouse_y = (int16_t)drv_dbus_read_u16_le(&data[8]);
    frame_out->mouse_z = (int16_t)drv_dbus_read_u16_le(&data[10]);
    frame_out->mouse_press_l = data[12];
    frame_out->mouse_press_r = data[13];
    frame_out->key_bits = drv_dbus_read_u16_le(&data[14]);
    frame_out->sequence = sequence;

    /*
     * DR16 原始帧再做一次基础合法性检查：
     * - 四个摇杆通道必须落在常见有效范围 364..1684
     * - 两个三段拨杆必须是 1/2/3 中的合法编码
     *
     * 这样串口偶发错帧时，不会把异常值继续上推到 srv_rc。
     */
    while (index < 4U)
    {
        if (!drv_dbus_channel_is_valid(frame_out->ch[index]))
            return false;
        index++;
    }

    if (!drv_dbus_switch_is_valid(frame_out->s1) || !drv_dbus_switch_is_valid(frame_out->s2))
        return false;

    return true;
}
