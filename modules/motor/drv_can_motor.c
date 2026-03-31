#include "drv_can_motor.h"

#include <string.h>

#include "robot_def.h"
#include "bsp_can.h"
#include "can.h"

#define DRV_CAN_MOTOR_TX_OFFSET_FROM_ESC_ID(id)    ((uint8_t)(((id) - 1U) * 2U))

typedef struct
{
    /* 统一维护“轮位 <-> ESC ID <-> 反馈 ID <-> 0x200 字节槽位”的映射。 */
    uint8_t esc_id;
    uint16_t feedback_id;
    uint8_t tx_payload_offset;
} drv_can_motor_map_t;

static const drv_can_motor_map_t k_motor_map[4] = {
    { APP_CFG_CAN_ESC_ID_FRONT_LEFT,  APP_CFG_CAN_MOTOR1_FB_ID, DRV_CAN_MOTOR_TX_OFFSET_FROM_ESC_ID(APP_CFG_CAN_ESC_ID_FRONT_LEFT) },
    { APP_CFG_CAN_ESC_ID_FRONT_RIGHT, APP_CFG_CAN_MOTOR2_FB_ID, DRV_CAN_MOTOR_TX_OFFSET_FROM_ESC_ID(APP_CFG_CAN_ESC_ID_FRONT_RIGHT) },
    { APP_CFG_CAN_ESC_ID_REAR_LEFT,   APP_CFG_CAN_MOTOR3_FB_ID, DRV_CAN_MOTOR_TX_OFFSET_FROM_ESC_ID(APP_CFG_CAN_ESC_ID_REAR_LEFT) },
    { APP_CFG_CAN_ESC_ID_REAR_RIGHT,  APP_CFG_CAN_MOTOR4_FB_ID, DRV_CAN_MOTOR_TX_OFFSET_FROM_ESC_ID(APP_CFG_CAN_ESC_ID_REAR_RIGHT) }
};

static drv_m3508_feedback_t g_pending_feedback[4];
static volatile uint32_t g_pending_mask = 0U;
static drv_can_motor_stats_t g_can_motor_stats;

static int drv_can_motor_find_index(uint16_t std_id)
{
    uint32_t index = 0U;

    /* 将标准帧 ID 映射到统一轮序下标。 */
    while (index < 4U)
    {
        if (k_motor_map[index].feedback_id == std_id)
            return (int)index;
        index++;
    }

    return -1;
}

int drv_can_motor_init(void)
{
    CAN_HandleTypeDef *hcan = bsp_can_get_handle(BSP_CAN_PORT_CHASSIS);

    memset(&g_can_motor_stats, 0, sizeof(g_can_motor_stats));

    /* 第一版统一由这里完成 CAN 的运行时启动动作。 */
    if (bsp_can_configure_accept_all(hcan, 0U) != HAL_OK)
        return -1;
    if (bsp_can_start(hcan) != HAL_OK)
        return -1;
    if (bsp_can_enable_rx_notification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        return -1;

    return 0;
}

uint32_t drv_can_motor_fetch_pending(drv_m3508_feedback_t feedback_out[4])
{
    uint32_t mask;

    /* 任务侧一次性拿走 ISR 缓存的快照，随后清空待处理标志。 */
    __disable_irq();
    mask = g_pending_mask;
    if (mask != 0U)
    {
        memcpy(feedback_out, g_pending_feedback, sizeof(g_pending_feedback));
        g_pending_mask = 0U;
    }
    __enable_irq();

    return mask;
}

bool drv_can_motor_send_chassis_currents(int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
    const int16_t current_cmd[COMMON_WHEEL_COUNT] = { i1, i2, i3, i4 };
    uint8_t payload[8] = {0};
    uint32_t index = 0U;

    /*
     * 按 C620 官方协议组帧：
     * - 标准帧 0x200 的 8 字节依次对应 ESC ID 1~4
     * - 每个 ESC 占 2 字节有符号电流给定
     */
    while (index < COMMON_WHEEL_COUNT)
    {
        uint8_t offset = k_motor_map[index].tx_payload_offset;
        int16_t value = current_cmd[index];

        payload[offset] = (uint8_t)((value >> 8U) & 0xFFU);
        payload[offset + 1U] = (uint8_t)(value & 0xFFU);
        index++;
    }

    if (bsp_can_send_std(bsp_can_get_handle(BSP_CAN_PORT_CHASSIS), APP_CFG_CAN_CHASSIS_TX_ID, payload, 8U) == HAL_OK)
    {
        g_can_motor_stats.tx_count++;
        return true;
    }

    g_can_motor_stats.tx_fail_count++;
    return false;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    /* 用户层只接管 CAN1 FIFO0 的底盘电机反馈。 */
    if (hcan != &hcan1)
        return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        int index;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            g_can_motor_stats.rx_error_count++;
            break;
        }

        if ((header.IDE != CAN_ID_STD) || (header.DLC < 7U))
        {
            g_can_motor_stats.rx_ignored_count++;
            continue;
        }

        g_can_motor_stats.rx_frame_count++;
        g_can_motor_stats.last_std_id = header.StdId;

        index = drv_can_motor_find_index((uint16_t)header.StdId);
        if (index < 0)
        {
            g_can_motor_stats.rx_ignored_count++;
            continue;
        }

        /* ISR 里只做轻量解析和缓存，不直接更新业务层状态。 */
        drv_m3508_parse_feedback(data, &g_pending_feedback[index]);
        g_pending_mask |= (1UL << (uint32_t)index);
    }
}

void drv_can_motor_get_stats(drv_can_motor_stats_t *stats_out)
{
    uint32_t esr;

    if (stats_out == NULL)
        return;

    *stats_out = g_can_motor_stats;
    esr = hcan1.Instance->ESR;
    stats_out->bus_off = ((esr & CAN_ESR_BOFF) != 0U) ? 1U : 0U;
    stats_out->error_passive = ((esr & CAN_ESR_EPVF) != 0U) ? 1U : 0U;
    stats_out->warning = ((esr & CAN_ESR_EWGF) != 0U) ? 1U : 0U;
    stats_out->tec = (uint8_t)((esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);
    stats_out->rec = (uint8_t)((esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos);
}
