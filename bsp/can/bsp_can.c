#include "bsp_can.h"

#include "can.h"

CAN_HandleTypeDef *bsp_can_get_handle(bsp_can_port_t port)
{
    /* 当前仅有 CAN1，因此逻辑端口直接映射到 hcan1。 */
    (void)port;
    return &hcan1;
}

HAL_StatusTypeDef bsp_can_configure_accept_all(CAN_HandleTypeDef *hcan, uint32_t filter_bank)
{
    CAN_FilterTypeDef filter;

    /* 先全开接收，方便第一版同时接电机反馈和后续调试报文。 */
    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0U;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0U;
    filter.FilterMaskIdLow = 0U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

HAL_StatusTypeDef bsp_can_start(CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_Start(hcan);
}

HAL_StatusTypeDef bsp_can_enable_rx_notification(CAN_HandleTypeDef *hcan, uint32_t active_interrupts)
{
    return HAL_CAN_ActivateNotification(hcan, active_interrupts);
}

HAL_StatusTypeDef bsp_can_send_std(CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox = 0U;

    /* 底盘电机发送均使用标准数据帧。 */
    header.StdId = std_id;
    header.ExtId = 0U;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = dlc;
    header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox);
}
