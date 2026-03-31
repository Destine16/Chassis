#include "drv_nav_proto.h"

#include <string.h>

#include "robot_def.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

typedef struct
{
    uint8_t frame[APP_CFG_NAV_TX_BUFFER_SIZE];
    uint16_t length;
} nav_tx_slot_t;

static nav_tx_slot_t g_nav_tx_queue[APP_CFG_NAV_TX_QUEUE_DEPTH];
static uint8_t g_nav_tx_active[APP_CFG_NAV_TX_BUFFER_SIZE];
static volatile uint32_t g_nav_tx_drop_count = 0U;
static volatile uint32_t g_nav_tx_queue_full_count = 0U;
static volatile uint32_t g_nav_tx_start_fail_count = 0U;
static volatile uint32_t g_nav_tx_enqueue_count = 0U;
static volatile uint32_t g_nav_tx_dequeue_count = 0U;
static uint8_t g_nav_tx_seq = 0U;
static volatile uint8_t g_nav_tx_head = 0U;
static volatile uint8_t g_nav_tx_tail = 0U;
static volatile uint8_t g_nav_tx_count = 0U;
static volatile uint8_t g_nav_tx_queue_depth_peak = 0U;
static volatile uint8_t g_nav_tx_busy = 0U;

extern USBD_HandleTypeDef hUsbDeviceFS;

static bool drv_nav_proto_usb_ready(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) && (hUsbDeviceFS.pClassData != NULL);
}

static void drv_nav_proto_reset_queue(void)
{
    __disable_irq();
    g_nav_tx_head = 0U;
    g_nav_tx_tail = 0U;
    g_nav_tx_count = 0U;
    g_nav_tx_busy = 0U;
    __enable_irq();
}

static bool drv_nav_proto_try_start_tx(void)
{
    uint16_t length = 0U;
    bool have_frame = false;

    if (!drv_nav_proto_usb_ready())
    {
        drv_nav_proto_reset_queue();
        return false;
    }

    __disable_irq();
    if ((g_nav_tx_busy == 0U) && (g_nav_tx_count > 0U))
    {
        uint8_t tail = g_nav_tx_tail;

        length = g_nav_tx_queue[tail].length;
        memcpy(g_nav_tx_active, g_nav_tx_queue[tail].frame, length);
        g_nav_tx_tail = (uint8_t)((tail + 1U) % APP_CFG_NAV_TX_QUEUE_DEPTH);
        g_nav_tx_count--;
        g_nav_tx_dequeue_count++;
        g_nav_tx_busy = 1U;
        have_frame = true;
    }
    __enable_irq();

    if (!have_frame)
        return false;

    if (CDC_Transmit_FS(g_nav_tx_active, length) != USBD_OK)
    {
        __disable_irq();
        g_nav_tx_busy = 0U;
        g_nav_tx_drop_count++;
        g_nav_tx_start_fail_count++;
        __enable_irq();
        return false;
    }

    return true;
}

int drv_nav_proto_init(void)
{
    g_nav_tx_seq = 0U;
    g_nav_tx_drop_count = 0U;
    g_nav_tx_queue_full_count = 0U;
    g_nav_tx_start_fail_count = 0U;
    g_nav_tx_enqueue_count = 0U;
    g_nav_tx_dequeue_count = 0U;
    g_nav_tx_queue_depth_peak = 0U;
    drv_nav_proto_reset_queue();
    return 0;
}

bool drv_nav_proto_send_observation(const protocol_nav_observation_payload_t *payload)
{
    uint8_t frame[APP_CFG_NAV_TX_BUFFER_SIZE];
    uint16_t length;

    if (!drv_nav_proto_usb_ready())
    {
        drv_nav_proto_reset_queue();
        return false;
    }

    length = protocol_nav_encode_observation(g_nav_tx_seq++, payload, frame, sizeof(frame));
    if (length == 0U)
        return false;

    __disable_irq();
    if (g_nav_tx_count >= APP_CFG_NAV_TX_QUEUE_DEPTH)
    {
        g_nav_tx_tail = (uint8_t)((g_nav_tx_tail + 1U) % APP_CFG_NAV_TX_QUEUE_DEPTH);
        g_nav_tx_count--;
        g_nav_tx_drop_count++;
        g_nav_tx_queue_full_count++;
    }

    memcpy(g_nav_tx_queue[g_nav_tx_head].frame, frame, length);
    g_nav_tx_queue[g_nav_tx_head].length = length;
    g_nav_tx_head = (uint8_t)((g_nav_tx_head + 1U) % APP_CFG_NAV_TX_QUEUE_DEPTH);
    g_nav_tx_count++;
    g_nav_tx_enqueue_count++;
    if (g_nav_tx_count > g_nav_tx_queue_depth_peak)
        g_nav_tx_queue_depth_peak = g_nav_tx_count;
    __enable_irq();

    (void)drv_nav_proto_try_start_tx();
    return true;
}

void drv_nav_proto_on_tx_complete(void)
{
    __disable_irq();
    g_nav_tx_busy = 0U;
    __enable_irq();

    (void)drv_nav_proto_try_start_tx();
}

void drv_nav_proto_get_stats(drv_nav_proto_stats_t *stats_out)
{
    if (stats_out == NULL)
        return;

    __disable_irq();
    stats_out->drop_count = g_nav_tx_drop_count;
    stats_out->queue_full_count = g_nav_tx_queue_full_count;
    stats_out->tx_start_fail_count = g_nav_tx_start_fail_count;
    stats_out->enqueue_count = g_nav_tx_enqueue_count;
    stats_out->dequeue_count = g_nav_tx_dequeue_count;
    stats_out->queue_count = g_nav_tx_count;
    stats_out->queue_depth_peak = g_nav_tx_queue_depth_peak;
    stats_out->busy = g_nav_tx_busy;
    stats_out->seq = g_nav_tx_seq;
    __enable_irq();
}
