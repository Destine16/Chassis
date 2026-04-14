#include "drv_bmi088_internal.h"

#include <string.h>

#include "bsp_time.h"
#include "main.h"

bmi088_context_t g_bmi088_ctx;

int drv_bmi088_init(void)
{
    memset(&g_bmi088_ctx, 0, sizeof(g_bmi088_ctx));
    g_bmi088_ctx.last_init_attempt_ms = bsp_time_get_ms();
    g_bmi088_ctx.data.error_flags = bmi088_try_init();
    g_bmi088_ctx.ready = (g_bmi088_ctx.data.error_flags == DRV_BMI088_ERR_NONE);
    if (g_bmi088_ctx.ready)
        bmi088_async_enable();
    return g_bmi088_ctx.ready ? 0 : -1;
}

void drv_bmi088_poll(void)
{
    uint32_t now_ms = bsp_time_get_ms();

    if (!g_bmi088_ctx.ready)
    {
        if (bsp_time_is_expired(now_ms, g_bmi088_ctx.last_init_attempt_ms, BMI088_INIT_RETRY_MS))
        {
            g_bmi088_ctx.last_init_attempt_ms = now_ms;
            g_bmi088_ctx.data.error_flags = bmi088_try_init();
            g_bmi088_ctx.ready = (g_bmi088_ctx.data.error_flags == DRV_BMI088_ERR_NONE);
            if (g_bmi088_ctx.ready)
                bmi088_async_enable();
        }
        return;
    }

    if ((g_bmi088_ctx.async.state != BMI088_ASYNC_IDLE) &&
        bsp_time_is_expired(now_ms, g_bmi088_ctx.last_transfer_start_ms, BMI088_DMA_TIMEOUT_MS))
    {
        g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_SPI;
        bmi088_async_reset();
        bmi088_async_enable();
    }

    if (bsp_time_is_expired(now_ms, g_bmi088_ctx.last_async_update_ms, BMI088_ASYNC_FALLBACK_MS))
    {
        g_bmi088_ctx.async.pending_accel = 1U;
        g_bmi088_ctx.async.pending_gyro = 1U;
    }

    bmi088_service_pending_transfer();

    if (bsp_time_is_expired(now_ms, g_bmi088_ctx.last_async_update_ms, BMI088_OFFLINE_TIMEOUT_MS))
    {
        g_bmi088_ctx.ready = false;
        g_bmi088_ctx.data.online = false;
        g_bmi088_ctx.data.valid = false;
        g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_SPI;
        bmi088_async_reset();
        return;
    }
}

bool drv_bmi088_get_data(bmi088_data_t *out)
{
    uint32_t primask;

    if (out == NULL)
        return false;

    primask = __get_PRIMASK();
    __disable_irq();
    *out = g_bmi088_ctx.data;
    if (!primask)
        __enable_irq();
    return out->valid;
}

bool drv_bmi088_is_online(void)
{
    return g_bmi088_ctx.data.online;
}

uint32_t drv_bmi088_get_error_flags(void)
{
    return g_bmi088_ctx.data.error_flags;
}
