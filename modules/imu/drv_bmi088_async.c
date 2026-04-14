#include "drv_bmi088_internal.h"

#include <string.h>

#include "bsp_time.h"
#include "bmi088_reg.h"
#include "main.h"
#include "spi.h"

static void bmi088_accel_cs_low(void)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_RESET);
}

static void bmi088_accel_cs_high(void)
{
    HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, GPIO_PIN_SET);
}

static void bmi088_gyro_cs_low(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET);
}

static void bmi088_gyro_cs_high(void)
{
    HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

static bool bmi088_start_gyro_dma_transfer(void)
{
    if ((g_bmi088_ctx.async.enabled == 0U) || (g_bmi088_ctx.async.state != BMI088_ASYNC_IDLE))
        return false;
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        g_bmi088_ctx.async.pending_gyro = 1U;
        return false;
    }

    g_bmi088_ctx.async.tx_buf[0] = BMI088_GYRO_X_L | 0x80U;
    memset(&g_bmi088_ctx.async.tx_buf[1], 0x55, BMI088_GYRO_DMA_FRAME_LEN - 1U);
    bmi088_gyro_cs_low();
    g_bmi088_ctx.async.state = BMI088_ASYNC_GYRO_BUSY;
    g_bmi088_ctx.last_transfer_start_ms = bsp_time_get_ms();

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, g_bmi088_ctx.async.tx_buf, g_bmi088_ctx.async.rx_buf, BMI088_GYRO_DMA_FRAME_LEN) != HAL_OK)
    {
        bmi088_gyro_cs_high();
        g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_SPI;
        return false;
    }

    return true;
}

static bool bmi088_start_accel_dma_transfer(void)
{
    if ((g_bmi088_ctx.async.enabled == 0U) || (g_bmi088_ctx.async.state != BMI088_ASYNC_IDLE))
        return false;
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        g_bmi088_ctx.async.pending_accel = 1U;
        return false;
    }

    g_bmi088_ctx.async.tx_buf[0] = BMI088_ACCEL_XOUT_L | 0x80U;
    memset(&g_bmi088_ctx.async.tx_buf[1], 0x55, BMI088_ACCEL_DMA_FRAME_LEN - 1U);
    bmi088_accel_cs_low();
    g_bmi088_ctx.async.state = BMI088_ASYNC_ACCEL_BUSY;
    g_bmi088_ctx.last_transfer_start_ms = bsp_time_get_ms();

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, g_bmi088_ctx.async.tx_buf, g_bmi088_ctx.async.rx_buf, BMI088_ACCEL_DMA_FRAME_LEN) != HAL_OK)
    {
        bmi088_accel_cs_high();
        g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_SPI;
        return false;
    }

    return true;
}

void bmi088_async_reset(void)
{
    if (hspi1.State != HAL_SPI_STATE_READY)
        (void)HAL_SPI_Abort(&hspi1);

    g_bmi088_ctx.async.enabled = 0U;
    g_bmi088_ctx.async.pending_accel = 0U;
    g_bmi088_ctx.async.pending_gyro = 0U;
    g_bmi088_ctx.async.prefer_accel = 0U;
    g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    bmi088_accel_cs_high();
    bmi088_gyro_cs_high();
}

void bmi088_async_enable(void)
{
    uint32_t now_ms = bsp_time_get_ms();

    g_bmi088_ctx.async.enabled = 1U;
    g_bmi088_ctx.async.pending_accel = 1U;
    g_bmi088_ctx.async.pending_gyro = 1U;
    g_bmi088_ctx.async.prefer_accel = 0U;
    g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    g_bmi088_ctx.last_async_update_ms = now_ms;
    g_bmi088_ctx.last_transfer_start_ms = now_ms;
}

void bmi088_service_pending_transfer(void)
{
    if ((g_bmi088_ctx.async.enabled == 0U) || (g_bmi088_ctx.async.state != BMI088_ASYNC_IDLE))
        return;

    if ((g_bmi088_ctx.async.pending_accel != 0U) &&
        (g_bmi088_ctx.async.pending_gyro != 0U))
    {
        if (g_bmi088_ctx.async.prefer_accel != 0U)
        {
            g_bmi088_ctx.async.pending_accel = 0U;
            if (bmi088_start_accel_dma_transfer())
                return;
        }
        else
        {
            g_bmi088_ctx.async.pending_gyro = 0U;
            if (bmi088_start_gyro_dma_transfer())
                return;
        }
    }

    if (g_bmi088_ctx.async.pending_gyro != 0U)
    {
        g_bmi088_ctx.async.pending_gyro = 0U;
        if (bmi088_start_gyro_dma_transfer())
            return;
    }

    if (g_bmi088_ctx.async.pending_accel != 0U)
    {
        g_bmi088_ctx.async.pending_accel = 0U;
        (void)bmi088_start_accel_dma_transfer();
    }
}

void bmi088_parse_gyro_frame(const uint8_t *rx_buf)
{
    int16_t raw;
    float gx;
    float gy;
    float gz;

    raw = (int16_t)(((uint16_t)rx_buf[2] << 8) | rx_buf[1]);
    gx = raw * BMI088_GYRO_SEN_2000;
    raw = (int16_t)(((uint16_t)rx_buf[4] << 8) | rx_buf[3]);
    gy = raw * BMI088_GYRO_SEN_2000;
    raw = (int16_t)(((uint16_t)rx_buf[6] << 8) | rx_buf[5]);
    gz = raw * BMI088_GYRO_SEN_2000;

    g_bmi088_ctx.data.gyro_radps[0] = gy;
    g_bmi088_ctx.data.gyro_radps[1] = -gx;
    g_bmi088_ctx.data.gyro_radps[2] = gz;
    g_bmi088_ctx.data.sample_ms = bsp_time_get_ms();
    g_bmi088_ctx.data.online = true;
    g_bmi088_ctx.data.valid = true;
}

void bmi088_parse_accel_frame(const uint8_t *rx_buf)
{
    int16_t raw;
    float ax;
    float ay;
    float az;

    raw = (int16_t)(((uint16_t)rx_buf[3] << 8) | rx_buf[2]);
    ax = raw * BMI088_ACCEL_SEN_6G;
    raw = (int16_t)(((uint16_t)rx_buf[5] << 8) | rx_buf[4]);
    ay = raw * BMI088_ACCEL_SEN_6G;
    raw = (int16_t)(((uint16_t)rx_buf[7] << 8) | rx_buf[6]);
    az = raw * BMI088_ACCEL_SEN_6G;

    g_bmi088_ctx.data.accel_mps2[0] = ay;
    g_bmi088_ctx.data.accel_mps2[1] = -ax;
    g_bmi088_ctx.data.accel_mps2[2] = az;
    g_bmi088_ctx.data.sample_ms = bsp_time_get_ms();
    g_bmi088_ctx.data.online = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((g_bmi088_ctx.ready == false) || (g_bmi088_ctx.async.enabled == 0U))
        return;

    if (GPIO_Pin == BMI088_ACCEL_INT_Pin)
    {
        g_bmi088_ctx.async.pending_accel = 1U;
    }
    else if (GPIO_Pin == BMI088_GYRO_INT_Pin)
    {
        g_bmi088_ctx.async.pending_gyro = 1U;
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1)
        return;

    if (g_bmi088_ctx.async.state == BMI088_ASYNC_GYRO_BUSY)
    {
        bmi088_gyro_cs_high();
        bmi088_parse_gyro_frame(g_bmi088_ctx.async.rx_buf);
        g_bmi088_ctx.last_async_update_ms = bsp_time_get_ms();
        g_bmi088_ctx.async.prefer_accel = 1U;
    }
    else if (g_bmi088_ctx.async.state == BMI088_ASYNC_ACCEL_BUSY)
    {
        bmi088_accel_cs_high();
        bmi088_parse_accel_frame(g_bmi088_ctx.async.rx_buf);
        g_bmi088_ctx.last_async_update_ms = bsp_time_get_ms();
        g_bmi088_ctx.async.prefer_accel = 0U;
    }
    else
    {
        return;
    }

    g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    g_bmi088_ctx.data.error_flags &= ~DRV_BMI088_ERR_SPI;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1)
        return;

    g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_SPI;
    g_bmi088_ctx.ready = false;
    g_bmi088_ctx.data.online = false;
    g_bmi088_ctx.data.valid = false;
    g_bmi088_ctx.async.enabled = 0U;
    g_bmi088_ctx.async.pending_accel = 0U;
    g_bmi088_ctx.async.pending_gyro = 0U;
    g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    bmi088_accel_cs_high();
    bmi088_gyro_cs_high();
}
