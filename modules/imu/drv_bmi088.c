#include "drv_bmi088.h"

#include <string.h>

#include "bmi088_reg.h"
#include "bsp_time.h"
#include "main.h"
#include "spi.h"

#define BMI088_SPI_TIMEOUT_MS                10U
#define BMI088_INIT_RETRY_MS                 500U
#define BMI088_OFFLINE_TIMEOUT_MS            100U
#define BMI088_ACCEL_SEN_6G                  0.00179443359375f
#define BMI088_GYRO_SEN_2000                 0.00106526443603f
#define BMI088_GYRO_BIAS_CALIB_SAMPLES       300U
#define BMI088_ACCEL_DMA_FRAME_LEN           8U
#define BMI088_GYRO_DMA_FRAME_LEN            7U

typedef enum
{
    BMI088_ASYNC_IDLE = 0,
    BMI088_ASYNC_GYRO_BUSY,
    BMI088_ASYNC_ACCEL_BUSY,
} bmi088_async_state_t;

typedef struct
{
    volatile uint8_t enabled;
    volatile uint8_t pending_accel;
    volatile uint8_t pending_gyro;
    volatile bmi088_async_state_t state;
    uint8_t tx_buf[8];
    uint8_t rx_buf[8];
} bmi088_async_context_t;

typedef struct
{
    bmi088_data_t data;
    uint32_t last_init_attempt_ms;
    uint32_t last_async_update_ms;
    float gyro_bias_radps[3];
    float gyro_bias_sum[3];
    uint16_t gyro_bias_count;
    bool ready;
    bool bias_ready;
    bmi088_async_context_t async;
} bmi088_context_t;

static bmi088_context_t g_bmi088_ctx;

static void bmi088_accel_cs_low(void);
static void bmi088_accel_cs_high(void);
static void bmi088_gyro_cs_low(void);
static void bmi088_gyro_cs_high(void);
static bool bmi088_write_reg(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t value);
static bool bmi088_read_reg(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t *value);
static uint32_t bmi088_try_init(void);
static void bmi088_async_reset(void);
static void bmi088_async_enable(void);
static void bmi088_service_pending_transfer(void);
static bool bmi088_start_gyro_dma_transfer(void);
static bool bmi088_start_accel_dma_transfer(void);
static void bmi088_parse_gyro_frame(const uint8_t *rx_buf);
static void bmi088_parse_accel_frame(const uint8_t *rx_buf);
static void bmi088_complete_gyro_sample(float *gyro_x, float *gyro_y, float *gyro_z);

int drv_bmi088_init(void)
{
    memset(&g_bmi088_ctx, 0, sizeof(g_bmi088_ctx));
    g_bmi088_ctx.last_init_attempt_ms = bsp_time_get_ms();
    g_bmi088_ctx.data.error_flags = bmi088_try_init();
    g_bmi088_ctx.ready = (g_bmi088_ctx.data.error_flags == DRV_BMI088_ERR_NONE);
    if (g_bmi088_ctx.ready)
    {
        g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_GYRO_BIAS_NOT_READY;
        bmi088_async_enable();
    }
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
            {
                g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_GYRO_BIAS_NOT_READY;
                bmi088_async_enable();
            }
        }
        return;
    }

    if (bsp_time_is_expired(now_ms, g_bmi088_ctx.last_async_update_ms, BMI088_OFFLINE_TIMEOUT_MS))
    {
        g_bmi088_ctx.ready = false;
        g_bmi088_ctx.data.online = false;
        g_bmi088_ctx.data.valid = false;
        g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_SPI;
        bmi088_async_reset();
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

static bool bmi088_write_reg(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi1, tx, 2U, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        return false;
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    return true;
}

static bool bmi088_read_reg(GPIO_TypeDef *port, uint16_t pin, uint8_t reg, uint8_t *value)
{
    uint8_t tx[2] = {reg | 0x80U, 0x55U};
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        return false;
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    *value = rx[1];
    return true;
}

static uint32_t bmi088_try_init(void)
{
    uint32_t err = DRV_BMI088_ERR_NONE;
    uint8_t chip_id = 0U;

    bmi088_async_reset();
    bmi088_accel_cs_high();
    bmi088_gyro_cs_high();
    HAL_Delay(1U);

    if (!bmi088_read_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_CHIP_ID, &chip_id) ||
        (chip_id != BMI088_ACC_CHIP_ID_VALUE))
    {
        err |= DRV_BMI088_ERR_ACCEL_NOT_FOUND;
    }

    if (!bmi088_read_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_CHIP_ID, &chip_id) ||
        (chip_id != BMI088_GYRO_CHIP_ID_VALUE))
    {
        err |= DRV_BMI088_ERR_GYRO_NOT_FOUND;
    }

    if (err != DRV_BMI088_ERR_NONE)
        return err;

    (void)bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(80U);
    (void)bmi088_write_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(80U);

    if (!bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON) ||
        !bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE) ||
        !bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_SET) ||
        !bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G) ||
        !bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_HIGH) ||
        !bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT))
    {
        err |= DRV_BMI088_ERR_ACCEL_CONFIG;
    }

    if (!bmi088_write_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_RANGE, BMI088_GYRO_2000) ||
        !bmi088_write_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_SET) ||
        !bmi088_write_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE) ||
        !bmi088_write_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_CTRL, BMI088_DRDY_ON) ||
        !bmi088_write_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_HIGH) ||
        !bmi088_write_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3))
    {
        err |= DRV_BMI088_ERR_GYRO_CONFIG;
    }

    if (err == DRV_BMI088_ERR_NONE)
    {
        memset(g_bmi088_ctx.gyro_bias_radps, 0, sizeof(g_bmi088_ctx.gyro_bias_radps));
        memset(g_bmi088_ctx.gyro_bias_sum, 0, sizeof(g_bmi088_ctx.gyro_bias_sum));
        g_bmi088_ctx.gyro_bias_count = 0U;
        g_bmi088_ctx.bias_ready = false;
        g_bmi088_ctx.data.valid = false;
        g_bmi088_ctx.data.online = false;
        g_bmi088_ctx.data.sample_ms = 0U;
        g_bmi088_ctx.last_async_update_ms = bsp_time_get_ms();
    }

    return err;
}

static void bmi088_async_reset(void)
{
    g_bmi088_ctx.async.enabled = 0U;
    g_bmi088_ctx.async.pending_accel = 0U;
    g_bmi088_ctx.async.pending_gyro = 0U;
    g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    bmi088_accel_cs_high();
    bmi088_gyro_cs_high();
}

static void bmi088_async_enable(void)
{
    g_bmi088_ctx.async.enabled = 1U;
    g_bmi088_ctx.async.pending_accel = 0U;
    g_bmi088_ctx.async.pending_gyro = 0U;
    g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    g_bmi088_ctx.last_async_update_ms = bsp_time_get_ms();
}

static void bmi088_service_pending_transfer(void)
{
    if ((g_bmi088_ctx.async.enabled == 0U) || (g_bmi088_ctx.async.state != BMI088_ASYNC_IDLE))
        return;

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

static bool bmi088_start_gyro_dma_transfer(void)
{
    if ((g_bmi088_ctx.async.enabled == 0U) || (g_bmi088_ctx.async.state != BMI088_ASYNC_IDLE))
        return false;

    g_bmi088_ctx.async.tx_buf[0] = BMI088_GYRO_X_L | 0x80U;
    memset(&g_bmi088_ctx.async.tx_buf[1], 0x55, BMI088_GYRO_DMA_FRAME_LEN - 1U);
    bmi088_gyro_cs_low();
    g_bmi088_ctx.async.state = BMI088_ASYNC_GYRO_BUSY;

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

    g_bmi088_ctx.async.tx_buf[0] = BMI088_ACCEL_XOUT_L | 0x80U;
    memset(&g_bmi088_ctx.async.tx_buf[1], 0x55, BMI088_ACCEL_DMA_FRAME_LEN - 1U);
    bmi088_accel_cs_low();
    g_bmi088_ctx.async.state = BMI088_ASYNC_ACCEL_BUSY;

    if (HAL_SPI_TransmitReceive_DMA(&hspi1, g_bmi088_ctx.async.tx_buf, g_bmi088_ctx.async.rx_buf, BMI088_ACCEL_DMA_FRAME_LEN) != HAL_OK)
    {
        bmi088_accel_cs_high();
        g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
        g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_SPI;
        return false;
    }

    return true;
}

static void bmi088_complete_gyro_sample(float *gyro_x, float *gyro_y, float *gyro_z)
{
    if (!g_bmi088_ctx.bias_ready)
    {
        g_bmi088_ctx.gyro_bias_sum[0] += *gyro_x;
        g_bmi088_ctx.gyro_bias_sum[1] += *gyro_y;
        g_bmi088_ctx.gyro_bias_sum[2] += *gyro_z;
        ++g_bmi088_ctx.gyro_bias_count;

        if (g_bmi088_ctx.gyro_bias_count >= BMI088_GYRO_BIAS_CALIB_SAMPLES)
        {
            g_bmi088_ctx.gyro_bias_radps[0] = g_bmi088_ctx.gyro_bias_sum[0] / (float)g_bmi088_ctx.gyro_bias_count;
            g_bmi088_ctx.gyro_bias_radps[1] = g_bmi088_ctx.gyro_bias_sum[1] / (float)g_bmi088_ctx.gyro_bias_count;
            g_bmi088_ctx.gyro_bias_radps[2] = g_bmi088_ctx.gyro_bias_sum[2] / (float)g_bmi088_ctx.gyro_bias_count;
            g_bmi088_ctx.bias_ready = true;
            g_bmi088_ctx.data.error_flags &= ~DRV_BMI088_ERR_GYRO_BIAS_NOT_READY;
        }
        else
        {
            *gyro_x = 0.0f;
            *gyro_y = 0.0f;
            *gyro_z = 0.0f;
            return;
        }
    }

    *gyro_x -= g_bmi088_ctx.gyro_bias_radps[0];
    *gyro_y -= g_bmi088_ctx.gyro_bias_radps[1];
    *gyro_z -= g_bmi088_ctx.gyro_bias_radps[2];
}

static void bmi088_parse_gyro_frame(const uint8_t *rx_buf)
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

    bmi088_complete_gyro_sample(&gx, &gy, &gz);

    g_bmi088_ctx.data.gyro_radps[0] = gx;
    g_bmi088_ctx.data.gyro_radps[1] = gy;
    g_bmi088_ctx.data.gyro_radps[2] = gz;
    g_bmi088_ctx.data.sample_ms = bsp_time_get_ms();
    g_bmi088_ctx.data.online = true;
    g_bmi088_ctx.data.valid = g_bmi088_ctx.bias_ready;
}

static void bmi088_parse_accel_frame(const uint8_t *rx_buf)
{
    int16_t raw;

    raw = (int16_t)(((uint16_t)rx_buf[3] << 8) | rx_buf[2]);
    g_bmi088_ctx.data.accel_mps2[0] = raw * BMI088_ACCEL_SEN_6G;
    raw = (int16_t)(((uint16_t)rx_buf[5] << 8) | rx_buf[4]);
    g_bmi088_ctx.data.accel_mps2[1] = raw * BMI088_ACCEL_SEN_6G;
    raw = (int16_t)(((uint16_t)rx_buf[7] << 8) | rx_buf[6]);
    g_bmi088_ctx.data.accel_mps2[2] = raw * BMI088_ACCEL_SEN_6G;
    g_bmi088_ctx.data.sample_ms = bsp_time_get_ms();
    g_bmi088_ctx.data.online = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((g_bmi088_ctx.ready == false) || (g_bmi088_ctx.async.enabled == 0U))
        return;

    if (GPIO_Pin == BMI088_ACCEL_INT_Pin)
    {
        if (g_bmi088_ctx.async.state == BMI088_ASYNC_IDLE)
            (void)bmi088_start_accel_dma_transfer();
        else
            g_bmi088_ctx.async.pending_accel = 1U;
    }
    else if (GPIO_Pin == BMI088_GYRO_INT_Pin)
    {
        if (g_bmi088_ctx.async.state == BMI088_ASYNC_IDLE)
            (void)bmi088_start_gyro_dma_transfer();
        else
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
    }
    else if (g_bmi088_ctx.async.state == BMI088_ASYNC_ACCEL_BUSY)
    {
        bmi088_accel_cs_high();
        bmi088_parse_accel_frame(g_bmi088_ctx.async.rx_buf);
        g_bmi088_ctx.last_async_update_ms = bsp_time_get_ms();
    }
    else
    {
        return;
    }

    g_bmi088_ctx.async.state = BMI088_ASYNC_IDLE;
    g_bmi088_ctx.data.error_flags &= ~DRV_BMI088_ERR_SPI;
    bmi088_service_pending_transfer();
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1)
        return;

    (void)HAL_SPI_Abort_IT(hspi);
    g_bmi088_ctx.data.error_flags |= DRV_BMI088_ERR_SPI;
    g_bmi088_ctx.ready = false;
    g_bmi088_ctx.data.online = false;
    g_bmi088_ctx.data.valid = false;
    bmi088_async_reset();
}
