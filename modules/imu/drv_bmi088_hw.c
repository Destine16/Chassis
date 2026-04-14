#include "drv_bmi088_internal.h"

#include "bmi088_reg.h"
#include "bsp_time.h"
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

static bool bmi088_read_accel_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[3] = {reg | 0x80U, 0x55U, 0x55U};
    uint8_t rx[3] = {0};

    bmi088_accel_cs_low();
    if (HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3U, BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_accel_cs_high();
        return false;
    }
    bmi088_accel_cs_high();
    *value = rx[2];
    return true;
}

uint32_t bmi088_try_init(void)
{
    uint32_t err = DRV_BMI088_ERR_NONE;
    uint8_t chip_id = 0U;

    bmi088_async_reset();
    bmi088_accel_cs_high();
    bmi088_gyro_cs_high();
    HAL_Delay(1U);

    if (!bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &chip_id) ||
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
    (void)bmi088_read_accel_reg(BMI088_ACC_CHIP_ID, &chip_id);
    HAL_Delay(1U);
    (void)bmi088_write_reg(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(80U);

    if (!bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON))
    {
        err |= DRV_BMI088_ERR_ACCEL_CONFIG;
    }
    HAL_Delay(5U);

    if (!bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE))
    {
        err |= DRV_BMI088_ERR_ACCEL_CONFIG;
    }
    HAL_Delay(50U);

    if (!bmi088_write_reg(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin, BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_SET) ||
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
        g_bmi088_ctx.data.valid = true;
        g_bmi088_ctx.data.online = false;
        g_bmi088_ctx.data.sample_ms = 0U;
        g_bmi088_ctx.last_async_update_ms = bsp_time_get_ms();
        g_bmi088_ctx.last_transfer_start_ms = g_bmi088_ctx.last_async_update_ms;
    }

    return err;
}
