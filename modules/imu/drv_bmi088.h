#ifndef USER_MODULES_IMU_DRV_BMI088_H
#define USER_MODULES_IMU_DRV_BMI088_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float gyro_radps[3];
    float accel_mps2[3];
    uint32_t sample_ms;
    uint32_t error_flags;
    bool valid;
    bool online;
} bmi088_data_t;

enum
{
    DRV_BMI088_ERR_NONE = 0U,
    DRV_BMI088_ERR_ACCEL_NOT_FOUND = 1U << 0,
    DRV_BMI088_ERR_GYRO_NOT_FOUND = 1U << 1,
    DRV_BMI088_ERR_ACCEL_CONFIG = 1U << 2,
    DRV_BMI088_ERR_GYRO_CONFIG = 1U << 3,
    DRV_BMI088_ERR_SPI = 1U << 4,
    DRV_BMI088_ERR_GYRO_BIAS_NOT_READY = 1U << 5,
};

int drv_bmi088_init(void);
void drv_bmi088_poll(void);
bool drv_bmi088_get_data(bmi088_data_t *out);
bool drv_bmi088_is_online(void);
uint32_t drv_bmi088_get_error_flags(void);

#endif /* USER_MODULES_IMU_DRV_BMI088_H */
