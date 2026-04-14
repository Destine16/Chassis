#ifndef USER_MODULES_IMU_DRV_BMI088_INTERNAL_H
#define USER_MODULES_IMU_DRV_BMI088_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "drv_bmi088.h"

#define BMI088_SPI_TIMEOUT_MS                10U
#define BMI088_INIT_RETRY_MS                 500U
#define BMI088_OFFLINE_TIMEOUT_MS            100U
#define BMI088_ACCEL_SEN_6G                  0.00179443359375f
#define BMI088_GYRO_SEN_2000                 0.00106526443603f
#define BMI088_ACCEL_DMA_FRAME_LEN           8U
#define BMI088_GYRO_DMA_FRAME_LEN            7U
#define BMI088_ASYNC_FALLBACK_MS             5U
#define BMI088_DMA_TIMEOUT_MS                5U

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
    volatile uint8_t prefer_accel;
    volatile bmi088_async_state_t state;
    uint8_t tx_buf[8];
    uint8_t rx_buf[8];
} bmi088_async_context_t;

typedef struct
{
    bmi088_data_t data;
    uint32_t last_init_attempt_ms;
    uint32_t last_async_update_ms;
    uint32_t last_transfer_start_ms;
    bool ready;
    bmi088_async_context_t async;
} bmi088_context_t;

extern bmi088_context_t g_bmi088_ctx;

uint32_t bmi088_try_init(void);
void bmi088_async_reset(void);
void bmi088_async_enable(void);
void bmi088_service_pending_transfer(void);
void bmi088_parse_gyro_frame(const uint8_t *rx_buf);
void bmi088_parse_accel_frame(const uint8_t *rx_buf);

#endif /* USER_MODULES_IMU_DRV_BMI088_INTERNAL_H */
