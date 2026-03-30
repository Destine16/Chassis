#ifndef USER_MODULES_IMU_BMI088_REG_H
#define USER_MODULES_IMU_BMI088_REG_H

#define BMI088_ACC_CHIP_ID                   0x00U
#define BMI088_ACC_CHIP_ID_VALUE             0x1EU
#define BMI088_ACCEL_XOUT_L                  0x12U
#define BMI088_TEMP_M                        0x22U
#define BMI088_ACC_CONF                      0x40U
#define BMI088_ACC_CONF_MUST_SET             0x80U
#define BMI088_ACC_NORMAL                    0x20U
#define BMI088_ACC_800_HZ                    0x0BU
#define BMI088_ACC_RANGE                     0x41U
#define BMI088_ACC_RANGE_6G                  0x01U
#define BMI088_INT1_IO_CTRL                  0x53U
#define BMI088_ACC_INT1_IO_ENABLE            0x08U
#define BMI088_ACC_INT1_GPIO_PP              0x00U
#define BMI088_ACC_INT1_GPIO_HIGH            0x02U
#define BMI088_INT_MAP_DATA                  0x58U
#define BMI088_ACC_INT1_DRDY_INTERRUPT       0x04U
#define BMI088_ACC_PWR_CONF                  0x7CU
#define BMI088_ACC_PWR_ACTIVE_MODE           0x00U
#define BMI088_ACC_PWR_CTRL                  0x7DU
#define BMI088_ACC_ENABLE_ACC_ON             0x04U
#define BMI088_ACC_SOFTRESET                 0x7EU
#define BMI088_ACC_SOFTRESET_VALUE           0xB6U

#define BMI088_GYRO_CHIP_ID                  0x00U
#define BMI088_GYRO_CHIP_ID_VALUE            0x0FU
#define BMI088_GYRO_X_L                      0x02U
#define BMI088_GYRO_RANGE                    0x0FU
#define BMI088_GYRO_2000                     0x00U
#define BMI088_GYRO_BANDWIDTH                0x10U
#define BMI088_GYRO_BANDWIDTH_MUST_SET       0x80U
#define BMI088_GYRO_2000_230_HZ              0x01U
#define BMI088_GYRO_LPM1                     0x11U
#define BMI088_GYRO_NORMAL_MODE              0x00U
#define BMI088_GYRO_SOFTRESET                0x14U
#define BMI088_GYRO_SOFTRESET_VALUE          0xB6U
#define BMI088_GYRO_CTRL                     0x15U
#define BMI088_DRDY_ON                       0x80U
#define BMI088_GYRO_INT3_INT4_IO_CONF        0x16U
#define BMI088_GYRO_INT3_GPIO_PP             0x00U
#define BMI088_GYRO_INT3_GPIO_HIGH           0x01U
#define BMI088_GYRO_INT3_INT4_IO_MAP         0x18U
#define BMI088_GYRO_DRDY_IO_INT3             0x01U

#endif /* USER_MODULES_IMU_BMI088_REG_H */
