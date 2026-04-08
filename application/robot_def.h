#ifndef CHASSIS_APPLICATION_ROBOT_DEF_H
#define CHASSIS_APPLICATION_ROBOT_DEF_H

#include <stdint.h>

#include "common_def.h"

/* 版本号可用于遥测或串口协议扩展。 */
#define APP_VERSION_MAJOR                      0U
#define APP_VERSION_MINOR                      1U
#define APP_VERSION_PATCH                      0U

/* 系统主控制频率和慢周期任务节拍。 */
#define APP_CFG_CONTROL_HZ                     1000U
#define APP_CFG_CONTROL_PERIOD_MS              1U
#define APP_CFG_TELEMETRY_PERIOD_MS            5U
#define APP_CFG_ENABLE_USB_CDC                 1U
#define APP_CFG_ENABLE_BMI088                  1U

/* 统一离线判定超时。 */
#define APP_CFG_RC_TIMEOUT_MS                  100U
#define APP_CFG_MOTOR_TIMEOUT_MS               100U

/* 通信缓冲区尺寸。 */
#define APP_CFG_DBUS_FRAME_LEN                 COMMON_DBUS_FRAME_LENGTH
#define APP_CFG_NAV_TX_BUFFER_SIZE             64U
#define APP_CFG_NAV_TX_QUEUE_DEPTH             16U

/* DR16 原始通道范围和接管判定阈值。 */
#define APP_CFG_RC_VALUE_MIN                   364U
#define APP_CFG_RC_VALUE_CENTER                1024U
#define APP_CFG_RC_VALUE_MAX                   1684U
#define APP_CFG_RC_VALUE_SPAN                  ((float)(APP_CFG_RC_VALUE_MAX - APP_CFG_RC_VALUE_CENTER))
#define APP_CFG_RC_DEADBAND_RAW                12.0f
#define APP_CFG_RC_TAKEOVER_THRESHOLD          0.18f
#define APP_CFG_RC_TAKEOVER_HOLD_MS            60U

/* 遥控器到车体坐标的符号映射，后续如发现方向反了可只改这里。 */
#define APP_CFG_RC_SIGN_VX                     1.0f
#define APP_CFG_RC_SIGN_VY                     -1.0f
#define APP_CFG_RC_SIGN_WZ                     -1.0f

/*
 * 第一版可编译骨架限幅。
 * TODO: 按实车能力替换。
 */
#define APP_CFG_MAX_VX_MPS                     1.3f
#define APP_CFG_MAX_VY_MPS                     1.3f
#define APP_CFG_MAX_WZ_RADPS                   2.0f

/*
 * 当前机械文档给出的关键参数：
 * - 轮半径 81.5 mm
 * - 轮对角线距离 425 mm
 * - hexroll 总减速比 268:17
 *
 * 对于 45° 四全向轮布局，轮对角线长度可直接换算为“底盘中心到轮心距离”的两倍。
 * 因此：
 * - center_to_wheel = diagonal / 2
 * - 平移投影系数使用 sin/cos(45°) = 1/sqrt(2)
 */
#define APP_CFG_OMNI_WHEEL_RADIUS_M            0.0815f
#define APP_CFG_OMNI_LAYOUT_DIAGONAL_M         0.425f
#define APP_CFG_OMNI_CENTER_TO_WHEEL_M         (APP_CFG_OMNI_LAYOUT_DIAGONAL_M * 0.5f)
#define APP_CFG_OMNI_45DEG_COEF                0.70710678118f
#define APP_CFG_M3508_GEAR_RATIO               (268.0f / 17.0f)

/*
 * 速度环默认限制。
 * 增益通过调试器直接修改全局 PID 变量；这里只保留积分和输出限制。
 */
#define APP_CFG_SPEED_PID_INT_LIMIT            3000.0f
#define APP_CFG_SPEED_PID_OUT_LIMIT            10000.0f
#define APP_CFG_C620_CURRENT_LIMIT             10000.0f

/*
 * 轮速参考统一饱和上限。
 * 当前按现有 vx/vy/wz 包络反推，单轮最坏情况约 22.6 rad/s，
 * 这里保留约 20% 调试余量，先收紧到 28 rad/s，避免保护过松。
 */
#define APP_CFG_MAX_WHEEL_SPEED_RADPS          28.0f

/* 底盘命令斜坡，避免切换模式或摇杆突变时电流冲击过大。 */
#define APP_CFG_CMD_RAMP_VX_MPS2               4.0f
#define APP_CFG_CMD_RAMP_VY_MPS2               4.0f
#define APP_CFG_CMD_RAMP_WZ_RADPS2             8.0f

/* 单轮系统辨识默认参数与 CCMRAM 缓冲容量。 */
#define APP_CFG_SYSID_DEFAULT_WHEEL_ID         CHASSIS_WHEEL_FRONT_LEFT
#define APP_CFG_SYSID_DEFAULT_AMPLITUDE_RAW    2500
#define APP_CFG_SYSID_DEFAULT_BIT_PERIOD_MS    40U
#define APP_CFG_SYSID_DEFAULT_TOTAL_BITS       127U
#define APP_CFG_SYSID_SAMPLE_CAPACITY          6000U

/* USB CDC 观测帧协议基础字段。 */
#define APP_CFG_NAV_PROTO_SOF0                 0xA5U
#define APP_CFG_NAV_PROTO_SOF1                 0x5AU
#define APP_CFG_NAV_OBSERVATION_PAYLOAD_SIZE   46U

/*
 * C620/M3508 官方 CAN 协议：
 * - ESC ID 1~4 使用标准帧 0x200 下发电流
 * - ESC ID 5~8 使用标准帧 0x1FF 下发电流
 * - 反馈报文 ID = 0x200 + ESC ID
 *
 * 当前底盘四个电机均位于 1~4 组，因此统一走 0x200。
 * 若后续某个轮子的 ESC ID 改动，只需要改下面这四个宏。
 */
#define APP_CFG_CAN_C620_GROUP1_TX_ID          0x200U
#define APP_CFG_CAN_C620_GROUP2_TX_ID          0x1FFU
#define APP_CFG_CAN_M3508_FB_BASE_ID           0x200U

#define APP_CFG_CAN_ESC_ID_FRONT_LEFT          2U
#define APP_CFG_CAN_ESC_ID_FRONT_RIGHT         1U
#define APP_CFG_CAN_ESC_ID_REAR_LEFT           3U
#define APP_CFG_CAN_ESC_ID_REAR_RIGHT          4U

#define APP_CFG_CAN_CHASSIS_TX_ID              APP_CFG_CAN_C620_GROUP1_TX_ID
#define APP_CFG_CAN_MOTOR1_FB_ID               (APP_CFG_CAN_M3508_FB_BASE_ID + APP_CFG_CAN_ESC_ID_FRONT_LEFT)
#define APP_CFG_CAN_MOTOR2_FB_ID               (APP_CFG_CAN_M3508_FB_BASE_ID + APP_CFG_CAN_ESC_ID_FRONT_RIGHT)
#define APP_CFG_CAN_MOTOR3_FB_ID               (APP_CFG_CAN_M3508_FB_BASE_ID + APP_CFG_CAN_ESC_ID_REAR_LEFT)
#define APP_CFG_CAN_MOTOR4_FB_ID               (APP_CFG_CAN_M3508_FB_BASE_ID + APP_CFG_CAN_ESC_ID_REAR_RIGHT)

#if ((APP_CFG_CAN_ESC_ID_FRONT_LEFT < 1U) || (APP_CFG_CAN_ESC_ID_FRONT_LEFT > 4U) || \
     (APP_CFG_CAN_ESC_ID_FRONT_RIGHT < 1U) || (APP_CFG_CAN_ESC_ID_FRONT_RIGHT > 4U) || \
     (APP_CFG_CAN_ESC_ID_REAR_LEFT < 1U) || (APP_CFG_CAN_ESC_ID_REAR_LEFT > 4U) || \
     (APP_CFG_CAN_ESC_ID_REAR_RIGHT < 1U) || (APP_CFG_CAN_ESC_ID_REAR_RIGHT > 4U))
#error "Chassis wheel ESC IDs must stay in C620 CAN group 1 (1..4)."
#endif

#if ((APP_CFG_CAN_ESC_ID_FRONT_LEFT == APP_CFG_CAN_ESC_ID_FRONT_RIGHT) || \
     (APP_CFG_CAN_ESC_ID_FRONT_LEFT == APP_CFG_CAN_ESC_ID_REAR_LEFT) || \
     (APP_CFG_CAN_ESC_ID_FRONT_LEFT == APP_CFG_CAN_ESC_ID_REAR_RIGHT) || \
     (APP_CFG_CAN_ESC_ID_FRONT_RIGHT == APP_CFG_CAN_ESC_ID_REAR_LEFT) || \
     (APP_CFG_CAN_ESC_ID_FRONT_RIGHT == APP_CFG_CAN_ESC_ID_REAR_RIGHT) || \
     (APP_CFG_CAN_ESC_ID_REAR_LEFT == APP_CFG_CAN_ESC_ID_REAR_RIGHT))
#error "Chassis wheel ESC IDs must be unique on the same CAN bus."
#endif

#endif /* CHASSIS_APPLICATION_ROBOT_DEF_H */
