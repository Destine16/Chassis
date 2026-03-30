#ifndef USER_COMMON_COMMON_DEF_H
#define USER_COMMON_COMMON_DEF_H

#include <stdbool.h>
#include <stdint.h>

/* 全工程统一常量，避免在不同模块里重复写死。 */
#define COMMON_WHEEL_COUNT         4U
#define COMMON_DBUS_FRAME_LENGTH   18U

/* 系统最终工作模式。当前底盘移动只允许 SAFE / MANUAL。 */
typedef enum
{
    APP_MODE_SAFE = 0,
    APP_MODE_MANUAL = 1
} app_mode_t;

/* DR16 拨杆在 DBUS 中的常见编码。 */
typedef enum
{
    RC_SWITCH_UNKNOWN = 0,
    RC_SWITCH_UP = 1,
    RC_SWITCH_DOWN = 2,
    RC_SWITCH_MID = 3
} rc_switch_pos_t;

/* 统一轮序，所有运动学、反馈和电流发送都按这个顺序组织数组。 */
typedef enum
{
    CHASSIS_WHEEL_FRONT_LEFT = 0,
    CHASSIS_WHEEL_FRONT_RIGHT = 1,
    CHASSIS_WHEEL_REAR_LEFT = 2,
    CHASSIS_WHEEL_REAR_RIGHT = 3
} chassis_wheel_id_t;

/* 底盘统一速度命令。 */
typedef struct
{
    float vx_mps;      /* 单位: m/s */
    float vy_mps;      /* 单位: m/s */
    float wz_radps;    /* 单位: rad/s */
} chassis_cmd_t;

/* 底盘控制链路中间结果：轮速参考和电流给定。 */
typedef struct
{
    float wheel_speed_ref_radps[COMMON_WHEEL_COUNT];  /* 单位: rad/s */
    float current_cmd[COMMON_WHEEL_COUNT];            /* 单位: C620 电流给定原始量 */
} wheel_targets_t;

/* 鼠标附加输入，第一版仅保留原始值，不直接参与底盘控制。 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
} rc_mouse_t;

/*
 * 遥控器状态。
 * ch_raw 和 ch_norm 同时保留，便于调试原始量与控制量的对应关系。
 */
typedef struct
{
    uint16_t ch_raw[4];     /* DBUS 原始值，典型范围 364..1684 */
    float ch_norm[4];       /* 经过死区处理后的归一化值，范围 [-1, 1] */
    rc_switch_pos_t s1;
    rc_switch_pos_t s2;
    rc_mouse_t mouse;
    uint16_t key_bits;
    bool online;
    bool takeover_active;
    uint32_t last_update_ms;
    uint32_t takeover_since_ms;
} rc_state_t;

/* 导航侧下发给底盘的最新控制状态。 */
typedef struct
{
    float vx_mps;        /* 单位: m/s */
    float vy_mps;        /* 单位: m/s */
    float wz_radps;      /* 单位: rad/s */
    bool valid;
    bool online;
    uint8_t seq;
    uint32_t last_update_ms;
} nav_state_t;

/* 电机反馈统一缓存格式。 */
typedef struct
{
    uint16_t ecd;
    int16_t rpm;
    int16_t current;
    uint8_t temperature;
    bool online;
    uint32_t last_update_ms;
    float wheel_speed_radps;    /* 单位: rad/s，减速箱输出端轮速 */
} motor_feedback_t;

/* 仲裁状态快照，供底盘任务和遥测统一读取。 */
typedef struct
{
    app_mode_t requested_mode;
    app_mode_t resolved_mode;
    uint32_t flags;
    uint32_t last_transition_ms;
} arbiter_state_t;

/* MCU 回传给导航侧的遥测状态。 */
typedef struct
{
    app_mode_t mode;
    uint32_t flags;
    chassis_cmd_t cmd_estimate;     /* 第一版先回传限幅后的命令值，后续再接入估计器 */
    uint32_t seq;
} telemetry_status_t;

#endif /* USER_COMMON_COMMON_DEF_H */
