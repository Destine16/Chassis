#include "FreeRTOS.h"
#include <string.h>
#include "robot.h"
#include "robot_def.h"
#include "bsp_time.h"
#include "drv_bmi088.h"
#include "drv_dbus.h"
#include "drv_nav_proto.h"
#include "srv_arbiter.h"
#include "srv_debug.h"
#include "srv_motor.h"
#include "srv_rc.h"
#include "srv_watchdog.h"
#include "task.h"
#include "ctrl_chassis.h"
#include "protocol_nav.h"
#include "sysid.h"

/*
 * 1 kHz 底盘主控任务。
 * 这里只做控制闭环主链路，不在中断里执行控制算法。
 */
void StartChassisTask(void *argument)
{
    TickType_t last_wake_time;

    (void)argument;
    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        chassis_cmd_t final_cmd;
        arbiter_state_t arbiter_state;

        /* 固定周期运行，保证底盘控制节拍稳定。 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(APP_CFG_CONTROL_PERIOD_MS));

        /* 先拉取最新反馈并更新在线状态，再进入仲裁和控制。 */
        srv_motor_poll_driver_feedback();
        if (APP_CFG_ENABLE_BMI088 != 0U)
            drv_bmi088_poll();
        srv_watchdog_update();
        srv_arbiter_step();

        if (sysid_task_step())
        {
            srv_debug_refresh();
            continue;
        }

        arbiter_state = srv_arbiter_get_state();
        final_cmd = srv_arbiter_get_final_cmd();
        ctrl_chassis_execute(arbiter_state.resolved_mode, &final_cmd);
        srv_debug_refresh();
    }
}

/* 遥控器任务由 USART3 IDLE 中断通知唤醒，只解析最新一帧 DBUS。 */
void StartRcTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t notifications = 0U;
        drv_dbus_frame_t frame;

        xTaskNotifyWait(0U, 0xFFFFFFFFUL, &notifications, portMAX_DELAY);
        if ((notifications & ROBOT_NOTIFY_RC_FRAME) == 0U)
            continue;

        /* DBUS 为固定 18 字节帧，驱动层已处理 DMA/IDLE 细节。 */
        if (drv_dbus_read_latest(&frame))
            srv_rc_update_from_dbus(&frame);
    }
}

/* 遥测任务通过 USB CDC 周期上报轮速与 IMU 原始观测。 */
void StartTelemetryTask(void *argument)
{
    (void)argument;

    if (APP_CFG_ENABLE_USB_CDC == 0U)
    {
        vTaskSuspend(NULL);
    }

    for (;;)
    {
        arbiter_state_t arbiter_state;
        bmi088_data_t imu;
        motor_feedback_t motors[COMMON_WHEEL_COUNT];
        protocol_nav_observation_payload_t payload = {0};
        uint16_t flags = 0U;
        uint32_t watchdog_flags;

        vTaskDelay(pdMS_TO_TICKS(APP_CFG_TELEMETRY_PERIOD_MS));

        arbiter_state = srv_arbiter_get_state();
        watchdog_flags = srv_watchdog_get_flags();
        srv_motor_get_feedback_all(motors);

        payload.t_ms = bsp_time_get_ms();
        payload.w_fl = motors[CHASSIS_WHEEL_FRONT_LEFT].wheel_speed_radps;
        payload.w_fr = motors[CHASSIS_WHEEL_FRONT_RIGHT].wheel_speed_radps;
        payload.w_rl = motors[CHASSIS_WHEEL_REAR_LEFT].wheel_speed_radps;
        payload.w_rr = motors[CHASSIS_WHEEL_REAR_RIGHT].wheel_speed_radps;
        if ((APP_CFG_ENABLE_BMI088 != 0U) && drv_bmi088_get_data(&imu))
        {
            payload.gyro_x = imu.gyro_radps[0];
            payload.gyro_y = imu.gyro_radps[1];
            payload.gyro_z = imu.gyro_radps[2];
            payload.acc_x = imu.accel_mps2[0];
            payload.acc_y = imu.accel_mps2[1];
            payload.acc_z = imu.accel_mps2[2];
            flags |= PROTOCOL_NAV_OBS_FLAG_IMU_VALID;
        }
        else
        {
            payload.gyro_x = 0.0f;
            payload.gyro_y = 0.0f;
            payload.gyro_z = 0.0f;
            payload.acc_x = 0.0f;
            payload.acc_y = 0.0f;
            payload.acc_z = 0.0f;
            flags |= PROTOCOL_NAV_OBS_FLAG_IMU_FAULT;
        }

        if (motors[CHASSIS_WHEEL_FRONT_LEFT].online)
            flags |= PROTOCOL_NAV_OBS_FLAG_WHEEL_FL_VALID;
        if (motors[CHASSIS_WHEEL_FRONT_RIGHT].online)
            flags |= PROTOCOL_NAV_OBS_FLAG_WHEEL_FR_VALID;
        if (motors[CHASSIS_WHEEL_REAR_LEFT].online)
            flags |= PROTOCOL_NAV_OBS_FLAG_WHEEL_RL_VALID;
        if (motors[CHASSIS_WHEEL_REAR_RIGHT].online)
            flags |= PROTOCOL_NAV_OBS_FLAG_WHEEL_RR_VALID;
        if ((watchdog_flags & (SRV_WATCHDOG_FLAG_MOTOR1_OFFLINE |
                               SRV_WATCHDOG_FLAG_MOTOR2_OFFLINE |
                               SRV_WATCHDOG_FLAG_MOTOR3_OFFLINE |
                               SRV_WATCHDOG_FLAG_MOTOR4_OFFLINE)) != 0U)
            flags |= PROTOCOL_NAV_OBS_FLAG_MOTOR_FAULT;
        if ((watchdog_flags & SRV_WATCHDOG_FLAG_RC_OFFLINE) != 0U)
            flags |= PROTOCOL_NAV_OBS_FLAG_RC_OFFLINE;
        if (arbiter_state.resolved_mode == APP_MODE_SAFE)
            flags |= PROTOCOL_NAV_OBS_FLAG_CHASSIS_SAFE_MODE;
        payload.flags = flags;

        (void)drv_nav_proto_send_observation(&payload);
        srv_debug_refresh();
    }
}
