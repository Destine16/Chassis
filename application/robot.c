#include "robot.h"

#include "cmsis_os2.h"

#include "bsp_can.h"
#include "bsp_time.h"
#include "common_assert.h"
#include "ctrl_chassis.h"
#include "drv_can_motor.h"
#include "drv_dbus.h"
#include "drv_bmi088.h"
#include "drv_nav_proto.h"
#include "srv_arbiter.h"
#include "srv_chassis.h"
#include "srv_debug.h"
#include "srv_motor.h"
#include "srv_rc.h"
#include "srv_watchdog.h"
#include "sysid.h"
#include "usb_device.h"

extern osThreadId_t rcTaskHandle;

void robot_init(void)
{
    /*
     * 先初始化状态缓存和控制器，再启动底层通信驱动。
     * 这样一旦中断接收到数据，相关服务层已经具备接收条件。
     */
    srv_rc_init();
    srv_motor_init();
    srv_chassis_init();
    srv_watchdog_init();
    srv_arbiter_init();
    srv_debug_init();
    ctrl_chassis_init();
    sysid_init();
    if (APP_CFG_ENABLE_USB_CDC != 0U)
        MX_USB_DEVICE_Init();

    /* IMU 对当前 sysid 不关键，可按配置关闭初始化以避免外设 bring-up 阻塞主链。 */
    if (APP_CFG_ENABLE_BMI088 != 0U)
        (void)drv_bmi088_init();

    /* 启动 DBUS、USB CDC 观测链和 CAN 电机驱动骨架。 */
    COMMON_ASSERT(drv_dbus_init() == 0);
    COMMON_ASSERT(drv_nav_proto_init() == 0);
    COMMON_ASSERT(drv_can_motor_init() == 0);
}

TaskHandle_t robot_get_rc_task(void)
{
    /* CMSIS-RTOS 线程 ID 在 FreeRTOS 适配层里可直接当作任务句柄使用。 */
    return (TaskHandle_t)rcTaskHandle;
}
