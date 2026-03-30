#ifndef CHASSIS_APPLICATION_ROBOT_H
#define CHASSIS_APPLICATION_ROBOT_H

#include "FreeRTOS.h"
#include "task.h"

/* 任务通知位定义。当前只有 RC 使用“ISR 置位 + 任务读取最新值”的模式。 */
#define ROBOT_NOTIFY_RC_FRAME    (1UL << 0)

/* 用户层应用初始化总入口，只允许执行一次。 */
void robot_init(void);

/* 仅暴露真正需要从驱动层访问的任务句柄。 */
TaskHandle_t robot_get_rc_task(void);

#endif /* CHASSIS_APPLICATION_ROBOT_H */
