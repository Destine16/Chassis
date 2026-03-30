#ifndef USER_SERVICE_SRV_WATCHDOG_H
#define USER_SERVICE_SRV_WATCHDOG_H

#include <stdint.h>

/* 统一离线故障位，供遥测直接回传。 */
#define SRV_WATCHDOG_FLAG_RC_OFFLINE         (1UL << 0)
#define SRV_WATCHDOG_FLAG_MOTOR1_OFFLINE     (1UL << 1)
#define SRV_WATCHDOG_FLAG_MOTOR2_OFFLINE     (1UL << 2)
#define SRV_WATCHDOG_FLAG_MOTOR3_OFFLINE     (1UL << 3)
#define SRV_WATCHDOG_FLAG_MOTOR4_OFFLINE     (1UL << 4)

/* 看门狗不主动恢复系统，只负责集中汇总在线状态。 */
void srv_watchdog_init(void);
void srv_watchdog_update(void);
uint32_t srv_watchdog_get_flags(void);

#endif /* USER_SERVICE_SRV_WATCHDOG_H */
