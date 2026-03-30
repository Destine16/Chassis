#ifndef USER_SERVICE_SRV_ARBITER_H
#define USER_SERVICE_SRV_ARBITER_H

#include "common_def.h"

/* 仲裁层附加标志位，用于调试和遥测。 */
#define SRV_ARBITER_FLAG_RC_OFFLINE       (1UL << 0)

/* SAFE / MANUAL 仲裁总入口。 */
void srv_arbiter_init(void);
void srv_arbiter_step(void);
arbiter_state_t srv_arbiter_get_state(void);
chassis_cmd_t srv_arbiter_get_final_cmd(void);

#endif /* USER_SERVICE_SRV_ARBITER_H */
