#ifndef USER_SERVICE_SRV_NAV_H
#define USER_SERVICE_SRV_NAV_H

#include "common_def.h"
#include "drv_nav_proto.h"

/* 导航服务层只缓存最新值，不做历史堆积。 */
void srv_nav_init(void);
void srv_nav_update_from_driver(const drv_nav_cmd_t *cmd);
void srv_nav_refresh_online(uint32_t now_ms);
nav_state_t srv_nav_get_state(void);

#endif /* USER_SERVICE_SRV_NAV_H */
