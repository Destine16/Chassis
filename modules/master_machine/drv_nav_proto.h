#ifndef USER_DRIVERS_DRV_NAV_PROTO_H
#define USER_DRIVERS_DRV_NAV_PROTO_H

#include "protocol_nav.h"

/* 初始化 USB CDC 观测发送链。 */
int drv_nav_proto_init(void);
bool drv_nav_proto_send_observation(const protocol_nav_observation_payload_t *payload);
void drv_nav_proto_on_tx_complete(void);

#endif /* USER_DRIVERS_DRV_NAV_PROTO_H */
