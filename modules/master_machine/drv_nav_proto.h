#ifndef USER_DRIVERS_DRV_NAV_PROTO_H
#define USER_DRIVERS_DRV_NAV_PROTO_H

#include <stdint.h>

#include "protocol_nav.h"

typedef struct
{
    uint32_t drop_count;
    uint32_t queue_full_count;
    uint32_t tx_start_fail_count;
    uint32_t enqueue_count;
    uint32_t dequeue_count;
    uint8_t queue_count;
    uint8_t queue_depth_peak;
    uint8_t busy;
    uint8_t seq;
} drv_nav_proto_stats_t;

/* 初始化 USB CDC 观测发送链。 */
int drv_nav_proto_init(void);
bool drv_nav_proto_send_observation(const protocol_nav_observation_payload_t *payload);
void drv_nav_proto_on_tx_complete(void);
void drv_nav_proto_get_stats(drv_nav_proto_stats_t *stats_out);

#endif /* USER_DRIVERS_DRV_NAV_PROTO_H */
