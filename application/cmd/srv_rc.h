#ifndef USER_SERVICE_SRV_RC_H
#define USER_SERVICE_SRV_RC_H

#include "common_def.h"
#include "drv_dbus.h"

/* 遥控器服务层负责保存最新状态、在线标志和人工接管判定。 */
void srv_rc_init(void);
void srv_rc_update_from_dbus(const drv_dbus_frame_t *frame);
void srv_rc_refresh_online(uint32_t now_ms);
rc_state_t srv_rc_get_state(void);

/* 根据左拨杆 S1 位置得到请求模式；右拨杆 S2 当前预留。 */
app_mode_t srv_rc_get_requested_mode(void);

/* 将当前启用的三个摇杆通道映射成 MANUAL 模式底盘速度命令。 */
chassis_cmd_t srv_rc_get_manual_cmd(void);

#endif /* USER_SERVICE_SRV_RC_H */
