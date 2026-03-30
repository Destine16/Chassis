#ifndef USER_SERVICE_SRV_CHASSIS_H
#define USER_SERVICE_SRV_CHASSIS_H

#include "common_def.h"

/* 底盘服务层负责保存最终命令、限幅后命令和四轮目标快照。 */
void srv_chassis_init(void);
void srv_chassis_set_final_cmd(const chassis_cmd_t *cmd);
void srv_chassis_set_limited_cmd(const chassis_cmd_t *cmd);
void srv_chassis_set_actual_cmd(const chassis_cmd_t *cmd);
void srv_chassis_set_wheel_targets(const wheel_targets_t *targets);
chassis_cmd_t srv_chassis_get_final_cmd(void);
chassis_cmd_t srv_chassis_get_limited_cmd(void);
chassis_cmd_t srv_chassis_get_actual_cmd(void);
wheel_targets_t srv_chassis_get_wheel_targets(void);
void srv_chassis_stop(void);

#endif /* USER_SERVICE_SRV_CHASSIS_H */
