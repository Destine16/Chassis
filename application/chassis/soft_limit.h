#ifndef USER_CONTROL_SOFT_LIMIT_H
#define USER_CONTROL_SOFT_LIMIT_H

#include "common_def.h"

/* 对称限幅工具。 */
float soft_limit_symmetric(float value, float limit);

/* 按 robot_def 中的统一上限约束底盘三自由度命令。 */
void soft_limit_chassis_cmd(chassis_cmd_t *cmd);

#endif /* USER_CONTROL_SOFT_LIMIT_H */
