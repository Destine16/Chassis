#ifndef USER_CONTROL_OMNI_CHASSIS_KINEMATICS_H
#define USER_CONTROL_OMNI_CHASSIS_KINEMATICS_H

#include "common_def.h"

/*
 * 四全向轮 X 布局逆运动学。
 * 当前按“正方形对角线为 425 mm”的结构参数建模，
 * 并由对角线换算得到正方形边长，再取半长和半宽。
 * 坐标约定: +vx 向前，+vy 向左，+wz 逆时针。
 */
void omni_chassis_kinematics_inverse(const chassis_cmd_t *cmd, float wheel_speed_ref_radps[COMMON_WHEEL_COUNT]);
void omni_chassis_kinematics_forward(const float wheel_speed_radps[COMMON_WHEEL_COUNT], chassis_cmd_t *cmd_out);
float omni_chassis_desaturate_wheel_speeds(float wheel_speed_radps[COMMON_WHEEL_COUNT], float max_abs_wheel_speed_radps);

#endif /* USER_CONTROL_OMNI_CHASSIS_KINEMATICS_H */
