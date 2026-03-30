#ifndef USER_CONTROL_RAMP_H
#define USER_CONTROL_RAMP_H

#include "common_def.h"

/* 一阶斜坡限制器，用于控制命令平滑。 */
typedef struct
{
    float current;
    float accel_limit;
} ramp_filter_t;

/* accel_limit 单位与输入量一致，只是再除以秒。 */
void ramp_init(ramp_filter_t *ramp, float accel_limit);
void ramp_reset(ramp_filter_t *ramp, float value);
float ramp_apply(ramp_filter_t *ramp, float target, float dt_s);

#endif /* USER_CONTROL_RAMP_H */
