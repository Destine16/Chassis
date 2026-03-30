#include "ramp.h"

#include "common_math.h"

void ramp_init(ramp_filter_t *ramp, float accel_limit)
{
    ramp->current = 0.0f;
    ramp->accel_limit = accel_limit;
}

void ramp_reset(ramp_filter_t *ramp, float value)
{
    ramp->current = value;
}

float ramp_apply(ramp_filter_t *ramp, float target, float dt_s)
{
    float max_step = ramp->accel_limit * dt_s;
    float delta = target - ramp->current;

    /* 每一拍最多只允许变化 max_step。 */
    delta = common_clampf(delta, -max_step, max_step);
    ramp->current += delta;
    return ramp->current;
}
