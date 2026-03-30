#ifndef USER_COMMON_COMMON_MATH_H
#define USER_COMMON_COMMON_MATH_H

#include <math.h>
#include <stdint.h>

/* 浮点对称限幅。 */
static inline float common_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return value;
}

static inline int16_t common_clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value)
        return min_value;
    if (value > max_value)
        return max_value;
    return (int16_t)value;
}

/* 死区处理，只负责截断，不负责重新缩放。 */
static inline float common_apply_deadband(float value, float deadband)
{
    if (fabsf(value) <= deadband)
        return 0.0f;
    return value;
}

/* 统一封装浮点绝对值，调用点更直观。 */
static inline float common_absf(float value)
{
    return fabsf(value);
}

#endif /* USER_COMMON_COMMON_MATH_H */
