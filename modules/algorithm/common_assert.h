#ifndef USER_COMMON_COMMON_ASSERT_H
#define USER_COMMON_COMMON_ASSERT_H

#include <stdint.h>

/*
 * 用户层统一断言入口。
 * 这里不做复杂打印，断言失败后直接转到 CubeMX 生成的 Error_Handler。
 */
void common_assert_failed(const char *expr, const char *file, uint32_t line);

#define COMMON_ASSERT(expr) \
    do { \
        if (!(expr)) \
            common_assert_failed(#expr, __FILE__, (uint32_t)__LINE__); \
    } while (0)

#endif /* USER_COMMON_COMMON_ASSERT_H */
