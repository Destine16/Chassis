#ifndef USER_BSP_BSP_TIME_H
#define USER_BSP_BSP_TIME_H

#include <stdbool.h>
#include <stdint.h>

/* 获取系统毫秒节拍，底层直接使用 HAL_GetTick。 */
uint32_t bsp_time_get_ms(void);

/* 统一封装超时判断，利用无符号减法自动处理 tick 回绕。 */
bool bsp_time_is_expired(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms);

#endif /* USER_BSP_BSP_TIME_H */
