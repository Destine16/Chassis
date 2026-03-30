#include "bsp_time.h"

#include "stm32f4xx_hal.h"

uint32_t bsp_time_get_ms(void)
{
    return HAL_GetTick();
}

bool bsp_time_is_expired(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms)
{
    /* 采用无符号减法比较，tick 回绕后结果仍然成立。 */
    return (now_ms - last_ms) > timeout_ms;
}
