#ifndef USER_DRIVERS_DRV_DBUS_H
#define USER_DRIVERS_DRV_DBUS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * DR16 标准 18 字节 DBUS 帧解析结果。
 * 原始字段顺序固定为 ch0/ch1/ch2/ch3 + s1/s2 + 鼠标 + 键盘。
 * 当前底盘实际只使用 ch0/ch2/ch3 和 s1，其余字段先保留给后续扩展。
 */
typedef struct
{
    uint16_t ch[4]; /* ch0/ch1 = 右横/右纵，ch2/ch3 = 左横/左纵。 */
    uint8_t s1;     /* 左拨杆，当前用于 SAFE/MANUAL/AUTO 模式选择。 */
    uint8_t s2;     /* 右拨杆，当前底盘未使用。 */
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_press_l;
    uint8_t mouse_press_r;
    uint16_t key_bits;
    uint32_t sequence;
} drv_dbus_frame_t;

/* 初始化 USART3 双缓冲接收。 */
int drv_dbus_init(void);

/* 在 USART3 IDLE 中断里调用。 */
void drv_dbus_irq_handler(void);

/* 只返回最新一帧完整 DBUS，不保留旧帧。 */
bool drv_dbus_read_latest(drv_dbus_frame_t *frame_out);

#endif /* USER_DRIVERS_DRV_DBUS_H */
