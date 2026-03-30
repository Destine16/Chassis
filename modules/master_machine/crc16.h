#ifndef USER_MIDDLEWARE_CRC16_H
#define USER_MIDDLEWARE_CRC16_H

#include <stddef.h>
#include <stdint.h>

/* 导航串口协议使用的 CRC16-CCITT 计算。 */
uint16_t crc16_compute(const uint8_t *data, size_t length);

#endif /* USER_MIDDLEWARE_CRC16_H */
