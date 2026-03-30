#include "crc16.h"

uint16_t crc16_compute(const uint8_t *data, size_t length)
{
    size_t index = 0U;
    uint16_t crc = 0xFFFFU;

    /* 按位计算即可满足当前串口吞吐，不必额外引入查表。 */
    while (index < length)
    {
        uint8_t bit = 0U;

        crc ^= (uint16_t)((uint16_t)data[index++] << 8U);

        while (bit < 8U)
        {
            if ((crc & 0x8000U) != 0U)
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            else
                crc <<= 1U;
            bit++;
        }
    }

    return crc;
}
