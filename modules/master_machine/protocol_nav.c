#include "protocol_nav.h"

#include <string.h>

#include "crc16.h"

enum
{
    NAV_HEADER_SIZE = 4U,
    NAV_CRC_SIZE = 2U,
    NAV_TOTAL_FRAME_SIZE = NAV_HEADER_SIZE + APP_CFG_NAV_OBSERVATION_PAYLOAD_SIZE + NAV_CRC_SIZE
};

static void protocol_nav_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void protocol_nav_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void protocol_nav_write_f32_le(uint8_t *data, float value)
{
    uint32_t raw = 0U;
    memcpy(&raw, &value, sizeof(raw));
    protocol_nav_write_u32_le(data, raw);
}

uint16_t protocol_nav_encode_observation(uint8_t seq, const protocol_nav_observation_payload_t *payload, uint8_t *buffer, uint16_t buffer_size)
{
    uint16_t crc;

    if (buffer_size < NAV_TOTAL_FRAME_SIZE)
        return 0U;

    buffer[0] = APP_CFG_NAV_PROTO_SOF0;
    buffer[1] = APP_CFG_NAV_PROTO_SOF1;
    buffer[2] = seq;
    buffer[3] = (uint8_t)APP_CFG_NAV_OBSERVATION_PAYLOAD_SIZE;

    protocol_nav_write_u32_le(&buffer[4], payload->t_ms);
    protocol_nav_write_f32_le(&buffer[8], payload->w_fl);
    protocol_nav_write_f32_le(&buffer[12], payload->w_fr);
    protocol_nav_write_f32_le(&buffer[16], payload->w_rl);
    protocol_nav_write_f32_le(&buffer[20], payload->w_rr);
    protocol_nav_write_f32_le(&buffer[24], payload->gyro_x);
    protocol_nav_write_f32_le(&buffer[28], payload->gyro_y);
    protocol_nav_write_f32_le(&buffer[32], payload->gyro_z);
    protocol_nav_write_f32_le(&buffer[36], payload->acc_x);
    protocol_nav_write_f32_le(&buffer[40], payload->acc_y);
    protocol_nav_write_f32_le(&buffer[44], payload->acc_z);
    protocol_nav_write_u16_le(&buffer[48], payload->flags);

    crc = crc16_compute(buffer, NAV_HEADER_SIZE + APP_CFG_NAV_OBSERVATION_PAYLOAD_SIZE);
    protocol_nav_write_u16_le(&buffer[50], crc);

    return NAV_TOTAL_FRAME_SIZE;
}
