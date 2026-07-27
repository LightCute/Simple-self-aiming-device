/**
  ******************************************************************************
  * @file    f32c_protocol.c
  * @brief   F32C brushless motor TTL serial protocol — implementation
  *
  *          Works with any MCU: only external dependency is f32c_uart_send().
  ******************************************************************************
  */
#include "f32c_protocol.h"

/* ---------- BCC XOR Checksum ---------- */
uint8_t f32c_bcc_xor(uint8_t *data, uint8_t count)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < count; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

/* ---------- Enable Motor (5-byte frame) ---------- */
void f32c_enable(uint8_t motor_id)
{
    uint8_t data[5] = {0x7A, motor_id, 0x06, 0, 0x7B};
    data[3] = f32c_bcc_xor(data, 3);
    f32c_uart_send(data, 5);
}

/* ---------- Set Control Mode (7-byte frame) ---------- */
void f32c_set_mode(uint8_t motor_id, uint16_t mode)
{
    uint8_t data[7] = {0x7A, motor_id, 0x00,
                       mode >> 8, mode & 0xFF,
                       0, 0x7B};
    data[5] = f32c_bcc_xor(data, 5);
    f32c_uart_send(data, 7);
}

/* ---------- Set Speed (7-byte frame) ---------- */
void f32c_set_speed(uint8_t motor_id, int16_t speed_rpm)
{
    uint8_t data[7] = {0x7A, motor_id, 0x01,
                       speed_rpm >> 8, speed_rpm & 0xFF,
                       0, 0x7B};
    data[5] = f32c_bcc_xor(data, 5);
    f32c_uart_send(data, 7);
}

/* ---------- Set Target Position (9-byte frame) ---------- */
void f32c_set_position(uint8_t motor_id, int32_t position)
{
    uint8_t data[9] = {0x7A, motor_id, 0x02,
                       position >> 24, position >> 16,
                       position >> 8,  position & 0xFF,
                       0, 0x7B};
    data[7] = f32c_bcc_xor(data, 7);
    f32c_uart_send(data, 9);
}

/* ---------- Query Feedback (6-byte frame) ---------- */
void f32c_query(uint8_t motor_id, uint8_t query_type)
{
    uint8_t data[6] = {0x7A, motor_id, 0x0E, query_type, 0, 0x7B};
    data[4] = f32c_bcc_xor(data, 4);
    f32c_uart_send(data, 6);
}

/* ---------- Parse Feedback Frame ---------- */
f32c_feedback_t f32c_parse_feedback(uint8_t *buf, uint8_t len)
{
    f32c_feedback_t fb = {0};

    /* Minimum feedback frame is 9 bytes: HDR ID TYPE DATA[4] BCC TAIL */
    if (len < 9) return fb;
    if (buf[0] != F32C_FRAME_HEADER || buf[8] != F32C_FRAME_TAIL) return fb;
    if (buf[7] != f32c_bcc_xor(buf, 7)) return fb;

    fb.valid    = 1;
    fb.motor_id = buf[1];
    fb.type     = buf[2];
    fb.value    = (int32_t)(((uint32_t)buf[3] << 24) |
                            ((uint32_t)buf[4] << 16) |
                            ((uint32_t)buf[5] << 8)  |
                             (uint32_t)buf[6]);
    return fb;
}
