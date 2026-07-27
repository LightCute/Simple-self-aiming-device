/**
  ******************************************************************************
  * @file    f32c_protocol.h
  * @brief   F32C brushless motor TTL serial protocol layer
  *
  *          Pure C implementation — no HAL dependencies.
  *          User must provide f32c_uart_send() externally.
  ******************************************************************************
  */
#ifndef __F32C_PROTOCOL_H__
#define __F32C_PROTOCOL_H__

#include <stdint.h>

/* ========== Frame Constants ========== */
#define F32C_FRAME_HEADER  0x7A
#define F32C_FRAME_TAIL    0x7B

/* ========== Command Words ========== */
#define F32C_CMD_MODE      0x00
#define F32C_CMD_SPEED     0x01
#define F32C_CMD_POSITION  0x02
#define F32C_CMD_ENABLE    0x06
#define F32C_CMD_QUERY     0x0E

/* ========== Control Modes ========== */
#define F32C_MODE_SPEED              0
#define F32C_MODE_POS_T_CURVE        1
#define F32C_MODE_POS_NO_T_CURVE     2
#define F32C_MODE_POS_DIRECT         3

/* ========== Query Types ========== */
#define F32C_QUERY_SPEED    0x00
#define F32C_QUERY_POSITION 0x01

/* ========== Motor IDs ========== */
#define F32C_MOTOR_X  0x01
#define F32C_MOTOR_Y  0x02

/* ========== Angle Conversion Macros ========== */
/* 10 units = 1°  →  pos = degrees × 10 */
#define F32C_DEG_TO_POS(d)  ((int32_t)((d) * 10))
#define F32C_POS_TO_DEG(p)  ((float)(p) / 10.0f)

/* ========== Feedback Data ========== */
typedef struct {
    uint8_t  motor_id;
    uint8_t  type;       /* 0x00 = speed, 0x01 = position */
    int32_t  value;      /* parsed 32-bit signed value */
    uint8_t  valid;      /* 1 = BCC check passed */
} f32c_feedback_t;

/* ========== Core Protocol Functions ========== */

/** Compute XOR checksum over first `count` bytes */
uint8_t f32c_bcc_xor(uint8_t *data, uint8_t count);

/** Enable/disable motor output (5-byte frame) */
void f32c_enable(uint8_t motor_id);

/** Set control mode (7-byte frame) */
void f32c_set_mode(uint8_t motor_id, uint16_t mode);

/** Set target speed in RPM (7-byte frame, signed 16-bit) */
void f32c_set_speed(uint8_t motor_id, int16_t speed_rpm);

/** Set target position (9-byte frame, signed 32-bit, 10 units/°) */
void f32c_set_position(uint8_t motor_id, int32_t position);

/** Query motor feedback (6-byte frame) */
void f32c_query(uint8_t motor_id, uint8_t query_type);

/* ========== Feedback Parsing ========== */

/**
  * Parse a received feedback frame.
  * @param buf  raw bytes from UART
  * @param len  number of bytes received
  * @return     parsed feedback (check .valid before using)
  */
f32c_feedback_t f32c_parse_feedback(uint8_t *buf, uint8_t len);

/* ========== User Must Implement ========== */

/** Send raw bytes via UART — implemented by the application layer */
extern void f32c_uart_send(uint8_t *data, uint8_t len);

#endif /* __F32C_PROTOCOL_H__ */
