/**
  ******************************************************************************
  * @file    gimbal_driver.h
  * @brief   2-axis gimbal control layer (F32C motors via configurable UART)
  ******************************************************************************
  */
#ifndef __GIMBAL_DRIVER_H__
#define __GIMBAL_DRIVER_H__

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* ========== Configurable ========== */

#define GIMBAL_BOOKMARK_MAX  8   /* 书签槽位数，增大需要更多内存 */

/* Axis identifiers */
#define GIMBAL_AXIS_X  0x01
#define GIMBAL_AXIS_Y  0x02

/* ========== Init & Sync ========== */
void gimbal_init(UART_HandleTypeDef *huart);
void gimbal_enable(void);
void gimbal_sync(void);
uint8_t gimbal_is_init_done(void);  /* 1 = init completed */

/* ========== Bookmark ========== */

/** Save current position to slot n (0..MAX-1). Returns 0 on success. */
int gimbal_save_bookmark(uint8_t slot);

/** Go to saved position in slot n. Returns 0 on success, -1 if empty. */
int gimbal_go_bookmark(uint8_t slot);

/** Get bookmark slot info. Returns 1 if slot has data, 0 if empty. */
int gimbal_get_bookmark(uint8_t slot, int32_t *x, int32_t *y);

/** Directly set bookmark slot (for flash restore). */
void gimbal_set_bookmark_raw(uint8_t slot, int32_t x, int32_t y);

/* ========== Angle Control ========== */
void gimbal_set_angle(uint8_t axis, int32_t pos);
void gimbal_move_delta(uint8_t axis, int32_t delta);  /* relative move */
int32_t gimbal_get_current(uint8_t axis);
int32_t gimbal_get_target(uint8_t axis);

/* ========== Speed ========== */
void gimbal_set_speed(int16_t rpm);
int16_t gimbal_get_speed(void);

/* ========== Soft Limits ========== */
void gimbal_set_limit(uint8_t axis, int32_t min, int32_t max);
void gimbal_disable_limit(uint8_t axis);
int  gimbal_get_limit(uint8_t axis, int32_t *min, int32_t *max);  /* returns 0 if off */

/* ========== Polling & Feedback ========== */
void gimbal_poll(void);
void gimbal_on_rx_event(uint16_t len);

/* ========== Debug Counters ========== */
uint16_t gimbal_get_poll_count(void);
uint16_t gimbal_get_rx_count(void);
uint16_t gimbal_get_cb_count(void);
uint16_t gimbal_get_short_fail(void);
uint16_t gimbal_get_hdr_fail(void);
uint16_t gimbal_get_bcc_fail(void);
uint16_t gimbal_get_type_fail(void);

#endif /* __GIMBAL_DRIVER_H__ */
