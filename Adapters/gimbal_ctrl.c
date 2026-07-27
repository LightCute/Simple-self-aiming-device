/**
  ******************************************************************************
  * @file    gimbal_ctrl.c
  * @brief   Gimbal adapter — wraps BSP gimbal_driver into OOP struct
  *
  *          Thin wrapper: each function pointer forwards to the corresponding
  *          BSP gimbal_driver C function. All state lives inside gimbal_driver;
  *          this adapter provides the OOP interface for App layer consumption.
  ******************************************************************************
  */
#include "Adapters/gimbal_ctrl.h"
#include "BSP/gimbal_driver.h"

/* ================================================================ */
/*  Static wrappers — forward to BSP gimbal_driver                    */
/* ================================================================ */
static void gc_init(void *self, struct __UART_HandleTypeDef *huart)
{
    (void)self;
    gimbal_init((UART_HandleTypeDef *)huart);
}

static void gc_poll(void *self)
{
    (void)self;
    gimbal_poll();
}

static void gc_enable(void *self)
{
    (void)self;
    gimbal_enable();
}

static void gc_sync(void *self)
{
    (void)self;
    gimbal_sync();
}

static uint8_t gc_is_init_done(void *self)
{
    (void)self;
    return gimbal_is_init_done();
}

static void gc_set_angle(void *self, uint8_t axis, int32_t pos)
{
    (void)self;
    gimbal_set_angle(axis, pos);
}

static void gc_move_delta(void *self, uint8_t axis, int32_t delta)
{
    (void)self;
    gimbal_move_delta(axis, delta);
}

static int32_t gc_get_current(void *self, uint8_t axis)
{
    (void)self;
    return gimbal_get_current(axis);
}

static int32_t gc_get_target(void *self, uint8_t axis)
{
    (void)self;
    return gimbal_get_target(axis);
}

static void gc_set_speed(void *self, int16_t rpm)
{
    (void)self;
    gimbal_set_speed(rpm);
}

static int16_t gc_get_speed(void *self)
{
    (void)self;
    return gimbal_get_speed();
}

static int gc_save_bookmark(void *self, uint8_t slot)
{
    (void)self;
    return gimbal_save_bookmark(slot);
}

static int gc_go_bookmark(void *self, uint8_t slot)
{
    (void)self;
    return gimbal_go_bookmark(slot);
}

static int gc_get_bookmark(void *self, uint8_t slot, int32_t *x, int32_t *y)
{
    (void)self;
    return gimbal_get_bookmark(slot, x, y);
}

static void gc_set_limit(void *self, uint8_t axis, int32_t min, int32_t max)
{
    (void)self;
    gimbal_set_limit(axis, min, max);
}

static void gc_disable_limit(void *self, uint8_t axis)
{
    (void)self;
    gimbal_disable_limit(axis);
}

/* ================================================================ */
/*  Global instance                                                  */
/* ================================================================ */
Gimbal g_gimbal_ctrl = {
    .init          = gc_init,
    .poll          = gc_poll,
    .enable        = gc_enable,
    .sync          = gc_sync,
    .is_init_done  = gc_is_init_done,
    .set_angle     = gc_set_angle,
    .move_delta    = gc_move_delta,
    .get_current   = gc_get_current,
    .get_target    = gc_get_target,
    .set_speed     = gc_set_speed,
    .get_speed     = gc_get_speed,
    .save_bookmark = gc_save_bookmark,
    .go_bookmark   = gc_go_bookmark,
    .get_bookmark  = gc_get_bookmark,
    .set_limit     = gc_set_limit,
    .disable_limit = gc_disable_limit,
};
