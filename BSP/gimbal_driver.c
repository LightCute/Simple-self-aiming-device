/**
  ******************************************************************************
  * @file    gimbal_driver.c
  * @brief   2-axis gimbal control layer — implementation
  ******************************************************************************
  */
#include "gimbal_driver.h"
#include "f32c_protocol.h"

/* ================================================================ */
/*  Injected UART handle (set by gimbal_init)                        */
/* ================================================================ */
static UART_HandleTypeDef *g_huart = NULL;

/* ================================================================ */
/*  DMA RX buffer                                                    */
/* ================================================================ */
#define DMA_RX_BUF_SIZE   32
static uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];

/* ================================================================ */
/*  Gimbal state                                                     */
/* ================================================================ */
static int32_t  g_current[2]  = {0, 0}; /* [0]=X, [1]=Y, cached feedback */
static int32_t  g_target[2]   = {0, 0}; /* last-set target values */
static int32_t  g_bookmark[GIMBAL_BOOKMARK_MAX][2];  /* multi-slot bookmarks */
static uint8_t  g_bm_valid[GIMBAL_BOOKMARK_MAX];     /* 1 = slot in use */
static uint8_t  g_enabled     = 0;
static uint8_t  g_synced      = 0;
static uint8_t  g_initialized = 0;  /* 1 = gimbal_init() completed */
static uint8_t  poll_axis     = F32C_MOTOR_X;
/* ---- soft limits ---- */
static int32_t  g_limit_min[2] = {0, 0};
static int32_t  g_limit_max[2] = {0, 0};
static uint8_t  g_limit_on[2]  = {0, 0};   /* 1 = limit active */
/* ---- debug counters ---- */
static uint16_t g_poll_count  = 0;
static uint16_t g_rx_count    = 0;
static uint16_t g_cb_count    = 0;
static uint16_t g_short_fail  = 0;
static uint16_t g_hdr_fail    = 0;
static uint16_t g_bcc_fail    = 0;
static uint16_t g_type_fail   = 0;
/* ---- poll state machine ---- */
enum { PS_IDLE, PS_SEND_X, PS_SEND_Y, PS_QUERY };
static uint8_t g_poll_state = PS_IDLE;

/* ---- sync state machine ---- */
enum { SS_IDLE, SS_SEND_X, SS_WAIT_X, SS_SEND_Y, SS_WAIT_Y, SS_DONE };
static uint8_t  g_sync_state = SS_IDLE;
static uint8_t  g_sync_retry = 0;
static uint16_t g_sync_rx_snapshot = 0;

/* ---- deferred ops (processed by gimbal_poll, one step per tick) ---- */
enum { OP_NONE, OP_ENABLE, OP_SET_SPEED };
static uint8_t  g_pending_op = OP_NONE;
static uint8_t  g_op_step    = 0;
static int16_t  g_speed      = 50;

/* ================================================================ */
/*  f32c_protocol dependency: UART send                              */
/* ================================================================ */
void f32c_uart_send(uint8_t *data, uint8_t len)
{
    HAL_UART_Transmit(g_huart, data, len, 10);
}

/* ================================================================ */
/*  DMA RX helpers                                                   */
/* ================================================================ */
static void dma_rx_start(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(g_huart, dma_rx_buf, DMA_RX_BUF_SIZE);
}

static void sync_on_rx(uint8_t axis, int32_t val);  /* forward decl */

/* ================================================================ */
/*  Public: called from HAL_UARTEx_RxEventCallback                   */
/* ================================================================ */
void gimbal_on_rx_event(uint16_t len)
{
    g_cb_count++;

    if (len < 9)                        { g_short_fail++; dma_rx_start(); return; }
    if (dma_rx_buf[0] != 0x7A || dma_rx_buf[8] != 0x7B)
                                        { g_hdr_fail++;   dma_rx_start(); return; }
    if (dma_rx_buf[7] != f32c_bcc_xor(dma_rx_buf, 7))
                                        { g_bcc_fail++;   dma_rx_start(); return; }
    if (dma_rx_buf[2] != F32C_QUERY_POSITION)
                                        { g_type_fail++;  dma_rx_start(); return; }

    g_rx_count++;
    {
        int32_t val = (int32_t)((dma_rx_buf[3] << 24) | (dma_rx_buf[4] << 16) |
                                (dma_rx_buf[5] << 8)  |  dma_rx_buf[6]);
        uint8_t id = dma_rx_buf[1];
        if      (id == F32C_MOTOR_X) g_current[0] = val;
        else if (id == F32C_MOTOR_Y) g_current[1] = val;
        /* if sync is in progress, notify it */
        if (g_sync_state != SS_IDLE && g_sync_state != SS_DONE) {
            sync_on_rx(id, val);
        }
    }
    dma_rx_start();
}

/* ================================================================ */
/*  Init (power-up, one-shot)                                       */
/* ================================================================ */
void gimbal_init(UART_HandleTypeDef *huart)
{
    g_huart = huart;

    HAL_Delay(1500);                        /* 1. wait for motor boot */

    f32c_enable(F32C_MOTOR_X);              /* 2. enable both motors */
    HAL_Delay(1);
    f32c_enable(F32C_MOTOR_Y);
    HAL_Delay(1);

    f32c_set_mode(F32C_MOTOR_X, F32C_MODE_POS_T_CURVE);
    HAL_Delay(1);
    f32c_set_mode(F32C_MOTOR_Y, F32C_MODE_POS_T_CURVE);
    HAL_Delay(1);

    f32c_set_speed(F32C_MOTOR_X, 50);       /* 4. travel speed */
    HAL_Delay(10);
    f32c_set_speed(F32C_MOTOR_Y, 50);
    HAL_Delay(10);

    g_enabled = 1;
    g_initialized = 1;

    /* Y-axis default soft limit: -120° ~ +120° */
    gimbal_set_limit(GIMBAL_AXIS_Y, -1200, 1200);

    dma_rx_start();
}

uint8_t gimbal_is_init_done(void) { return g_initialized; }

/* ================================================================ */
/*  Re-enable                                                       */
/* ================================================================ */
void gimbal_enable(void)
{
    g_speed = 50;
    g_pending_op = OP_ENABLE;
    g_op_step = 0;
}

static void process_enable_step(void)
{
    switch (g_op_step) {
    case 0: f32c_enable(F32C_MOTOR_X);                             g_op_step = 1; break;
    case 1: f32c_enable(F32C_MOTOR_Y);                             g_op_step = 2; break;
    case 2: f32c_set_mode(F32C_MOTOR_X, F32C_MODE_POS_T_CURVE);   g_op_step = 3; break;
    case 3: f32c_set_mode(F32C_MOTOR_Y, F32C_MODE_POS_T_CURVE);   g_op_step = 4; break;
    case 4: f32c_set_speed(F32C_MOTOR_X, g_speed);                g_op_step = 5; break;
    case 5:
        f32c_set_speed(F32C_MOTOR_Y, g_speed);
        g_enabled    = 1;
        g_synced     = 0;
        g_poll_state = PS_IDLE;
        g_sync_state = SS_IDLE;
        g_pending_op = OP_NONE;
        break;
    }
}

/* ================================================================ */
/*  Sync: non-blocking state machine — query new position each call  */
/* ================================================================ */
void gimbal_sync(void)
{
    if (!g_enabled) return;
    if (g_sync_state == SS_IDLE) {
        g_sync_state = SS_SEND_X;
        g_sync_retry = 0;
    }
}

/* internal: advance sync state machine one step (called from poll) */
static void sync_step(void)
{
    switch (g_sync_state) {
    case SS_IDLE:
    case SS_DONE:
        return;

    case SS_SEND_X:
        g_sync_rx_snapshot = g_rx_count;
        f32c_query(F32C_MOTOR_X, F32C_QUERY_POSITION);
        g_sync_state = SS_WAIT_X;
        g_sync_retry = 0;
        break;

    case SS_WAIT_X:
        g_sync_retry++;
        if (g_sync_retry >= 10) {
            /* timeout → move to Y */
            g_sync_state = SS_SEND_Y;
        } else {
            f32c_query(F32C_MOTOR_X, F32C_QUERY_POSITION);
        }
        break;

    case SS_SEND_Y:
        g_sync_rx_snapshot = g_rx_count;
        f32c_query(F32C_MOTOR_Y, F32C_QUERY_POSITION);
        g_sync_state = SS_WAIT_Y;
        g_sync_retry = 0;
        break;

    case SS_WAIT_Y:
        g_sync_retry++;
        if (g_sync_retry >= 10) {
            g_synced = 1;
            g_sync_state = SS_DONE;
        } else {
            f32c_query(F32C_MOTOR_Y, F32C_QUERY_POSITION);
        }
        break;

    default: break;
    }
}

/* called from gimbal_on_rx_event when sync is active and response arrives */
static void sync_on_rx(uint8_t axis, int32_t val)
{
    if (g_sync_state == SS_WAIT_X && axis == F32C_MOTOR_X) {
        g_target[0] = val;
        g_sync_state = SS_SEND_Y;
    } else if (g_sync_state == SS_WAIT_Y && axis == F32C_MOTOR_Y) {
        g_target[1] = val;
        g_synced = 1;
        g_sync_state = SS_DONE;
    }
}

/* ================================================================ */
/*  Bookmark: multi-slot save / go / query                           */
/* ================================================================ */
int gimbal_save_bookmark(uint8_t slot)
{
    if (slot >= GIMBAL_BOOKMARK_MAX) return -1;
    g_bookmark[slot][0] = g_current[0];
    g_bookmark[slot][1] = g_current[1];
    g_bm_valid[slot] = 1;
    return 0;
}

int gimbal_go_bookmark(uint8_t slot)
{
    if (slot >= GIMBAL_BOOKMARK_MAX) return -1;
    if (!g_bm_valid[slot]) return -1;
    if (!g_enabled) return -1;
    g_target[0] = g_bookmark[slot][0];
    g_target[1] = g_bookmark[slot][1];
    f32c_set_position(F32C_MOTOR_X, g_bookmark[slot][0]);
    f32c_set_position(F32C_MOTOR_Y, g_bookmark[slot][1]);
    return 0;
}

int gimbal_get_bookmark(uint8_t slot, int32_t *x, int32_t *y)
{
    if (slot >= GIMBAL_BOOKMARK_MAX || !g_bm_valid[slot]) return 0;
    *x = g_bookmark[slot][0];
    *y = g_bookmark[slot][1];
    return 1;
}

/* for flash restore: directly write bookmark slot */
void gimbal_set_bookmark_raw(uint8_t slot, int32_t x, int32_t y)
{
    if (slot >= GIMBAL_BOOKMARK_MAX) return;
    g_bookmark[slot][0] = x;
    g_bookmark[slot][1] = y;
    g_bm_valid[slot] = 1;
}

/* ================================================================ */
/*  Set angle                                                        */
/* ================================================================ */
void gimbal_set_angle(uint8_t axis, int32_t pos)
{
    if (!g_enabled) return;
    /* clamp to soft limit */
    uint8_t idx = axis - 1;
    if (g_limit_on[idx]) {
        if (pos < g_limit_min[idx]) pos = g_limit_min[idx];
        if (pos > g_limit_max[idx]) pos = g_limit_max[idx];
    }
    f32c_set_position(axis, pos);
    g_target[idx] = pos;
}

void gimbal_move_delta(uint8_t axis, int32_t delta)
{
    if (!g_enabled) return;
    int32_t new_pos = g_target[axis - 1] + delta;
    gimbal_set_angle(axis, new_pos);
}

/* ================================================================ */
/*  Soft limits                                                      */
/* ================================================================ */
void gimbal_set_limit(uint8_t axis, int32_t min, int32_t max)
{
    if (min > max) return;
    uint8_t idx = axis - 1;
    g_limit_min[idx] = min;
    g_limit_max[idx] = max;
    g_limit_on[idx]  = 1;
}

void gimbal_disable_limit(uint8_t axis)
{
    g_limit_on[axis - 1] = 0;
}

int gimbal_get_limit(uint8_t axis, int32_t *min, int32_t *max)
{
    uint8_t idx = axis - 1;
    if (!g_limit_on[idx]) return 0;
    *min = g_limit_min[idx];
    *max = g_limit_max[idx];
    return 1;
}

/* ================================================================ */
/*  Speed                                                            */
/* ================================================================ */

void gimbal_set_speed(int16_t rpm)
{
    g_speed = rpm;
    g_pending_op = OP_SET_SPEED;
    g_op_step = 0;
}

static void process_speed_step(void)
{
    switch (g_op_step) {
    case 0:
        f32c_set_speed(F32C_MOTOR_X, g_speed);
        g_op_step = 1;
        break;
    case 1:
        f32c_set_speed(F32C_MOTOR_Y, g_speed);
        g_pending_op = OP_NONE;
        break;
    }
}

int16_t gimbal_get_speed(void)
{
    return g_speed;
}

/* ================================================================ */
/*  Get cached current / target                                      */
/* ================================================================ */
int32_t gimbal_get_current(uint8_t axis)
{
    return g_current[axis - 1];
}

int32_t gimbal_get_target(uint8_t axis)
{
    return g_target[axis - 1];
}

/* ================================================================ */
/*  Polling — non-blocking state machine (one step per call)         */
/*  Call from main loop every ~10ms for aligned frame timing.        */
/* ================================================================ */
void gimbal_poll(void)
{
    if (!g_enabled) return;

    /* Process deferred ops first (one step per call) */
    if (g_pending_op == OP_ENABLE)    { process_enable_step(); return; }
    if (g_pending_op == OP_SET_SPEED) { process_speed_step();  return; }

    /* Drive sync state machine if active */
    if (g_sync_state != SS_IDLE && g_sync_state != SS_DONE) {
        sync_step();
    }

    g_poll_count++;

    switch (g_poll_state) {
    case PS_IDLE:
        if (g_synced) g_poll_state = PS_SEND_X;
        else          g_poll_state = PS_QUERY;
        break;

    case PS_SEND_X:
        f32c_set_position(F32C_MOTOR_X, g_target[0]);
        g_poll_state = PS_SEND_Y;
        break;

    case PS_SEND_Y:
        f32c_set_position(F32C_MOTOR_Y, g_target[1]);
        g_poll_state = PS_QUERY;
        break;

    case PS_QUERY:
        f32c_query(poll_axis, F32C_QUERY_POSITION);
        poll_axis = (poll_axis == F32C_MOTOR_X) ? F32C_MOTOR_Y : F32C_MOTOR_X;
        g_poll_state = PS_IDLE;
        break;

    default: break;
    }
}

/* ================================================================ */
/*  Debug counters                                                   */
/* ================================================================ */
uint16_t gimbal_get_poll_count(void) { return g_poll_count; }
uint16_t gimbal_get_rx_count(void)   { return g_rx_count;   }
uint16_t gimbal_get_cb_count(void)   { return g_cb_count;   }
uint16_t gimbal_get_short_fail(void) { return g_short_fail; }
uint16_t gimbal_get_hdr_fail(void)   { return g_hdr_fail;   }
uint16_t gimbal_get_bcc_fail(void)   { return g_bcc_fail;   }
uint16_t gimbal_get_type_fail(void)  { return g_type_fail;  }

/* ================================================================ */
/*  HAL DMA RX callback (overrides __weak default)                   */
/*  Dispatches to gimbal_on_rx_event when UART4 data arrives.        */
/* ================================================================ */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4) {
        gimbal_on_rx_event(Size);
    }
}
