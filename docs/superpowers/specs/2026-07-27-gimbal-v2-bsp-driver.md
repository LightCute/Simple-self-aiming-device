# 2D Gimbal (F32C) — v2 BSP Driver Integration

**Date**: 2026-07-27
**Status**: approved
**Branch**: `feature/gimbal-v2`

## Overview

Transplant the F32C gimbal driver from reference project into BSP layer.
Start minimal: init + poll + read position. No OOP wrapper yet — App mode
calls BSP C functions directly.

Reference: `D:\Project\TICup2026\gimbal\gimbal_brushless_motor\stm32_ctrl\Core`

## Architecture

```
BSP/
  f32c_protocol.c/h    ← copy from reference (pure protocol, zero HAL deps)
  gimbal_driver.c/h    ← copy from reference gimbal.c/h, renamed

App/
  mode_gimbal_test.c/h ← new AppMode, calls gimbal_driver directly
  main.c               ← 4 changes (include, init calls, mode register, DMA callback)
```

No Adapters layer. No OOP struct. Pure C static variables in driver.
This is Phase 1 — verify hardware works, then wrap with OOP in Phase 2.

## Files

### BSP/f32c_protocol.c/h (copy, zero changes)

- All 7 protocol functions: `f32c_bcc_xor`, `f32c_enable`, `f32c_set_mode`,
  `f32c_set_speed`, `f32c_set_position`, `f32c_query`, `f32c_parse_feedback`
- All constants: `F32C_FRAME_HEADER/TAIL`, `F32C_CMD_*`, `F32C_MODE_*`,
  `F32C_MOTOR_X/Y`, `F32C_DEG_TO_POS/POS_TO_DEG`
- `f32c_feedback_t` struct for parsed feedback
- Declares `extern void f32c_uart_send(uint8_t *data, uint8_t len)` — user implements

### BSP/gimbal_driver.c/h (copy from reference gimbal.c/h, renamed)

Changes from reference:
- Rename files to `gimbal_driver` (avoid naming conflicts)
- Move `HAL_UARTEx_RxEventCallback` from reference main.c into driver (filter by `huart->Instance == UART4`, dispatch to `gimbal_on_rx_event`)
- Remove `g_poll_tick` dependency (main loop ~10ms natural tick replaces TIM6 flag)
- Keep all other logic unchanged: static variables, init sequence, poll state
  machine, sync, bookmarks, limits, speed control, debug counters

Exported API (17 functions):
```
gimbal_init, gimbal_enable, gimbal_sync, gimbal_is_init_done
gimbal_set_angle, gimbal_move_delta, gimbal_get_current, gimbal_get_target
gimbal_save_bookmark, gimbal_go_bookmark, gimbal_get_bookmark,
gimbal_set_bookmark_raw
gimbal_set_speed, gimbal_get_speed
gimbal_set_limit, gimbal_disable_limit, gimbal_get_limit
gimbal_poll, gimbal_on_rx_event
gimbal_get_poll_count, gimbal_get_rx_count, gimbal_get_cb_count,
gimbal_get_short_fail, gimbal_get_hdr_fail, gimbal_get_bcc_fail,
gimbal_get_type_fail
```

### App/mode_gimbal_test.c (new)

Follows `mode_chassis_test.c` pattern:
- `on_enter`: call `gimbal_init(&huart4)`, then `gimbal_sync()`
- `on_isr`: no-op (poll runs in on_ui)
- `on_ui`: call `gimbal_poll()`, display X/Y current/target + speed + sync on OLED
- `on_command`: handled by main.c (CMD_NEXT for mode switch)

### App/main.c (4 modifications)

1. **Includes**: add `"gimbal_driver.h"` and `"mode_gimbal_test.h"`
2. **Init calls**: after `MX_UART8_Init()`, add `MX_DMA_Init()` and `MX_UART4_Init()`
   (user manually adds these — CubeMX coordination)
3. **Mode register**: `g_modes[] = {&mode_lap_run, &mode_gimbal_test}`, `MODE_COUNT=2`
4. **DMA callback**: define `HAL_UARTEx_RxEventCallback` in main.c USER CODE 4,
   dispatch to `gimbal_on_rx_event(Size)` when `huart->Instance == UART4`

## CubeMX Dependencies

- UART4 (PA11=RX, PA12=TX, 115200, 8N1)
- DMA1_Stream0 for UART4_RX (normal mode)
- NVIC: UART4_IRQn, DMA1_Stream0_IRQn
- User must: add `MX_UART4_Init()` + `MX_DMA_Init()` to App/main.c
- User must: add `Core/Src/dma.c` to Keil project

## Polling Flow

Main loop (~10ms per iteration):
```
g_modes[g_cur_mode]->on_ui()   → gimbal_poll()
  ├── process pending ops (enable, set_speed)  [1 step per call]
  ├── drive sync state machine if active       [1 step per call]
  └── poll state machine:
      IDLE → SEND_X → SEND_Y → QUERY → IDLE    [1 step per call, 4-call cycle]
```

DMA RX (interrupt context):
```
UART4 IDLE interrupt → HAL_UART_IRQHandler
  → HAL_UARTEx_RxEventCallback
    → gimbal_on_rx_event(Size)
      → validate frame → update g_current[] → restart DMA
```

## What's NOT in this phase

- No OOP adapter (Gimbal struct with fn pointers)
- No Logger injection (use printf for now)
- No serial commands for gimbal control (just mode switch)
- No HAL/gimbal.h abstract interface
