# Gimbal v2 BSP Driver — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Transplant F32C gimbal driver from reference into BSP layer as pure C modules.

**Architecture:** BSP → App direct. No OOP adapter. `f32c_protocol` (pure protocol) +
`gimbal_driver` (C driver with static state) → `mode_gimbal_test` (AppMode).  Phase 1 is
init + poll + read position on OLED.

**Tech Stack:** STM32H7 HAL, F32C brushless motor protocol, UART4 DMA RX

**Ref:** `docs/superpowers/specs/2026-07-27-gimbal-v2-bsp-driver.md`

## Global Constraints

- BSP files copied from reference with minimal changes (rename only, plus 2 structural moves)
- No OOP struct — pure C static variables in gimbal_driver
- No HAL/gimbal.h abstract interface (Phase 2)
- No Logger injection — use printf for now
- `HAL_UARTEx_RxEventCallback` lives in gimbal_driver.c (not in main.c)
- Follow `mode_chassis_test.c` pattern for AppMode
- CubeMX-generated files not modified

---

### Task 1: Copy f32c_protocol.c/h from reference

**Files:**
- Create: `BSP/f32c_protocol.c`
- Create: `BSP/f32c_protocol.h`

**Interfaces:**
- Produces: `f32c_bcc_xor`, `f32c_enable`, `f32c_set_mode`, `f32c_set_speed`,
  `f32c_set_position`, `f32c_query`, `f32c_parse_feedback`, `f32c_feedback_t`
- Produces: `F32C_CMD_*`, `F32C_MODE_*`, `F32C_MOTOR_X/Y`, `F32C_DEG_TO_POS`, `F32C_POS_TO_DEG`
- Requires: `extern void f32c_uart_send(uint8_t *data, uint8_t len)` — provided by Task 2

- [ ] **Step 1: Copy header**

```bash
cp "D:/Project/TICup2026/gimbal/gimbal_brushless_motor/stm32_ctrl/Core/Inc/f32c_protocol.h" \
   "d:/Project/TICup2026/H7-HW/H7-HW/BSP/f32c_protocol.h"
```

- [ ] **Step 2: Copy source**

```bash
cp "D:/Project/TICup2026/gimbal/gimbal_brushless_motor/stm32_ctrl/Core/Src/f32c_protocol.c" \
   "d:/Project/TICup2026/H7-HW/H7-HW/BSP/f32c_protocol.c"
```

- [ ] **Step 3: Verify**

```bash
ls -la BSP/f32c_protocol.*
```

Expected: ~2.8KB header, ~2.7KB source. Files byte-identical to reference.

- [ ] **Step 4: Commit**

```bash
git add BSP/f32c_protocol.c BSP/f32c_protocol.h
git commit -m "feat: copy f32c_protocol from reference (pure protocol layer)"
```

---

### Task 2: Copy gimbal.c/h → gimbal_driver.c/h (with 3 changes)

**Files:**
- Create: `BSP/gimbal_driver.c`
- Create: `BSP/gimbal_driver.h`

**Interfaces:**
- Consumes: `BSP/f32c_protocol.h` (Task 1)
- Produces: `gimbal_init`, `gimbal_enable`, `gimbal_sync`, `gimbal_is_init_done`,
  `gimbal_set_angle`, `gimbal_move_delta`, `gimbal_get_current`, `gimbal_get_target`,
  `gimbal_save_bookmark`, `gimbal_go_bookmark`, `gimbal_get_bookmark`,
  `gimbal_set_bookmark_raw`, `gimbal_set_speed`, `gimbal_get_speed`,
  `gimbal_set_limit`, `gimbal_disable_limit`, `gimbal_get_limit`,
  `gimbal_poll`, `gimbal_on_rx_event`,
  `gimbal_get_poll_count`, `gimbal_get_rx_count`, `gimbal_get_cb_count`,
  `gimbal_get_short_fail`, `gimbal_get_hdr_fail`, `gimbal_get_bcc_fail`,
  `gimbal_get_type_fail`
- Provides: `f32c_uart_send` implementation (extern dependency from Task 1)
- Provides: `HAL_UARTEx_RxEventCallback` override (dispatches to `gimbal_on_rx_event`)

**Changes from reference gimbal.c:**
1. Rename files: `gimbal.c/h` → `gimbal_driver.c/h`
2. Remove `g_poll_tick` declaration (line 60 of reference gimbal.h) — not needed
3. Move `HAL_UARTEx_RxEventCallback` from reference main.c into `gimbal_driver.c`
   (was in main.c:259-265, now in driver for self-containment)

- [ ] **Step 1: Read reference gimbal.c and gimbal.h**

Read both files from `D:\Project\TICup2026\gimbal\gimbal_brushless_motor\stm32_ctrl\Core\Inc\gimbal.h`
and `Core\Src\gimbal.c`. Full content is needed for the copy.

- [ ] **Step 2: Write BSP/gimbal_driver.h**

Copy reference `gimbal.h` with these edits:

1. Change include guard: `__GIMBAL_H__` → `__GIMBAL_DRIVER_H__`
2. Remove the `extern volatile uint8_t g_poll_tick;` line (line 61 in reference)

Everything else stays identical — all 17 function declarations, axis constants,
bookmark max, etc.

- [ ] **Step 3: Write BSP/gimbal_driver.c**

Copy reference `gimbal.c` with these edits:

1. Change `#include "gimbal.h"` to `#include "gimbal_driver.h"`
2. Remove the `volatile uint8_t g_poll_tick = 0;` line (reference line 338)
3. At end of file, add `HAL_UARTEx_RxEventCallback`:

```c
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
```

All other code stays byte-identical to reference: static variables (lines 13-58),
`f32c_uart_send` (lines 63-66), `dma_rx_start` (lines 71-74), `sync_on_rx`
(lines 76, 231-241), `gimbal_on_rx_event` (lines 81-106), `gimbal_init`
(lines 111-139), `gimbal_is_init_done` (line 141), `gimbal_enable` (lines 146-151),
`process_enable_step` (lines 153-170), `gimbal_sync` (lines 175-182),
`sync_step` (lines 185-228), bookmark functions (lines 246-282),
angle control (lines 287-305), soft limits (lines 310-331),
speed control (lines 340-364), getters (lines 369-377),
`gimbal_poll` (lines 386-425), debug counters (lines 430-436).

- [ ] **Step 4: Verify**

```bash
wc -l BSP/gimbal_driver.c BSP/gimbal_driver.h
```

Expected: ~450 lines for .c, ~73 lines for .h.

- [ ] **Step 5: Commit**

```bash
git add BSP/gimbal_driver.c BSP/gimbal_driver.h
git commit -m "feat: copy gimbal_driver from reference (BSP layer, C driver)"
```

---

### Task 3: Create App/mode_gimbal_test.c/h

**Files:**
- Create: `App/mode_gimbal_test.c`
- Create: `App/mode_gimbal_test.h`

**Interfaces:**
- Consumes: `HAL/app_mode.h`, `BSP/gimbal_driver.h`, `HAL/display.h`, `HAL/logger.h`
- Produces: `extern const AppMode mode_gimbal_test;`

- [ ] **Step 1: Write App/mode_gimbal_test.h**

Follow exact pattern from `App/mode_chassis_test.h`:
```c
#ifndef APP_MODE_GIMBAL_TEST_H
#define APP_MODE_GIMBAL_TEST_H
#include "HAL/app_mode.h"

extern const AppMode mode_gimbal_test;
#endif
```

- [ ] **Step 2: Write App/mode_gimbal_test.c**

Follow `mode_chassis_test.c` pattern. Dependencies:
- `extern Display *g_disp;`, `extern Logger *g_log;`
- From `Core/Inc/usart.h`: `extern UART_HandleTypeDef huart4;`
- Includes: `"mode_gimbal_test.h"`, `"BSP/gimbal_driver.h"`, `"HAL/display.h"`,
  `"HAL/logger.h"`, `<stdio.h>`

```c
#include "mode_gimbal_test.h"
#include "BSP/gimbal_driver.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include <stdio.h>

extern Display *g_disp;
extern Logger  *g_log;
extern UART_HandleTypeDef huart4;

static void gmt_enter(void)
{
    gimbal_init(&huart4);   /* blocking ~1500ms motor boot */
    gimbal_sync();           /* start position sync */
    g_log->info("Enter GIMBAL mode");
}

static void gmt_isr(void)
{
    /* Poll runs in on_ui (calls HAL_UART_Transmit, must be main-loop context) */
}

static void gmt_ui(void)
{
    gimbal_poll();  /* non-blocking, one step per call */

    /* OLED display */
    char buf[32];
    g_disp->clear();
    g_disp->show_str(0, 0, "GIMBAL");
    sprintf(buf, "X:%4.1f->%4.1f",
            gimbal_get_current(GIMBAL_AXIS_X) / 10.0f,
            gimbal_get_target(GIMBAL_AXIS_X) / 10.0f);
    g_disp->show_str(0, 18, buf);
    sprintf(buf, "Y:%4.1f->%4.1f",
            gimbal_get_current(GIMBAL_AXIS_Y) / 10.0f,
            gimbal_get_target(GIMBAL_AXIS_Y) / 10.0f);
    g_disp->show_str(0, 36, buf);
    sprintf(buf, "spd:%d %s",
            (int)gimbal_get_speed(),
            gimbal_is_init_done() ? "OK" : "");
    g_disp->show_str(0, 52, buf);
    g_disp->flush();
}

static void gmt_cmd(Command cmd, char data)
{
    (void)data;
    /* CMD_NEXT handled by main.c (mode switch).
       CMD_TOGGLE: sync re-trigger. */
    if (cmd == CMD_TOGGLE) {
        gimbal_sync();
        g_log->info("GIMBAL re-sync");
    }
}

const AppMode mode_gimbal_test = {
    .name       = "GIMBAL",
    .on_enter   = gmt_enter,
    .on_isr     = gmt_isr,
    .on_ui      = gmt_ui,
    .on_command = gmt_cmd,
};
```

- [ ] **Step 3: Commit**

```bash
git add App/mode_gimbal_test.c App/mode_gimbal_test.h
git commit -m "feat: add GIMBAL AppMode — BSP driver direct integration"
```

---

### Task 4: Modify App/main.c — wire gimbal into scheduler

**Files:**
- Modify: `App/main.c`

**Interfaces:**
- Consumes: `BSP/gimbal_driver.h`, `App/mode_gimbal_test.h` (Task 2, 3)
- Existing: `Core/Inc/usart.h` (huart4), `Core/Inc/dma.h` (MX_DMA_Init)

**4 changes:**
1. Add includes
2. Add `MX_DMA_Init()` + `MX_UART4_Init()` calls
3. Register mode in `g_modes[]`
4. (Nothing else — DMA callback is in gimbal_driver.c)

- [ ] **Step 1: Add includes**

After the existing adapter includes block (~line 35), add:
```c
#include "BSP/gimbal_driver.h"
```

After the existing mode includes (~line 38), add:
```c
#include "mode_gimbal_test.h"
```

Also add near the top (after `#include "gpio.h"`):
```c
#include "dma.h"
```

- [ ] **Step 2: Add MX_DMA_Init() and MX_UART4_Init()**

After `MX_UART8_Init();` (~line 138), add:
```c
  MX_DMA_Init();
  MX_UART4_Init();
```

Peripheral init order: DMA before UART4 (DMA must be clocked before UART4 MspInit configures the DMA stream).

- [ ] **Step 3: Register mode**

Change:
```c
static const AppMode *g_modes[] = { &mode_lap_run };
#define MODE_COUNT 1
```
To:
```c
static const AppMode *g_modes[] = { &mode_lap_run, &mode_gimbal_test };
#define MODE_COUNT 2
```

- [ ] **Step 4: Verify main.c**

Read back and check:
- `#include "dma.h"` present
- `#include "BSP/gimbal_driver.h"` present
- `#include "mode_gimbal_test.h"` present
- `MX_DMA_Init();` before `MX_UART4_Init();`
- `g_modes[]` has both modes, `MODE_COUNT` = 2
- No `gimbal_init()` or `gimbal_poll()` in main loop (these are in mode callbacks)

- [ ] **Step 5: Commit**

```bash
git add App/main.c
git commit -m "feat: wire gimbal v2 into main.c (DMA+UART4 init, mode register)"
```

---

## Verification Checklist

After all tasks:

1. **Compilation**: Project compiles in Keil. New files in BSP/ need include paths:
   `..\BSP` added to Keil C/C++ include paths.
2. **Link**: `Core/Src/dma.c` must be in Keil project Source Group.
3. **Runtime**: Power on → default mode is LAP. Press KEY3 → GIMBAL mode.
   `gimbal_init` blocks 1.5s. OLED shows X/Y current/target/speed.
   Serial log shows UART4 TX frames. If gimbal connected + powered,
   UART4 RX feedback should arrive.
