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
