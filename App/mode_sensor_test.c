#include "mode_sensor_test.h"
#include "HAL/tracker.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include "tracking.h"
#include <stdio.h>

extern Tracker *g_tracker;
extern Display *g_disp;
extern Logger  *g_log;

static uint8_t g_active = 0;  /* 是否启用 (KEY4切换) */

static void sen_enter(void) {
    g_active = 1;
    g_log->info("Enter SENSOR mode");
}

static void sen_isr(void) {
    /* ISR只读取传感器, 不做PID */
    if (g_active) {
        Tracking_UpdateOffset();
    }
}

static void sen_ui(void) {
    char buf[32];
    uint8_t raw  = Tracking_GetRaw();
    uint8_t sharp = Tracking_IsSharpTurn();

    g_disp->clear();
    g_disp->show_str(0, 0, "SENSOR");

    /* 8路传感器位图: 1=黑 0=白 */
    for (int i = 0; i < 8; i++) {
        char bit[2];
        bit[0] = (raw & (1 << (7-i))) ? '1' : '0';
        bit[1] = '\0';
        g_disp->show_str(i * 14 + 10, 14, bit);
    }

    /* 偏移量 */
    sprintf(buf, "Off:%d", (int)g_tracker->offset);
    g_disp->show_str(0, 26, buf);

    /* 直角弯判定 */
    if (sharp == 1)      g_disp->show_str(20, 40, "<< LEFT");
    else if (sharp == 2) g_disp->show_str(20, 40, "RIGHT >>");
    else                 g_disp->show_str(20, 40, "        ");

    /* 计数 */
    sprintf(buf, "Raw:0x%02X", raw);
    g_disp->show_str(0, 50, buf);

    g_disp->flush();

    /* 仅在检测到路口时输出日志 */
    if (sharp != 0) {
        g_log->data("Raw=0x%02X Off=%d -> %s",
                    raw, (int)g_tracker->offset,
                    (sharp==1)?"LEFT":(sharp==2)?"RIGHT":"T");
    }
}

static void sen_cmd(Command cmd, char data) {
    (void)data;
    if (cmd == CMD_TOGGLE) {
        g_active = !g_active;
        g_log->info(g_active ? "Sensor ON" : "Sensor OFF");
    }
}

const AppMode mode_sensor_test = {
    .name       = "SENSOR",
    .on_enter   = sen_enter,
    .on_isr     = sen_isr,
    .on_ui      = sen_ui,
    .on_command = sen_cmd,
};
