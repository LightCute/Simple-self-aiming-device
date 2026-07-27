#include "mode_distance.h"
#include "HAL/chassis.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include <stdio.h>

extern Chassis *g_chassis;
extern Display *g_disp;
extern Logger  *g_log;

static int32_t g_target   = 500;   /* 目标编码脉冲, +/-调方向 */
static int32_t g_start_enc = 0;    /* 起点编码值 */
static int32_t g_cur_enc   = 0;    /* 当前编码值 */
static uint8_t g_run       = 0;

static void dist_enter(void) {
    g_run = 0;
    g_log->info("Enter DIST mode");
}

static void dist_isr(void) {
    if (!g_run) return;

    int32_t el, er;
    g_chassis->get_encoders(&el, &er);
    g_cur_enc = (el + er) / 2;

    int32_t traveled = g_cur_enc - g_start_enc;
    if (traveled < 0) traveled = -traveled;

    int32_t target_abs = (g_target > 0) ? g_target : -g_target;

    if (traveled >= target_abs) {
        g_chassis->stop();
        g_run = 0;
        g_log->info("DIST done: %d", (int)traveled);
    } else {
        int16_t spd = (g_target > 0) ? (int16_t)20 : (int16_t)-20;
        g_chassis->set_speeds(spd, spd);
    }
}

static void dist_ui(void) {
    char buf[16];
    g_disp->clear();
    g_disp->show_str(0, 0, "DIST");
    g_disp->show_str(50, 0, g_run ? "RUN" : "STOP");

    sprintf(buf, "Tgt:%d", (int)g_target);
    g_disp->show_str(0, 14, buf);

    int32_t traveled = g_cur_enc - g_start_enc;
    if (traveled < 0) traveled = -traveled;
    sprintf(buf, "Cur:%d/%d", (int)traveled,
            (int)((g_target > 0) ? g_target : -g_target));
    g_disp->show_str(0, 26, buf);

    sprintf(buf, "Spd L:%d R:%d", g_chassis->actual_l, g_chassis->actual_r);
    g_disp->show_str(0, 38, buf);

    g_disp->flush();
}

static void dist_cmd(Command cmd, char data) {
    (void)data;
    switch (cmd) {
    case CMD_TOGGLE:
        g_run = !g_run;
        if (g_run) {
            int32_t el, er;
            g_chassis->get_encoders(&el, &er);
            g_start_enc = (el + er) / 2;   /* 快照起点 */
        } else {
            g_chassis->stop();
        }
        break;
    case CMD_UP:
        if (g_target < 10000) g_target += 100;
        break;
    case CMD_DOWN:
        if (g_target > -10000) g_target -= 100;
        break;
    default: break;
    }
}

const AppMode mode_distance = {
    .name       = "DIST",
    .on_enter   = dist_enter,
    .on_isr     = dist_isr,
    .on_ui      = dist_ui,
    .on_command = dist_cmd,
};
