#include "mode_speed.h"
#include "HAL/chassis.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include <stdio.h>

extern Chassis *g_chassis;
extern Display *g_disp;
extern Logger  *g_log;

static int16_t g_speed = 20;     /* 目标速度, KEY1/2调 */
static uint8_t g_run  = 0;       /* 运行/停止 */

static void spd_enter(void) {
    g_run = 0;
    g_log->info("Enter SPEED mode");
}

static void spd_isr(void) {
    if (g_run) {
        g_chassis->set_speeds(g_speed, g_speed);
    }
}

static void spd_ui(void) {
    char buf[16];
    g_disp->clear();
    g_disp->show_str(0, 0, "SPEED");
    g_disp->show_str(50, 0, g_run ? "RUN" : "STOP");

    sprintf(buf, "Kp:%.0f Ki:%.1f", (double)g_chassis->pid_left->Kp,
            (double)g_chassis->pid_left->Ki);
    g_disp->show_str(0, 14, buf);

    sprintf(buf, "T:%d L:%d", g_speed, g_chassis->actual_l);
    g_disp->show_str(0, 26, buf);
    sprintf(buf, "   R:%d", g_chassis->actual_r);
    g_disp->show_str(0, 38, buf);

    g_disp->flush();

    g_log->data("Spd:%d AL:%d AR:%d", g_speed,
                g_chassis->actual_l, g_chassis->actual_r);
}

static void spd_cmd(Command cmd) {
    switch (cmd) {
    case CMD_TOGGLE:
        g_run = !g_run;
        if (!g_run) { g_chassis->set_speeds(0, 0); g_chassis->brake(); }
        break;
    case CMD_UP:
        if (g_speed < 900) g_speed += 5;
        if (g_run) g_chassis->set_speeds(g_speed, g_speed);
        break;
    case CMD_DOWN:
        if (g_speed > -900) g_speed -= 5;
        if (g_run) g_chassis->set_speeds(g_speed, g_speed);
        break;
    default: break;
    }
}

const AppMode mode_speed = {
    .name       = "SPEED",
    .on_enter   = spd_enter,
    .on_isr     = spd_isr,
    .on_ui      = spd_ui,
    .on_command = spd_cmd,
};
