#include "mode_track.h"
#include "HAL/tracker.h"
#include "HAL/chassis.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include <stdio.h>

extern Tracker *g_tracker;
extern Chassis *g_chassis;
extern Display *g_disp;
extern Logger  *g_log;

static uint8_t g_run = 0;

static void trk_enter(void) {
    g_run = 0;
    g_tracker->init(g_tracker);
    g_log->info("Enter TRACK mode");
}

static void trk_isr(void) {
    if (!g_run) return;

    TrackEvent ev = g_tracker->update(g_tracker);

    if (ev == TRACK_OK) {
        int16_t base = g_tracker->base_speed;
        int16_t corr = (int16_t)g_tracker->correction;
        g_chassis->set_speeds(base - corr, base + corr);
    } else {
        /* 遇到路口 → 停车 */
        g_chassis->stop();
        g_run = 0;
        if (ev == TRACK_LEFT)  g_log->info("LEFT corner, stop");
        if (ev == TRACK_RIGHT) g_log->info("RIGHT corner, stop");
        if (ev == TRACK_TJUNC) g_log->info("T-junction, stop");
    }
}

static void trk_ui(void) {
    char buf[16];
    g_disp->clear();
    g_disp->show_str(0, 0, "TRACK");
    g_disp->show_str(50, 0, g_run ? "RUN" : "STOP");

    sprintf(buf, "Off:%d", (int)g_tracking_offset);
    g_disp->show_str(0, 14, buf);

    sprintf(buf, "Corr:%.0f", (double)g_tracker->correction);
    g_disp->show_str(0, 26, buf);

    sprintf(buf, "Spd L:%d R:%d", g_chassis->actual_l, g_chassis->actual_r);
    g_disp->show_str(0, 38, buf);

    g_disp->flush();
}

static void trk_cmd(Command cmd, char data) {
    (void)data;
    if (cmd == CMD_TOGGLE) {
        g_run = !g_run;
        if (!g_run) g_chassis->stop();
    }
}

const AppMode mode_track = {
    .name       = "TRACK",
    .on_enter   = trk_enter,
    .on_isr     = trk_isr,
    .on_ui      = trk_ui,
    .on_command = trk_cmd,
};
