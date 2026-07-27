#include "mode_lap_run.h"
#include "HAL/tracker.h"
#include "HAL/turn.h"
#include "HAL/chassis.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include <stdio.h>

extern Tracker  *g_tracker;
extern TurnCtrl *g_turn;
extern Chassis  *g_chassis;
extern Display  *g_disp;
extern Logger   *g_log;

typedef enum { S_IDLE, S_LINE, S_CORNER_STRAIGHT, S_CORNER_TURN, S_DONE } LapState;
static LapState g_st = S_IDLE;

static uint8_t g_lap_target = 3;
static uint8_t g_lap_count  = 0;
static uint8_t g_corner_cnt = 0;

static uint8_t g_corner_dir = 0;
static int16_t g_straight_dist = 300;
static uint8_t g_grace = 0;

static const char *state_name(void) {
    switch(g_st) {
    case S_IDLE: return "IDLE"; case S_LINE: return "LINE";
    case S_CORNER_STRAIGHT: return "STR"; case S_CORNER_TURN: return "TURN";
    case S_DONE: return "DONE"; default: return "?";
    }
}

static void lap_enter(void) {
    g_st = S_IDLE;
    g_tracker->init(g_tracker);
    g_turn->init(g_turn);
    g_log->info("Enter LAP mode");
}

static void lap_isr(void) {
    switch (g_st) {

    case S_IDLE:
        break;

    case S_LINE: {
        if (g_grace > 0) g_grace--;
        TrackEvent ev = (g_grace > 0) ? TRACK_OK : g_tracker->update(g_tracker);

        if (ev == TRACK_OK) {
            int16_t base = g_tracker->base_speed;
            int16_t corr = (int16_t)g_tracker->correction;
            g_chassis->set_speeds(base - corr, base + corr);
        } else {
            /* 检测到弯道 */
            g_chassis->stop();
            g_corner_dir = (ev == TRACK_LEFT) ? 1 : 2;
            int16_t spd = (int16_t)20;
            g_chassis->set_speeds(spd, spd);
            g_st = S_CORNER_STRAIGHT;
            g_log->info("Corner %s, lap=%d/%d corner=%d/4",
                        (g_corner_dir==1)?"L":"R",
                        g_lap_count+1, g_lap_target, g_corner_cnt+1);
        }
        break;
    }

    case S_CORNER_STRAIGHT: {
        int16_t spd = (int16_t)20;
        g_chassis->set_speeds(spd, spd);
        /* 直行g_straight_dist脉冲后转弯 (延时估算) */
        static uint8_t cnt = 0;
        if (++cnt >= (g_straight_dist / 10)) {   /* 约等于编码距离/10帧 */
            cnt = 0;
            g_chassis->stop();
            float angle = (g_corner_dir == 1) ? 90.0f : -90.0f;
            g_turn->spot(g_turn, angle);
            g_st = S_CORNER_TURN;
            g_log->info("Turn %.0f", (double)angle);
        }
        break;
    }

    case S_CORNER_TURN:
        g_turn->update(g_turn);
        if (g_turn->state == TURN_DONE) {
            g_corner_cnt++;
            g_log->info("Corner done: %d/4 lap=%d/%d",
                        g_corner_cnt, g_lap_count+1, g_lap_target);

            uint8_t is_last = 0;
            if (g_corner_cnt >= 4) {
                g_corner_cnt = 0;
                g_lap_count++;
                if (g_lap_count >= g_lap_target) {
                    is_last = 1;
                }
            }

            if (is_last) {
                g_chassis->stop();
                g_st = S_DONE;
                g_log->info("ALL LAPS DONE!");
            } else {
                g_grace = 30;
                g_tracker->init(g_tracker);
                g_st = S_LINE;
                g_log->info("Resume LINE, grace=%d", g_grace);
            }
        }
        break;

    case S_DONE:
        break;
    }
}

static void lap_ui(void) {
    char buf[32];
    g_disp->clear();
    sprintf(buf, "LAP %s", state_name());
    g_disp->show_str(0, 0, buf);
    sprintf(buf, "L:%d/%d C:%d/4", g_lap_count+1, g_lap_target, g_corner_cnt);
    g_disp->show_str(0, 20, buf);
    sprintf(buf, "Spd L:%d R:%d", g_chassis->actual_l, g_chassis->actual_r);
    g_disp->show_str(0, 38, buf);
    g_disp->flush();
}

static void lap_cmd(Command cmd, char data) {
    (void)data;
    switch (cmd) {
    case CMD_TOGGLE:
        if (g_st == S_IDLE || g_st == S_DONE) {
            g_lap_count = 0; g_corner_cnt = 0; g_grace = 0;
            g_tracker->init(g_tracker);
            g_st = S_LINE;
            g_log->info("LAP start, target=%d laps", g_lap_target);
        }
        break;
    case CMD_UP:   if (g_st==S_IDLE && g_lap_target<5) g_lap_target++; break;
    case CMD_DOWN: if (g_st==S_IDLE && g_lap_target>1) g_lap_target--; break;
    default: break;
    }
}

const AppMode mode_lap_run = {
    .name = "LAP", .on_enter = lap_enter,
    .on_isr = lap_isr, .on_ui = lap_ui, .on_command = lap_cmd,
};
