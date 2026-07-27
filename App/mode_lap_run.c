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

/* 状态 */
typedef enum { S_IDLE, S_LINE, S_CORNER_STRAIGHT, S_CORNER_TURN, S_DONE } LapState;
static LapState g_st = S_IDLE;

/* 圈数 */
static uint8_t g_lap_target = 3;   /* 1~5 */
static uint8_t g_lap_count  = 0;
static uint8_t g_corner_cnt = 0;   /* 0~3 */

/* 编码测距 */
static int32_t g_enc_start  = 0;
static int32_t g_enc_accum  = 0;
static int32_t g_edge_len   = 0;    /* 正方形边长 */
static int32_t g_first_seg  = 0;    /* 起点→第一弯 */

/* 弯道 */
static uint8_t g_corner_dir = 0;    /* 1=左, 2=右 */
static int16_t g_straight_dist = 200; /* 弯前直行补偿 */

/* ======== 辅助 ======== */
static int32_t enc_dist(void) {
    int32_t el, er;
    g_chassis->get_encoders(&el, &er);
    int32_t cur = (el + er) / 2;
    return cur - g_enc_start;
}

static void enc_mark_start(void) {
    int32_t el, er;
    g_chassis->get_encoders(&el, &er);
    g_enc_start = (el + er) / 2;
}

static const char *state_name(void) {
    switch(g_st) {
    case S_IDLE:            return "IDLE";
    case S_LINE:            return "LINE";
    case S_CORNER_STRAIGHT: return "STR";
    case S_CORNER_TURN:     return "TURN";
    case S_DONE:            return "DONE";
    default: return "?";
    }
}

/* ======== AppMode ======== */
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
        TrackEvent ev = g_tracker->update(g_tracker);
        g_enc_accum = enc_dist();

        if (ev == TRACK_OK) {
            int16_t base = g_tracker->base_speed;
            int16_t corr = (int16_t)g_tracker->correction;
            g_chassis->set_speeds(base - corr, base + corr);
        } else {
            /* 检测到弯道 */
            g_chassis->stop();
            g_corner_dir = (ev == TRACK_LEFT) ? 1 : 2;

            /* 记录段长 */
            if (g_corner_cnt == 0 && g_lap_count == 0) {
                g_first_seg = g_enc_accum;   /* 起点到第一弯 */
            } else if (g_enc_accum > 0) {
                g_edge_len = g_enc_accum;    /* 边长 */
            }

            /* 进入弯前直行 */
            enc_mark_start();
            int16_t spd = (g_tracker->base_speed > 0) ? g_tracker->base_speed
                                                       : (int16_t)20;
            g_chassis->set_speeds(spd, spd);
            g_st = S_CORNER_STRAIGHT;
            g_log->info("Corner %s, go straight %d",
                        (g_corner_dir==1)?"L":"R", (int)g_straight_dist);
        }
        break;
    }

    case S_CORNER_STRAIGHT: {
        int32_t d = enc_dist();
        if (d < 0) d = -d;
        if (d >= g_straight_dist) {
            g_chassis->stop();
            /* 转弯 */
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

            if (g_corner_cnt >= 4) {
                g_corner_cnt = 0;
                g_lap_count++;
                if (g_lap_count >= g_lap_target) {
                    /* 最后一弯: 直行回起点 */
                    int32_t back = g_edge_len - g_first_seg;
                    if (back < 0) back = -back;
                    enc_mark_start();
                    int16_t spd = (int16_t)20;
                    g_chassis->set_speeds(spd, spd);
                    g_st = S_IDLE;  /* 简单停车 */
                    g_chassis->stop();
                    g_st = S_DONE;
                    g_log->info("ALL DONE! back=%d", (int)back);
                    break;
                }
            }
            /* 继续巡线 */
            g_tracker->init(g_tracker);
            enc_mark_start();
            g_st = S_LINE;
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
    g_disp->show_str(0, 14, buf);

    sprintf(buf, "Acc:%d", (int)g_enc_accum);
    g_disp->show_str(0, 26, buf);

    if (g_edge_len) {
        sprintf(buf, "Edge:%d 1st:%d", (int)g_edge_len, (int)g_first_seg);
        g_disp->show_str(0, 38, buf);
    }

    sprintf(buf, "Spd L:%d R:%d", g_chassis->actual_l, g_chassis->actual_r);
    g_disp->show_str(0, 50, buf);

    g_disp->flush();
}

static void lap_cmd(Command cmd, char data) {
    (void)data;
    switch (cmd) {
    case CMD_TOGGLE:  /* KEY4: 启动 */
        if (g_st == S_IDLE || g_st == S_DONE) {
            g_lap_count  = 0;
            g_corner_cnt = 0;
            g_edge_len   = 0;
            g_first_seg  = 0;
            g_tracker->init(g_tracker);
            enc_mark_start();
            g_st = S_LINE;
            g_log->info("LAP start, target=%d laps", g_lap_target);
        }
        break;
    case CMD_UP:   /* KEY2: 圈数+1 */
        if (g_st == S_IDLE && g_lap_target < 5) g_lap_target++;
        break;
    case CMD_DOWN: /* KEY1: 圈数-1 */
        if (g_st == S_IDLE && g_lap_target > 1) g_lap_target--;
        break;
    default: break;
    }
}

const AppMode mode_lap_run = {
    .name       = "LAP",
    .on_enter   = lap_enter,
    .on_isr     = lap_isr,
    .on_ui      = lap_ui,
    .on_command = lap_cmd,
};
