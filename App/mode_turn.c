#include "mode_turn.h"
#include "HAL/turn.h"
#include "HAL/imu.h"
#include "HAL/chassis.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include "Adapters/cmd_serial.h"
#include "Middleware/angle_pid.h"
#include <stdio.h>
#include <string.h>

extern TurnCtrl *g_turn;
extern IMU      *g_imu;
extern Chassis  *g_chassis;
extern Display  *g_disp;
extern Logger   *g_log;

static float   g_angle     = 90.0f;
static int16_t g_base_spd  = 10;     /* 弧线基速 */

static void trn_enter(void) {
    g_turn->init(g_turn);
    g_log->info("Enter TURN mode");
}

static TurnState g_prev_st = TURN_IDLE;

static void trn_isr(void) {
    g_turn->update(g_turn);
    if (g_turn->state != g_prev_st) {
        const char *name = (g_turn->state==TURN_IDLE)?"IDLE":
                           (g_turn->state==TURN_RUNNING)?"RUN":"DONE";
        g_log->info("TURN state: %s", name);
        g_prev_st = g_turn->state;
    }
}

static void trn_ui(void) {
    char buf[16];
    g_disp->clear();
    g_disp->show_str(0, 0, "TURN");

    const char *st = "IDLE";
    if (g_turn->state == TURN_RUNNING) st = "RUN";
    if (g_turn->state == TURN_DONE)    st = "DONE";
    g_disp->show_str(50, 0, st);

    sprintf(buf, "Set:%.0f Base:%d", (double)g_angle, (int)g_base_spd);
    g_disp->show_str(0, 14, buf);

    sprintf(buf, "Z:%.1f", (double)g_imu->yaw);
    g_disp->show_str(0, 26, buf);

    sprintf(buf, "Err:%.1f Tol:%.0f", (double)g_turn->angle_err,
            (double)g_angle_turn_tolerance);
    g_disp->show_str(0, 38, buf);

    sprintf(buf, "Spd L:%d R:%d", g_chassis->actual_l, g_chassis->actual_r);
    g_disp->show_str(0, 50, buf);

    g_disp->flush();
}

static void trn_cmd(Command cmd, char data) {
    if (cmd == CMD_TOGGLE) {
        /* KEY4: 原地转, 方向交替 */
        static int8_t dir = 1;
        g_turn->spot(g_turn, g_angle * dir);
        dir = -dir;
        g_log->info("Turn spot %.0f", (double)(g_angle * (dir < 0 ? 1 : -1)));
        return;
    }

    if (cmd == CMD_CUSTOM) {
        const char *s = CmdSerial_GetString();
        if (strcmp(s, "arc") == 0) {
            static int8_t dir = 1;
            g_turn->arc(g_turn, g_angle * dir, g_base_spd);
            dir = -dir;
            g_log->info("Turn arc %.0f spd=%d",
                        (double)(g_angle * (dir < 0 ? 1 : -1)), (int)g_base_spd);
        }
    }
}

const AppMode mode_turn = {
    .name       = "TURN",
    .on_enter   = trn_enter,
    .on_isr     = trn_isr,
    .on_ui      = trn_ui,
    .on_command = trn_cmd,
};
