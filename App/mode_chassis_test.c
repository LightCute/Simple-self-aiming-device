#include "mode_chassis_test.h"
#include "HAL/chassis.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include "cmd_serial.h"
#include <stdio.h>
#include <string.h>

extern Chassis *g_chassis;
extern Display *g_disp;
extern Logger  *g_log;

static uint8_t  g_enc_show = 1;    /* 编码器显示 */
static uint8_t  g_spd_run  = 0;    /* 电机运行 */
static int16_t  g_tgt_l    = 20;   /* 左目标速度 */
static int16_t  g_tgt_r    = 20;   /* 右目标速度 */

static void cht_enter(void) {
    g_enc_show = 1;
    g_spd_run  = 0;
    g_log->info("CHASSIS TEST ready");
}

static void cht_isr(void) {
    if (g_spd_run) {
        g_chassis->set_speeds(g_tgt_l, g_tgt_r);
    }
}

static void cht_ui(void) {
    char buf[32];
    g_disp->clear();
    g_disp->show_str(0, 0, "CHASSIS");
    g_disp->show_str(55, 0, g_spd_run ? "RUN" : "STOP");

    if (g_enc_show) {
        int32_t el, er;
        g_chassis->get_encoders(&el, &er);
        sprintf(buf, "Enc L:%d R:%d", (int)el, (int)er);
        g_disp->show_str(0, 14, buf);
    }

    sprintf(buf, "Spd L:%d R:%d", g_chassis->actual_l, g_chassis->actual_r);
    g_disp->show_str(0, 26, buf);

    sprintf(buf, "Tgt L:%d R:%d", g_tgt_l, g_tgt_r);
    g_disp->show_str(0, 38, buf);

    sprintf(buf, "Kp:%.0f Ki:%.1f", (double)g_chassis->pid_left->Kp,
            (double)g_chassis->pid_left->Ki);
    g_disp->show_str(0, 50, buf);

    g_disp->flush();

    /* g_log->data("Enc... Spd..."); -- 刷屏, 暂时关闭 */
}

static void cht_cmd(Command cmd, char data) {
    (void)data;

    if (cmd == CMD_TOGGLE) {    /* KEY4 */
        g_spd_run = !g_spd_run;
        if (!g_spd_run) g_chassis->stop();
        return;
    }

    if (cmd == CMD_CUSTOM) {
        const char *s = CmdSerial_GetString();
        int v1, v2;
        float fv;

        if (strcmp(s, "encoder:on") == 0) {
            g_enc_show = 1; g_log->info("Encoder ON");
        }
        else if (strcmp(s, "encoder:off") == 0) {
            g_enc_show = 0; g_log->info("Encoder OFF");
        }
        else if (strcmp(s, "stop:on") == 0) {
            g_chassis->stop(); g_spd_run = 0; g_log->info("Stop");
        }
        else if (strcmp(s, "brake:on") == 0) {
            g_chassis->brake(); g_spd_run = 0; g_log->info("Brake");
        }
        else if (sscanf(s, "set_speed:%d,%d", &v1, &v2) == 2) {
            g_tgt_l = (int16_t)v1;
            g_tgt_r = (int16_t)v2;
            g_chassis->set_speeds(g_tgt_l, g_tgt_r);
            g_spd_run = 1;
            g_log->info("set_speed:%d,%d", v1, v2);
        }
        else if (sscanf(s, "kp:%f", &fv) == 1) {
            g_chassis->pid_left->Kp  = fv;
            g_chassis->pid_right->Kp = fv;
            g_log->info("Kp=%.1f", (double)fv);
        }
        else if (sscanf(s, "ki:%f", &fv) == 1) {
            g_chassis->pid_left->Ki  = fv;
            g_chassis->pid_right->Ki = fv;
            g_log->info("Ki=%.1f", (double)fv);
        }
    }
}

const AppMode mode_chassis_test = {
    .name       = "CHASSIS",
    .on_enter   = cht_enter,
    .on_isr     = cht_isr,
    .on_ui      = cht_ui,
    .on_command = cht_cmd,
};
