#include "mode_chassis_test.h"
#include "HAL/chassis.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include <stdio.h>

extern Chassis *g_chassis;
extern Display *g_disp;
extern Logger  *g_log;

static uint8_t g_enc_run  = 0;   /* 编码测量模式 */
static uint8_t g_spd_run  = 0;   /* set_speeds 运行 */
static int16_t g_test_spd = 20;  /* 固定测试速度 */

static void cht_enter(void) {
    g_enc_run = 0;
    g_spd_run = 0;
    g_log->info("Enter CHASSIS TEST");
}

static void cht_isr(void) {
    if (g_spd_run) {
        g_chassis->set_speeds(g_test_spd, g_test_spd);
    }
}

static void cht_ui(void) {
    char buf[16];
    int32_t el, er;
    g_chassis->get_encoders(&el, &er);

    g_disp->clear();
    g_disp->show_str(0, 0, "CHASSIS");
    g_disp->show_str(55, 0, g_spd_run ? "SPD" : "---");

    sprintf(buf, "Enc L:%d", (int)el);
    g_disp->show_str(0, 14, buf);
    sprintf(buf, "Enc R:%d", (int)er);
    g_disp->show_str(0, 26, buf);

    sprintf(buf, "SpdL:%d SpdR:%d", g_chassis->actual_l, g_chassis->actual_r);
    g_disp->show_str(0, 38, buf);

    sprintf(buf, "Kp:%.0f", (double)g_chassis->pid_left->Kp);
    g_disp->show_str(0, 50, buf);

    g_disp->flush();

    g_log->data("Enc L:%d R:%d SpdL:%d SpdR:%d",
                (int)el, (int)er,
                g_chassis->actual_l, g_chassis->actual_r);
}

static void cht_cmd(Command cmd, char data) {
    switch (cmd) {
    case CMD_TOGGLE:   /* KEY4: 设置速度的启停 */
        g_spd_run = !g_spd_run;
        if (!g_spd_run) g_chassis->stop();
        break;
    case CMD_CUSTOM:
        if (data == '1') {   /* 串口1: 编码测量启停 */
            g_enc_run = !g_enc_run;
            g_log->info(g_enc_run ? "Encoder ON" : "Encoder OFF");
        }
        else if (data == '2') {  /* 串口2: set_speeds 启停 */
            g_spd_run = !g_spd_run;
            if (!g_spd_run) g_chassis->stop();
            g_log->info(g_spd_run ? "Motor ON" : "Motor OFF");
        }
        break;
    default: break;
    }
}

const AppMode mode_chassis_test = {
    .name       = "CHASSIS",
    .on_enter   = cht_enter,
    .on_isr     = cht_isr,
    .on_ui      = cht_ui,
    .on_command = cht_cmd,
};
