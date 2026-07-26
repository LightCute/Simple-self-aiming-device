#include "chassis.h"
#include "HAL/motor.h"
#include "Middleware/speed_ctrl.h"
#include "BSP/tb6612.h"   /* for g_motor_tb6612 */

static const Motor *g_motor = &g_motor_tb6612;
static SpeedCtrl    g_spd;

/* ==== 接口实现 ==== */

static void ch_init(void) {
    g_motor->init();
    SpeedCtrl_Init(&g_spd);
}

static void ch_update(void) {
    if (!g_spd.enabled) return;
    g_motor->update_speed();
    SpeedCtrl_SetActual(&g_spd, g_motor->speed_l, g_motor->speed_r);
    SpeedCtrl_Update(&g_spd);
    g_motor->set_pwm(g_spd.pwm_l, g_spd.pwm_r);
    g_chassis_inst.actual_l = g_spd.actual_l;
    g_chassis_inst.actual_r = g_spd.actual_r;
}

static void ch_set_speeds(int16_t l, int16_t r) {
    g_spd.target_l = l; g_spd.target_r = r;
    g_chassis_inst.target_l = l; g_chassis_inst.target_r = r;
    SpeedCtrl_ClearIntegral(&g_spd);
    g_spd.enabled = 1;
}

static void ch_stop(void) {
    g_spd.enabled = 0;
    SpeedCtrl_ClearIntegral(&g_spd);
    g_motor->stop();
}

static void ch_brake(void) {
    g_spd.enabled = 0;
    SpeedCtrl_ClearIntegral(&g_spd);
    g_motor->brake();
}

static void ch_get_encoders(int32_t *l, int32_t *r) {
    int32_t el, er;
    g_motor->get_encoder(&el, &er);
    *l = (el < 0) ? (int32_t)(65535 + el) : el;
    *r = (er < 0) ? (int32_t)(65535 + er) : er;
}

Chassis g_chassis_inst = {
    .init         = ch_init,
    .update       = ch_update,
    .set_speeds   = ch_set_speeds,
    .stop         = ch_stop,
    .brake        = ch_brake,
    .get_encoders = ch_get_encoders,
    .actual_l = 0, .actual_r = 0,
    .target_l = 0, .target_r = 0,
    .enabled  = 0,
    .pid_left  = &g_spd.pid_l,
    .pid_right = &g_spd.pid_r,
};
