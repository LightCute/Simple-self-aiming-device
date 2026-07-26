#include "chassis_tb6612.h"
#include "tb6612.h"
#include <string.h>

#define PWM_MAX 999

/* PID 状态 */
static float g_iL=0, g_pL=0;  /* left integral, prev_error */
static float g_iR=0, g_pR=0;  /* right integral, prev_error */

static int16_t g_target_l=0, g_target_r=0;

/* PID 参数默认值 */
static PID_Params g_pid_l = { 85.0f, 1.7f, 0.0f, 500.0f, (float)PWM_MAX };
static PID_Params g_pid_r = { 85.0f, 1.7f, 0.0f, 500.0f, (float)PWM_MAX };

/* ======== PID 计算 ======== */
static float pid_calc(PID_Params *p, float *integral, float *prev, float error)
{
    *integral += error;
    if (*integral >  p->integral_max) *integral =  p->integral_max;
    if (*integral < -p->integral_max) *integral = -p->integral_max;

    float out = p->Kp * error + p->Ki * (*integral) + p->Kd * (error - *prev);
    *prev = error;

    if (out >  p->output_max) out =  p->output_max;
    if (out < -p->output_max) out = -p->output_max;
    return out;
}

/* ======== 接口实现 ======== */

static void tb_set_speeds(int16_t left, int16_t right)
{
    g_target_l = left;
    g_target_r = right;
}

static void tb_stop(void)  { TB6612_Stop();  g_target_l = g_target_r = 0; }
static void tb_brake(void) { TB6612_Brake(); g_target_l = g_target_r = 0; }

static void tb_get_encoders(int32_t *left, int32_t *right)
{
    int32_t el, er;
    TB6612_GetEncoder(&el, &er);
    /* 映射: 前进0→65535, 后退65535→0 */
    // *left  = (el < 0) ? (int32_t)(65535 + el) : el;
    // *right = (er < 0) ? (int32_t)(65535 + er) : er;
    *left  = el;
    *right = er;
    if(el < -23768) { //可认定为小车是前进的
        *left = el + 65535;
    }
    if(er < -23768) { //可认定为小车是前进的
        *right = er  + 65535;
    }
}

static void tb_init(void) { TB6612_Init(); TB6612_ResetEncoder(); }

/* 每10ms由ISR调用: 速度闭环 */
static void tb_update(void)
{
    int16_t al = TB6612_GetLeftSpeed();
    int16_t ar = TB6612_GetRightSpeed();
    g_chassis_tb6612.actual_l = al;
    g_chassis_tb6612.actual_r = ar;

    float el = (float)(g_target_l - al);
    float er = (float)(g_target_r - ar);

    int16_t pwm_l = (int16_t)pid_calc(&g_pid_l, &g_iL, &g_pL, el);
    int16_t pwm_r = (int16_t)pid_calc(&g_pid_r, &g_iR, &g_pR, er);

    TB6612_Run(pwm_l, pwm_r);
}

Chassis g_chassis_tb6612 = {
    .init         = tb_init,
    .update       = tb_update,
    .set_speeds   = tb_set_speeds,
    .stop         = tb_stop,
    .brake        = tb_brake,
    .get_encoders = tb_get_encoders,
    .actual_l = 0, .actual_r = 0,
    .pid_left  = &g_pid_l,
    .pid_right = &g_pid_r,
};
