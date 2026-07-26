#include "speed_ctrl.h"
#include <string.h>

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

void SpeedCtrl_Init(SpeedCtrl *s)
{
    memset(s, 0, sizeof(*s));
    s->pid_l.Kp = 85.0f; s->pid_l.Ki = 1.7f; s->pid_l.Kd = 0.0f;
    s->pid_r.Kp = 85.0f; s->pid_r.Ki = 1.7f; s->pid_r.Kd = 0.0f;
    s->pid_l.integral_max = 500; s->pid_l.output_max = 999;
    s->pid_r.integral_max = 500; s->pid_r.output_max = 999;
}

void SpeedCtrl_SetTargets(SpeedCtrl *s, int16_t l, int16_t r) { s->target_l=l; s->target_r=r; }
void SpeedCtrl_SetActual(SpeedCtrl *s, int16_t l, int16_t r) { s->actual_l=l; s->actual_r=r; }

void SpeedCtrl_ClearIntegral(SpeedCtrl *s)
{
    s->integral_l = 0; s->prev_err_l = 0;
    s->integral_r = 0; s->prev_err_r = 0;
}

void SpeedCtrl_Update(SpeedCtrl *s)
{
    float el = (float)(s->target_l - s->actual_l);
    float er = (float)(s->target_r - s->actual_r);
    s->pwm_l = (int16_t)pid_calc(&s->pid_l, &s->integral_l, &s->prev_err_l, el);
    s->pwm_r = (int16_t)pid_calc(&s->pid_r, &s->integral_r, &s->prev_err_r, er);
}
