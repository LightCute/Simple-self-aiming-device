#include "speed_ctrl.h"
#include "tb6612.h"

#define PWM_MAX 999

static float PID_Calc(PID_t *pid, float error)
{
    float p = pid->Kp * error;
    pid->integral += error;
    if (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i = pid->Ki * pid->integral;
    float d = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;
    float out = p + i + d;
    if (out >  pid->output_max) out =  pid->output_max;
    if (out < -pid->output_max) out = -pid->output_max;
    return out;
}

void SpeedCtrl_Init(SpeedCtrl *s)
{
    s->pid_l.Kp = 85.0f; s->pid_l.Ki = 1.7f; s->pid_l.Kd = 0.0f;
    s->pid_r.Kp = 85.0f; s->pid_r.Ki = 1.7f; s->pid_r.Kd = 0.0f;
    s->pid_l.integral = 0; s->pid_l.prev_error = 0;
    s->pid_r.integral = 0; s->pid_r.prev_error = 0;
    s->pid_l.integral_max = 500; s->pid_l.output_max = PWM_MAX;
    s->pid_r.integral_max = 500; s->pid_r.output_max = PWM_MAX;
    s->target_l = s->target_r = 0;
    s->actual_l = s->actual_r = 0;
    s->pwm_l = s->pwm_r = 0;
}

void SpeedCtrl_SetTargets(SpeedCtrl *s, int16_t left, int16_t right)
{
    s->target_l = left;
    s->target_r = right;
}

void SpeedCtrl_Update(SpeedCtrl *s)
{
    s->pid_l.Kp = 85.0f; s->pid_l.Ki = 1.7f; s->pid_l.Kd = 0.0f;
    s->pid_r.Kp = 85.0f; s->pid_r.Ki = 1.7f; s->pid_r.Kd = 0.0f;

    s->actual_l = TB6612_GetLeftSpeed();
    s->actual_r = TB6612_GetRightSpeed();

    s->pwm_l = (int16_t)PID_Calc(&s->pid_l, (float)(s->target_l - s->actual_l));
    s->pwm_r = (int16_t)PID_Calc(&s->pid_r, (float)(s->target_r - s->actual_r));

    TB6612_Run(s->pwm_l, s->pwm_r);
}
