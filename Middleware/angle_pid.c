#include "angle_pid.h"

void AnglePID_Init(AnglePID *p)
{
    p->Kp = 1.0f; p->Ki = 0.05f; p->Kd = 5.0f;
    p->integral = 0; p->prev_error = 0;
    p->integral_max = 200; p->output_max = 500;
    p->correction = 0; p->angle_err = 0; p->done = 0;
}

float AnglePID_Update(AnglePID *p, float err)
{
    p->angle_err = err;
    float abs_err = (err > 0) ? err : -err;

    /* 两段式: 粗调高速, 精调减速 */
    if (abs_err > 15.0f) p->output_max = 500;
    else                  p->output_max = 250;

    p->integral += err;
    if (p->integral >  p->integral_max) p->integral =  p->integral_max;
    if (p->integral < -p->integral_max) p->integral = -p->integral_max;

    float out = p->Kp * err + p->Ki * p->integral + p->Kd * (err - p->prev_error);
    p->prev_error = err;

    if (out >  p->output_max) out =  p->output_max;
    if (out < -p->output_max) out = -p->output_max;

    p->correction = out;
    p->done = (abs_err < 12.0f);  /* 误差<12° = 到位 */
    return out;
}
