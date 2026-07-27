#include "line_pid.h"

void LinePID_Init(LinePID *p)
{
    p->Kp = 1.12f; p->Ki = 0.0f; p->Kd = 0.96f;
    p->integral  = 0; p->prev_error = 0;
    p->integral_max = 200; p->output_max = 999;
    p->correction = 0;
}

float LinePID_Update(LinePID *p, int32_t offset)
{
    float err = (float)offset;
    p->integral += err;
    if (p->integral >  p->integral_max) p->integral =  p->integral_max;
    if (p->integral < -p->integral_max) p->integral = -p->integral_max;

    float out = p->Kp * err + p->Ki * p->integral + p->Kd * (err - p->prev_error);
    p->prev_error = err;

    if (out >  p->output_max) out =  p->output_max;
    if (out < -p->output_max) out = -p->output_max;

    p->correction = out;
    return out;
}
