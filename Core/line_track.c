#include "line_track.h"
#include "tracking.h"
#include <stdio.h>

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

void LineTrack_Init(LineTrack *t)
{
    t->steer_pid.Kp = 1.12f; t->steer_pid.Ki = 0.0f; t->steer_pid.Kd = 0.96f;
    t->steer_pid.integral = 0; t->steer_pid.prev_error = 0;
    t->steer_pid.integral_max = 200; t->steer_pid.output_max = 999;
    t->base_speed = 20;
    t->correction = 0;
}

TrackEvent LineTrack_Update(LineTrack *t)
{
    Tracking_UpdateOffset();
    t->steer_pid.Kp = 1.12f; t->steer_pid.Ki = 0.0f; t->steer_pid.Kd = 0.96f;
    t->steer_pid.output_max = 999;
    t->correction = PID_Calc(&t->steer_pid, (float)g_tracking_offset);

    uint8_t sharp = Tracking_IsSharpTurn();
    if (sharp == 1)
    {
        return TRACK_LEFT;
    }
    if (sharp == 2)
    {
        return TRACK_RIGHT;
    }
    return TRACK_OK;
}
