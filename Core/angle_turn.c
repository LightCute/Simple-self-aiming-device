#include "angle_turn.h"
#include "mpu6050.h"
#include <math.h>

extern MPU6050_t mpu6050;
static float g_target_angle = 0.0f;
float g_angle_tolerance = 12.0f;  /* 转弯到位容限(度) */

static float Angle_Error(float target, float current)
{
    float err = target - current;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

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

void AngleTurn_Init(AngleTurn *a)
{
    a->pid.Kp = 0.01f; a->pid.Ki = 0.0001f; a->pid.Kd = 0.01f;
    a->pid.integral = 0; a->pid.prev_error = 0;
    a->pid.integral_max = 200; a->pid.output_max = 300;
    a->correction = 0.0f;
    a->state = ANGLE_IDLE;
}

void AngleTurn_Start(AngleTurn *a, float delta_deg)
{
    a->ref_angle = mpu6050.KalmanAngleZ;
    g_target_angle = delta_deg;
    a->pid.integral = 0;
    a->pid.prev_error = 0;
    a->pid.output_max = 500;
    a->correction = 0;
    a->state = ANGLE_RUNNING;
}

AngleState AngleTurn_Update(AngleTurn *a)
{
    if (a->state != ANGLE_RUNNING) return a->state;

    float abs_target = a->ref_angle + g_target_angle;
    float angle_err = Angle_Error(abs_target, mpu6050.KalmanAngleZ);
    float abs_err   = (angle_err > 0) ? angle_err : -angle_err;

    a->pid.Kp = 1.0f; a->pid.Ki = 0.05f; a->pid.Kd = 5.0f;
    if (abs_err > 15.0f) a->pid.output_max = 300;
    else                  a->pid.output_max = 150;

    a->correction = PID_Calc(&a->pid, angle_err);

    if (abs_err < g_angle_tolerance)
    {
        a->correction = 0;
        a->state = ANGLE_DONE;
    }

    return a->state;
}

float AngleTurn_GetError(AngleTurn *a)
{
    float abs_target = a->ref_angle + g_target_angle;
    return Angle_Error(abs_target, mpu6050.KalmanAngleZ);
}
