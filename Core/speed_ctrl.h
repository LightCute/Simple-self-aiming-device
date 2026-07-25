#ifndef __SPEED_CTRL_H
#define __SPEED_CTRL_H

#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float integral_max, output_max;
} PID_t;

typedef struct {
    PID_t    pid_l, pid_r;
    int16_t  target_l, target_r;
    int16_t  actual_l, actual_r;
    int16_t  pwm_l, pwm_r;
} SpeedCtrl;

void SpeedCtrl_Init(SpeedCtrl *s);
void SpeedCtrl_SetTargets(SpeedCtrl *s, int16_t left, int16_t right);
void SpeedCtrl_Update(SpeedCtrl *s);

#endif
