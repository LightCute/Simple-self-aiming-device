#ifndef MID_SPEED_CTRL_H
#define MID_SPEED_CTRL_H
#include <stdint.h>
#include "HAL/chassis.h"  /* for PID_Params */

typedef struct {
    PID_Params pid_l, pid_r;
    float      integral_l, prev_err_l;
    float      integral_r, prev_err_r;
    int16_t    target_l, target_r;
    int16_t    actual_l, actual_r;
    int16_t    pwm_l, pwm_r;
    uint8_t    enabled;
} SpeedCtrl;

void SpeedCtrl_Init(SpeedCtrl *s);
void SpeedCtrl_SetTargets(SpeedCtrl *s, int16_t l, int16_t r);
void SpeedCtrl_SetActual(SpeedCtrl *s, int16_t l, int16_t r);
void SpeedCtrl_ClearIntegral(SpeedCtrl *s);
void SpeedCtrl_Update(SpeedCtrl *s);
#endif
