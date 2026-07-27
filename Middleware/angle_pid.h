#ifndef MID_ANGLE_PID_H
#define MID_ANGLE_PID_H
#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float integral_max, output_max;
    float correction;
    float angle_err;
    uint8_t done;
} AnglePID;

void AnglePID_Init(AnglePID *p);
float AnglePID_Update(AnglePID *p, float err);
#endif
