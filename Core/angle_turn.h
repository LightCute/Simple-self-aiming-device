#ifndef __ANGLE_TURN_H
#define __ANGLE_TURN_H

#include "speed_ctrl.h"   /* for PID_t */

typedef enum { ANGLE_IDLE, ANGLE_RUNNING, ANGLE_DONE } AngleState;

typedef struct {
    PID_t      pid;
    float      ref_angle;
    float      correction;
    AngleState state;
} AngleTurn;

void       AngleTurn_Init(AngleTurn *a);
void       AngleTurn_Start(AngleTurn *a, float delta_deg);
AngleState AngleTurn_Update(AngleTurn *a);
float      AngleTurn_GetError(AngleTurn *a);

#endif
