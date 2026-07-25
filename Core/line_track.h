#ifndef __LINE_TRACK_H
#define __LINE_TRACK_H

#include "speed_ctrl.h"   /* for PID_t */

typedef enum { TRACK_OK=0, TRACK_LEFT, TRACK_RIGHT, TRACK_TJUNC } TrackEvent;

typedef struct {
    PID_t   steer_pid;
    int16_t base_speed;
    float   correction;
} LineTrack;

void       LineTrack_Init(LineTrack *t);
TrackEvent LineTrack_Update(LineTrack *t);

#endif
