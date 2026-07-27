#include "line_track.h"
#include "tracking.h"
#include "Middleware/line_pid.h"

static LinePID g_line_pid;

static void lt_init(void *self) {
    (void)self;
    LinePID_Init(&g_line_pid);
}

static TrackEvent lt_update(void *self) {
    (void)self;
    Tracking_UpdateOffset();
    g_tracker_inst.offset     = g_tracking_offset;
    g_tracker_inst.correction = LinePID_Update(&g_line_pid, g_tracking_offset);

    uint8_t sharp = Tracking_IsSharpTurn();
    if (sharp == 1) return TRACK_LEFT;
    if (sharp == 2) return TRACK_RIGHT;
    return TRACK_OK;
}

Tracker g_tracker_inst = {
    .correction = 0,
    .base_speed = 20,
    .init   = lt_init,
    .update = lt_update,
};
