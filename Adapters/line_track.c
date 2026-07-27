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

    /* 连续帧防抖: 同方向连续3帧才确认 */
    static uint8_t  debounce_cnt = 0;
    static uint8_t  last_sharp   = 0;
    uint8_t sharp = Tracking_IsSharpTurn();

    if (sharp == last_sharp && sharp != 0) {
        if (++debounce_cnt >= 3) {
            debounce_cnt = 0;
            return (sharp == 1) ? TRACK_LEFT : TRACK_RIGHT;
        }
    } else {
        debounce_cnt = (sharp != 0) ? 1 : 0;
        last_sharp   = sharp;
    }
    return TRACK_OK;
}

Tracker g_tracker_inst = {
    .correction = 0,
    .base_speed = 20,
    .init   = lt_init,
    .update = lt_update,
};
