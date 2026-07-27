#ifndef HAL_TRACKER_H
#define HAL_TRACKER_H
#include <stdint.h>

typedef enum { TRACK_OK=0, TRACK_LEFT, TRACK_RIGHT, TRACK_TJUNC } TrackEvent;

typedef struct {
    float   correction;       /* 输出: 转向修正值 */
    int16_t base_speed;       /* 巡线基础速度 */
    TrackEvent (*update)(void *self);  /* ISR调用: 返回事件 */
    void (*init)(void *self);
} Tracker;

#endif
