#ifndef HAL_TURN_H
#define HAL_TURN_H
#include <stdint.h>

typedef enum { TURN_IDLE, TURN_RUNNING, TURN_DONE } TurnState;

typedef struct {
    float      correction;       /* 输出: 差速修正值 */
    float      angle_err;        /* 当前角度误差 (只读) */
    TurnState  state;
    void (*spot)(void *self, float delta_deg);    /* 原地转 */
    void (*arc)(void *self, float delta_deg, int16_t base_spd); /* 弧线转 */
    void (*update)(void *self);
    void (*init)(void *self);
} TurnCtrl;

#endif
