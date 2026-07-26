#ifndef HAL_MOTOR_H
#define HAL_MOTOR_H
#include <stdint.h>

/* 电机驱动抽象接口 */
typedef struct {
    void (*init)(void);
    void (*set_pwm)(int16_t left, int16_t right);   /* 原始PWM -999~999 */
    void (*stop)(void);
    void (*brake)(void);
    void (*get_encoder)(int32_t *l, int32_t *r);     /* 原始编码值 */
    void (*update_speed)(void);                       /* 读CNT→算speed */
    int16_t speed_l, speed_r;                         /* update后更新 */
} Motor;

#endif
