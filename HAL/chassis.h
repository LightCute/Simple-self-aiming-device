#ifndef HAL_CHASSIS_H
#define HAL_CHASSIS_H
#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral_max, output_max;
} PID_Params;

typedef struct {
    void (*init)(void);
    void (*update)(void);                                /* PID闭环, 每10ms */
    void (*set_speeds)(int16_t left, int16_t right);
    void (*stop)(void);
    void (*brake)(void);
    void (*get_encoders)(int32_t *left, int32_t *right);

    int16_t actual_l, actual_r;
    int16_t target_l, target_r;      /* debug可读 */
    uint8_t enabled;                 /* 1=PID运行 */
    PID_Params *pid_left;
    PID_Params *pid_right;
} Chassis;

#endif
