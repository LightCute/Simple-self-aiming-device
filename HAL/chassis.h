#ifndef HAL_CHASSIS_H
#define HAL_CHASSIS_H
#include <stdint.h>

/* PID 参数 (可在线修改) */
typedef struct {
    float Kp, Ki, Kd;
    float integral_max, output_max;
} PID_Params;

/* 运动底盘抽象接口 */
typedef struct {
    /* 基础运动: 目标速度 = 编码脉冲/10ms */
    void (*set_speeds)(int16_t left, int16_t right);
    void (*stop)(void);       /* 惯性滑行 */
    void (*brake)(void);      /* 短接制动 */

    /* 编码器: 读当前累计值 */
    void (*get_encoders)(int32_t *left, int32_t *right);

    /* 实际速度 (只读) */
    int16_t actual_l, actual_r;

    /* PID 参数 (Keil调试时直接改) */
    PID_Params *pid_left;
    PID_Params *pid_right;
} Chassis;

#endif
