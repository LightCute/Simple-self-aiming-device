#ifndef MID_LINE_PID_H
#define MID_LINE_PID_H
#include <stdint.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral, prev_error;
    float integral_max, output_max;
    float correction;         /* 输出: 转向修正值 */
} LinePID;

void LinePID_Init(LinePID *p);
float LinePID_Update(LinePID *p, int32_t offset);  /* 输入传感器偏移, 输出修正值 */
#endif
