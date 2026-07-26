#ifndef HAL_IMU_H
#define HAL_IMU_H
#include <stdint.h>

typedef struct {
    float yaw, pitch, roll;
    uint8_t (*init)(void);            /* 返回0成功, 非0重试 */
    void    (*calibrate)(int samples); /* 陀螺校准 */
    void    (*update)(void);           /* 更新 yaw/pitch/roll */
} IMU;

#endif
