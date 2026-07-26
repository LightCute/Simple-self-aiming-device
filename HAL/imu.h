#ifndef HAL_IMU_H
#define HAL_IMU_H
#include <stdint.h>

typedef struct {
    float yaw, pitch, roll;     /* 姿态角(度), update后更新 */
    uint8_t (*init)(void);      /* 初始化, 返回0成功 */
    void    (*update)(void);    /* 读取传感器, 更新yaw/pitch/roll */
} IMU;

#endif
