#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32h7xx_hal.h"

#define TRACKING_CH_NUM 8

extern const int8_t g_tracking_deviation[TRACKING_CH_NUM];
extern int32_t       g_tracking_offset;
extern uint8_t        g_tjunction;      /* T形路口标志，1=检测到 */

void  Tracking_UpdateOffset(void);
uint8_t Tracking_IsTJunction(void);  /* 返回1=检测到T形路口 */
uint8_t Tracking_GetRaw(void);       /* 返回8路传感器原始状态，bit0=RED1 .. bit7=RED8 */
uint8_t Tracking_IsSharpTurn(void);  /* 返回0=无, 1=左直角弯, 2=右直角弯 */

#endif /* __TRACKING_H */
