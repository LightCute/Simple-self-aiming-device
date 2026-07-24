#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

#include "stm32h7xx_hal.h"

void MotorTest_Init(void);
void MotorTest_Run(void);   /* 在主循环中调用，执行测试序列 */

#endif /* __MOTOR_TEST_H */
