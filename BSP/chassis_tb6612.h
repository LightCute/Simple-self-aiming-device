#ifndef BSP_CHASSIS_TB6612_H
#define BSP_CHASSIS_TB6612_H
#include "HAL/chassis.h"

extern Chassis g_chassis_tb6612;

/* 每10ms调用: 速度闭环PID */
void ChassisTB_Update(Chassis *c);
#endif
