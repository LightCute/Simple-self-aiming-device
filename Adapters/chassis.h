#ifndef ADAPTER_CHASSIS_H
#define ADAPTER_CHASSIS_H
#include "HAL/chassis.h"

extern Chassis g_chassis_inst;   /* 全局实例, main.c中用 &g_chassis_inst 注入 */
#endif
