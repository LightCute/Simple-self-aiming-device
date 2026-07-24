#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32h7xx_hal.h"

/* PID结构体 */
typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float integral_max;
    float output_max;
} PID_t;

/* ========== 速度环参数 ========== */
extern float g_speed_Kp_L, g_speed_Ki_L, g_speed_Kd_L;
extern float g_speed_Kp_R, g_speed_Ki_R, g_speed_Kd_R;

extern int16_t g_target_left;
extern int16_t g_target_right;
extern int16_t g_actual_left;
extern int16_t g_actual_right;
extern int16_t g_pwm_left;
extern int16_t g_pwm_right;

/* ========== 角度环参数 ========== */
extern float g_angle_Kp;
extern float g_angle_Ki;
extern float g_angle_Kd;
extern float g_target_angle;   /* 目标相对角度（度），如 90=右转90° */
extern float g_angle_error;    /* 当前角度误差（度），用于串口观察 */
extern float g_angle_tolerance; /* 角度误差容限（度），到达后退出角度环 */

/* ========== 转向环参数（tracking巡线） ========== */
extern float g_steer_Kp;
extern float g_steer_Ki;
extern float g_steer_Kd;

/* 模式选择: 0=仅速度环, 1=角度环, 2=转向环(tracking) */
extern uint8_t g_steer_mode;

/* 基础速度 */
extern int16_t g_base_speed;
extern int16_t g_speed_init;   /* 巡线默认速度，debug时可在线修改 */
extern float   g_arc_ratio;   /* 弧线速度比例 = g_speed_init × 此值 */


/* 小车状态 */
typedef enum {
    STATE_LINE = 0,  /* 巡线 */
    STATE_TURN,      /* 角度旋转中（预留） */
    STATE_STOP       /* 停车 */
} CarState_t;

extern CarState_t g_car_state;

/* 圈数计数 */
extern uint8_t g_lap_target;   /* 目标圈数 1~5 */
extern uint8_t g_lap_count;    /* 已完成圈数 */
extern uint8_t g_turn_count;   /* 当前圈内右直角弯计数 0~3 */
extern uint8_t g_turn_is_lap;  /* 本次转弯计入圈数标志 */

void Control_Init(void);
void Control_Update(void);     /* TIM6中断中调用，每10ms */
void Control_SetRelativeAngle(float delta);  /* 设置相对旋转角度 */
void Control_Stop(void);                     /* 立即停车 */
void Control_Start(void);                    /* 启动，进入转向环模式 */
void Control(void); /* 主控制函数 */

#endif /* __CONTROL_H */
