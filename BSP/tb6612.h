#include "HAL/motor.h"

#ifndef __TB6612_H__
#define __TB6612_H__

#include <stdint.h>
#include "main.h"
#include "tim.h"

/* 单个电机结构体（TB6612控制 + 编码器） */
typedef struct
{
    GPIO_TypeDef      *IN1_Port;        /* IN1 引脚端口 */
    uint16_t           IN1_Pin;         /* IN1 引脚编号 */
    GPIO_TypeDef      *IN2_Port;        /* IN2 引脚端口 */
    uint16_t           IN2_Pin;         /* IN2 引脚编号 */
    TIM_HandleTypeDef *htim;            /* PWM定时器句柄 */
    uint32_t           TIM_Channel;     /* PWM定时器通道 */
    TIM_HandleTypeDef *encoder_htim;    /* 编码器定时器句柄 */
    int8_t             encoder_dir;     /* 编码器方向系数: 1或-1，Forward时编码器读数应为正 */
    int8_t             motor_dir;       /* 电机安装方向系数: 1或-1，-1表示电机物理反向安装 */
    int32_t             prev_encoder;    /* 上一次编码器值（用于计算速度） */
    int32_t             speed;           /* 当前速度（编码器脉冲/10ms） */
} TB6612_Motor_t;

/* 双电机小车控制结构体 */
typedef struct
{
    TB6612_Motor_t MotorL;  /* 左电机 (TB6612 A通道: AIN1/AIN2 + LPWMA, 编码器: TIM5) */
    TB6612_Motor_t MotorR;  /* 右电机 (TB6612 B通道: BIN1/BIN2 + RPWMB, 编码器: TIM3) */
} TB6612_Car_t;

/* 全局默认小车实例 */
extern TB6612_Car_t g_car;

/* ========== 默认实例（操作 g_car，无需传指针）========== */
void     TB6612_Init(void);
void     TB6612_Run(int16_t left_speed, int16_t right_speed);
void     TB6612_Brake(void);
void     TB6612_Stop(void);
void     TB6612_GetEncoder(int32_t *left, int32_t *right);
void     TB6612_ResetEncoder(void);
void     TB6612_UpdateSpeed(void);
int32_t  TB6612_GetLeftSpeed(void);
int32_t  TB6612_GetRightSpeed(void);
void     TB6612_GetDistIncrement(int32_t *dl, int32_t *dr);  /* 返回本次增量绝对值 */

/* ========== 单电机控制（可传入自定义实例）========== */
void     TB6612_Motor_Init(TB6612_Motor_t *motor,
                           GPIO_TypeDef *IN1_Port, uint16_t IN1_Pin,
                           GPIO_TypeDef *IN2_Port, uint16_t IN2_Pin,
                           TIM_HandleTypeDef *htim, uint32_t TIM_Channel,
                           TIM_HandleTypeDef *encoder_htim,
                           int8_t encoder_dir, int8_t motor_dir);
void     TB6612_Motor_Forward(TB6612_Motor_t *motor, uint16_t speed);
void     TB6612_Motor_Reverse(TB6612_Motor_t *motor, uint16_t speed);
void     TB6612_Motor_Brake(TB6612_Motor_t *motor);
void     TB6612_Motor_Stop(TB6612_Motor_t *motor);
void     TB6612_Motor_SetSpeed(TB6612_Motor_t *motor, int16_t speed);
int32_t  TB6612_Motor_GetEncoder(TB6612_Motor_t *motor);
void     TB6612_Motor_ResetEncoder(TB6612_Motor_t *motor);
void     TB6612_Motor_UpdateSpeed(TB6612_Motor_t *motor);
int32_t  TB6612_Motor_GetSpeed(TB6612_Motor_t *motor);

/* ========== 小车控制（可传入自定义实例）========== */
void     TB6612_Car_Init(TB6612_Car_t *car);
void     TB6612_Car_Run(TB6612_Car_t *car, int16_t left_speed, int16_t right_speed);
void     TB6612_Car_Brake(TB6612_Car_t *car);
void     TB6612_Car_Stop(TB6612_Car_t *car);
void     TB6612_Car_GetEncoder(TB6612_Car_t *car, int32_t *left, int32_t *right);
void     TB6612_Car_ResetEncoder(TB6612_Car_t *car);
void     TB6612_Car_UpdateSpeed(TB6612_Car_t *car);
void     TB6612_Car_GetSpeed(TB6612_Car_t *car, int32_t *left, int32_t *right);

/* Motor 接口实例 */
extern Motor g_motor_tb6612;

#endif
