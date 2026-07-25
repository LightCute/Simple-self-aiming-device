#include "tb6612.h"

/* TIM1 ARR = 1000-1, 速度范围 0 ~ 999 */
#define TB6612_SPEED_MAX  999

/* 全局默认小车实例 */
TB6612_Car_t g_car;

/* ==================== 默认实例快捷函数 ==================== */

void TB6612_Init(void)           { TB6612_Car_Init(&g_car);                }
void TB6612_Run(int16_t l, int16_t r)    { TB6612_Car_Run(&g_car, l, r);        }
void TB6612_Brake(void)          { TB6612_Car_Brake(&g_car);              }
void TB6612_Stop(void)           { TB6612_Car_Stop(&g_car);               }
void TB6612_GetEncoder(int32_t *l, int32_t *r) { TB6612_Car_GetEncoder(&g_car, l, r); }
void TB6612_ResetEncoder(void)   { TB6612_Car_ResetEncoder(&g_car);       }
void TB6612_UpdateSpeed(void)    { TB6612_Car_UpdateSpeed(&g_car);        }
int32_t TB6612_GetLeftSpeed(void)  { return g_car.MotorL.speed;           }
int32_t TB6612_GetRightSpeed(void) { return g_car.MotorR.speed;           }

/*
 * 返回编码器累计值 (还原16bit硬件CNT: 前进0→65535, 后退65535→0)
 * 每10ms调用, 内部处理16bit翻转和符号转换
 */
void TB6612_GetDistIncrement(int32_t *dl, int32_t *dr)
{
    int32_t el, er;
    TB6612_GetEncoder(&el, &er);

    /* el<0(前进): 65535+el = 0→65535; el>0(后退): 直接输出 */
    *dl = (el < 0) ? (int32_t)(65535 + el) : el;
    *dr = (er < 0) ? (int32_t)(65535 + er) : er;
}

/* ==================== 单电机控制 ==================== */

/**
 * @brief 初始化电机GPIO、PWM通道和编码器定时器
 */
void TB6612_Motor_Init(TB6612_Motor_t *motor,
                       GPIO_TypeDef *IN1_Port, uint16_t IN1_Pin,
                       GPIO_TypeDef *IN2_Port, uint16_t IN2_Pin,
                       TIM_HandleTypeDef *htim, uint32_t TIM_Channel,
                       TIM_HandleTypeDef *encoder_htim,
                       int8_t encoder_dir, int8_t motor_dir)
{
    motor->IN1_Port      = IN1_Port;
    motor->IN1_Pin       = IN1_Pin;
    motor->IN2_Port      = IN2_Port;
    motor->IN2_Pin       = IN2_Pin;
    motor->htim          = htim;
    motor->TIM_Channel   = TIM_Channel;
    motor->encoder_htim  = encoder_htim;
    motor->encoder_dir   = encoder_dir;
    motor->motor_dir     = motor_dir;
    motor->prev_encoder  = 0;
    motor->speed         = 0;

    HAL_TIM_PWM_Start(htim, TIM_Channel);
    HAL_TIM_Encoder_Start(encoder_htim, TIM_CHANNEL_ALL);
    TB6612_Motor_Stop(motor);
}

/**
 * @brief 电机正转
 * @param speed 速度值 0 ~ 999
 */
void TB6612_Motor_Forward(TB6612_Motor_t *motor, uint16_t speed)
{
    if (speed > TB6612_SPEED_MAX) speed = TB6612_SPEED_MAX;

    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, speed);
}

/**
 * @brief 电机反转
 * @param speed 速度值 0 ~ 999
 */
void TB6612_Motor_Reverse(TB6612_Motor_t *motor, uint16_t speed)
{
    if (speed > TB6612_SPEED_MAX) speed = TB6612_SPEED_MAX;

    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, speed);
}

/**
 * @brief 电机制动（IN1=1, IN2=1, 短接制动）
 */
void TB6612_Motor_Brake(TB6612_Motor_t *motor)
{
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, 0);
}

/**
 * @brief 电机停止（IN1=0, IN2=0, 惯性滑行）
 */
void TB6612_Motor_Stop(TB6612_Motor_t *motor)
{
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->TIM_Channel, 0);
}

/**
 * @brief 带符号速度控制
 * @param speed 正数正转，负数反转
 */
void TB6612_Motor_SetSpeed(TB6612_Motor_t *motor, int16_t speed)
{
    speed *= motor->motor_dir;
    if (speed > 0)
        TB6612_Motor_Forward(motor, (uint16_t)speed);
    else if (speed < 0)
        TB6612_Motor_Reverse(motor, (uint16_t)(-speed));
    else
        TB6612_Motor_Brake(motor);
}

/**
 * @brief 读取编码器计数值
 */
int32_t TB6612_Motor_GetEncoder(TB6612_Motor_t *motor)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(motor->encoder_htim) * motor->encoder_dir;
}

/**
 * @brief 清零编码器计数值
 */
void TB6612_Motor_ResetEncoder(TB6612_Motor_t *motor)
{
    __HAL_TIM_SET_COUNTER(motor->encoder_htim, 0);
    motor->prev_encoder = 0;
    motor->speed = 0;
}

/**
 * @brief 更新电机速度（在TIM6中断中调用，每10ms一次）
 * @note  速度 = (当前编码器值 - 上次编码器值)，单位：脉冲/10ms
 */
void TB6612_Motor_UpdateSpeed(TB6612_Motor_t *motor)
{
    int32_t cur = TB6612_Motor_GetEncoder(motor);
    motor->speed = cur - motor->prev_encoder;
    motor->prev_encoder = cur;
}

/**
 * @brief 获取电机当前速度
 * @return 速度值，单位：编码器脉冲/10ms
 */
int32_t TB6612_Motor_GetSpeed(TB6612_Motor_t *motor)
{
    return motor->speed;
}

/* ==================== 双电机小车控制 ==================== */

/**
 * @brief 初始化双电机小车
 * @note  左电机: AIN1/AIN2 + LPWMA(TIM1_CH2), 编码器 TIM5(LE1A/LE2B)
 * @note  右电机: BIN1/BIN2 + RPWMB(TIM1_CH1), 编码器 TIM3(RE1A/RE2B)
 */
void TB6612_Car_Init(TB6612_Car_t *car)
{
    /* 左电机: AIN1=PE15, AIN2=PE12, LPWMA=TIM1_CH2(PE11), 编码器=TIM5 */
    TB6612_Motor_Init(&car->MotorL,
                      AIN1_GPIO_Port, AIN1_Pin,
                      AIN2_GPIO_Port, AIN2_Pin,
                      &htim1, TIM_CHANNEL_2,
                      &htim5, -1, 1);

    /* 右电机: BIN1=PE10, BIN2=PB2, RPWMB=TIM1_CH1(PE9), 编码器=TIM3 */
    TB6612_Motor_Init(&car->MotorR,
                      BIN1_GPIO_Port, BIN1_Pin,
                      BIN2_GPIO_Port, BIN2_Pin,
                      &htim1, TIM_CHANNEL_1,
                      &htim3, -1, -1);
}

/**
 * @brief 差速控制
 * @param left_speed  左轮速度
 * @param right_speed 右轮速度
 */
void TB6612_Car_Run(TB6612_Car_t *car, int16_t left_speed, int16_t right_speed)
{
    TB6612_Motor_SetSpeed(&car->MotorL, left_speed);
    TB6612_Motor_SetSpeed(&car->MotorR, right_speed);
}

void TB6612_Car_Brake(TB6612_Car_t *car)
{
    TB6612_Motor_Brake(&car->MotorL);
    TB6612_Motor_Brake(&car->MotorR);
}

void TB6612_Car_Stop(TB6612_Car_t *car)
{
    TB6612_Motor_Stop(&car->MotorL);
    TB6612_Motor_Stop(&car->MotorR);
}

void TB6612_Car_GetEncoder(TB6612_Car_t *car, int32_t *left, int32_t *right)
{
    *left  = TB6612_Motor_GetEncoder(&car->MotorL);
    *right = TB6612_Motor_GetEncoder(&car->MotorR);
}

void TB6612_Car_ResetEncoder(TB6612_Car_t *car)
{
    TB6612_Motor_ResetEncoder(&car->MotorL);
    TB6612_Motor_ResetEncoder(&car->MotorR);
}

void TB6612_Car_UpdateSpeed(TB6612_Car_t *car)
{
    TB6612_Motor_UpdateSpeed(&car->MotorL);
    TB6612_Motor_UpdateSpeed(&car->MotorR);
}

void TB6612_Car_GetSpeed(TB6612_Car_t *car, int32_t *left, int32_t *right)
{
    *left  = car->MotorL.speed;
    *right = car->MotorR.speed;
}
