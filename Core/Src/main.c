/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - 调度器
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "oled.h"
#include "mpu6050.h"
#include "stdio.h"
#include "tb6612.h"
#include "tracking.h"
#include "speed_ctrl.h"
#include "angle_turn.h"
#include "line_track.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
MPU6050_t mpu6050;
char msg[64];

/* 模式: 0=速度环, 1=角度转弯, 2=巡线, 3=直行测距 */
uint8_t   g_op_mode = 0;
SpeedCtrl g_spd;
AngleTurn g_ang;
LineTrack g_trk;

int16_t  g_debug_speed = 20;
float    g_debug_angle = 90.0f;
uint8_t  g_debug_run   = 0;
int16_t  g_angle_base  = 10;

int32_t  g_dist_target = 500;
int32_t  g_dist_accum  = 0;
uint8_t  g_dist_run    = 0;

uint8_t  g_track_run   = 0;    /* 巡线启/停 */
/* USER CODE END PV */

void SystemClock_Config(void);
static void MPU_Config(void);

/* USER CODE BEGIN 0 */
static const char *mode_name(uint8_t m)
{
    static const char *names[] = {"SPEED","ANGLE","TRACK","DIST"};
    return (m < 4) ? names[m] : "????";
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t tick_1ms = 0;
    if (htim != &htim6) return;
    tick_1ms++;
    if (tick_1ms >= 10)
    {
        tick_1ms = 0;
        MPU6050_Read_All(&hi2c2, &mpu6050);
        TB6612_UpdateSpeed();

        AngleTurn_Beep(&g_ang);

        switch (g_op_mode)
        {
        case 0: /* 速度环 */
            SpeedCtrl_Update(&g_spd);
            break;

        case 1: /* 角度转弯 */
            if (g_ang.state == ANGLE_RUNNING)
            {
                AngleTurn_Update(&g_ang);
                SpeedCtrl_SetTargets(&g_spd,
                    (int16_t)(g_angle_base + g_ang.correction)/10,
                    (int16_t)(g_angle_base - g_ang.correction)/10);
                SpeedCtrl_Update(&g_spd);
            }
            else if (g_ang.state == ANGLE_DONE)
            {
                SpeedCtrl_SetTargets(&g_spd, 0, 0);
                SpeedCtrl_Update(&g_spd);
            }
            break;

        case 2: /* 巡线 */
            if (g_track_run)
            {
                TrackEvent ev = LineTrack_Update(&g_trk);
                if (ev == TRACK_OK)
                {
                    SpeedCtrl_SetTargets(&g_spd,
                        (int16_t)(g_trk.base_speed - g_trk.correction),
                        (int16_t)(g_trk.base_speed + g_trk.correction));
                    SpeedCtrl_Update(&g_spd);
                }
                else if (ev == TRACK_LEFT || ev == TRACK_RIGHT)
                {
                    SpeedCtrl_SetTargets(&g_spd, 0, 0);
                    SpeedCtrl_Update(&g_spd);
                    g_ang.beep_cnt = 10;
                    g_track_run = 0;
                }
            }
            break;

        case 3: /* 直行测距 */
            if (g_dist_run)
            {
                int32_t al_raw = TB6612_GetLeftSpeed();
                int32_t ar_raw = TB6612_GetRightSpeed();
                int32_t al = (al_raw < 0) ? -al_raw : al_raw;
                int32_t ar = (ar_raw < 0) ? -ar_raw : ar_raw;
                g_dist_accum += (al + ar) / 2;
                static uint8_t dist_log = 0;
                if (++dist_log >= 5) { dist_log = 0;
                    printf("[DIST] acc=%d/%d al=%d(%d) ar=%d(%d)\r\n",
                           (int)g_dist_accum, (int)g_dist_target,
                           al, al_raw, ar, ar_raw);
                }
                if (g_dist_accum >= g_dist_target)
                {
                    SpeedCtrl_SetTargets(&g_spd, 0, 0);
                    SpeedCtrl_Update(&g_spd);
                    g_dist_run = 0;
                    g_ang.beep_cnt = 10;
                }
                else
                {
                    SpeedCtrl_SetTargets(&g_spd, g_debug_speed, g_debug_speed);
                    SpeedCtrl_Update(&g_spd);
                }
            }
            break;

            default:
            break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY3_Pin)
    {
        g_op_mode = (g_op_mode + 1) % 4;
        g_debug_run = 0;
        g_track_run = 0;
        g_dist_run  = 0;
        SpeedCtrl_SetTargets(&g_spd, 0, 0);
        SpeedCtrl_Update(&g_spd);
        TB6612_Brake();
        TB6612_ResetEncoder();
    }
    else if (GPIO_Pin == KEY4_Pin)
    {
        switch (g_op_mode)
        {
        case 0: /* 速度环: 启/停 */
            g_debug_run = !g_debug_run;
            if (g_debug_run)
            {
                SpeedCtrl_SetTargets(&g_spd, g_debug_speed, g_debug_speed);
                TB6612_ResetEncoder();
            }
            else { SpeedCtrl_SetTargets(&g_spd, 0, 0); SpeedCtrl_Update(&g_spd); }
            break;
        case 1: /* 角度转弯: 触发 */
            {
                static int8_t dir = 1;
                AngleTurn_Start(&g_ang, g_debug_angle * dir);
                dir = -dir;
            }
            break;
        case 2: /* 巡线: 启/停 */
            g_track_run = !g_track_run;
            if (g_track_run) LineTrack_Init(&g_trk);
            else { SpeedCtrl_SetTargets(&g_spd, 0, 0); SpeedCtrl_Update(&g_spd); }
            break;
        case 3: /* 直行测距: 启/停 */
            g_dist_run = !g_dist_run;
            if (g_dist_run) {
                g_dist_accum = 0;
                TB6612_ResetEncoder();
                SpeedCtrl_SetTargets(&g_spd, g_debug_speed, g_debug_speed);
                printf("[DIST] Start, target=%d\r\n", (int)g_dist_target);
            } else { SpeedCtrl_SetTargets(&g_spd, 0, 0); SpeedCtrl_Update(&g_spd); }
            break;
        }
    }
    else if (GPIO_Pin == KEY1_Pin)
    {
        switch (g_op_mode)
        {
        case 0: if (g_debug_speed > -900) g_debug_speed -= 5;
                if (g_debug_run) SpeedCtrl_SetTargets(&g_spd, g_debug_speed, g_debug_speed);
                break;
        case 1: g_debug_angle -= 15; if (g_debug_angle < -180) g_debug_angle = -180; break;
        case 3: if (g_dist_target > 100) g_dist_target -= 100; break;
        }
    }
    else if (GPIO_Pin == KEY2_Pin)
    {
        switch (g_op_mode)
        {
        case 0: if (g_debug_speed < 900) g_debug_speed += 5;
                if (g_debug_run) SpeedCtrl_SetTargets(&g_spd, g_debug_speed, g_debug_speed);
                break;
        case 1: g_debug_angle += 15; if (g_debug_angle > 180) g_debug_angle = 180; break;
        case 3: if (g_dist_target < 10000) g_dist_target += 100; break;
        }
    }
}
/* USER CODE END 0 */

int main(void)
{
  MPU_Config();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_UART8_Init();

  /* USER CODE BEGIN 2 */
  OLED_Init();
  while (MPU6050_Init(&hi2c2))
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(499);
  }
  MPU6050_CalibrateGyro(&hi2c2, 500);
  HAL_Delay(999);
  TB6612_Init();
  TB6612_ResetEncoder();
  SpeedCtrl_Init(&g_spd);
  AngleTurn_Init(&g_ang);
  LineTrack_Init(&g_trk);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  while (1)
  {
    OLED_NewFrame();
    OLED_PrintASCIIString(0, 0, (char*)mode_name(g_op_mode), &afont12x6, OLED_COLOR_NORMAL);

    switch (g_op_mode)
    {
    case 0:
      OLED_PrintASCIIString(40, 14, g_debug_run ? "RUN" : "STOP", &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "Spd:%d K1-5 K2+5", g_debug_speed);
      OLED_PrintASCIIString(0, 14, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TL:%d AL:%d", g_spd.target_l, g_spd.actual_l);
      OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TR:%d AR:%d", g_spd.target_r, g_spd.actual_r);
      OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "PWM L:%d R:%d", g_spd.pwm_l, g_spd.pwm_r);
      OLED_PrintASCIIString(0, 50, msg, &afont12x6, OLED_COLOR_NORMAL);
      /* printf("Spd:%d TL:%d AL:%d TR:%d AR:%d PWM:%d,%d\r\n", ...); */
      break;

    case 1:
      sprintf(msg, "Set:%.0f Base:%d", g_debug_angle, (int)g_angle_base);
      OLED_PrintASCIIString(0, 14, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "AngleZ:%.1f", mpu6050.KalmanAngleZ);
      OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "Err:%.1f Tol:%.0f %s", AngleTurn_GetError(&g_ang),
              g_angle_tolerance,
              (g_ang.state == ANGLE_RUNNING) ? "RUN" : "DONE");
      OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "PWM L:%d R:%d", g_spd.pwm_l, g_spd.pwm_r);
      OLED_PrintASCIIString(0, 50, msg, &afont12x6, OLED_COLOR_NORMAL);
      break;

    case 2:
      OLED_PrintASCIIString(40, 14, g_track_run ? "RUN" : "STOP", &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TRACK B:%d", g_trk.base_speed);
      OLED_PrintASCIIString(0, 14, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TL:%d AL:%d", g_spd.target_l, g_spd.actual_l);
      OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TR:%d AR:%d", g_spd.target_r, g_spd.actual_r);
      OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "Corr:%.0f", g_trk.correction);
      OLED_PrintASCIIString(0, 50, msg, &afont12x6, OLED_COLOR_NORMAL);
      /* printf("B:%d Corr:%.0f ...", ...); */
      break;

    case 3:
      OLED_PrintASCIIString(40, 14, g_dist_run ? "RUN" : "STOP", &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "Tgt:%d K1-100 K2+100", (int)g_dist_target);
      OLED_PrintASCIIString(0, 14, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "Acc:%d", (int)g_dist_accum);
      OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TL:%d AL:%d", g_spd.target_l, g_spd.actual_l);
      OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TR:%d AR:%d", g_spd.target_r, g_spd.actual_r);
      OLED_PrintASCIIString(0, 50, msg, &afont12x6, OLED_COLOR_NORMAL);
      break;
    }

    OLED_ShowFrame();
    HAL_Delay(10);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4; RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2; RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void) { __disable_irq(); while(1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
