/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c  - 重构v2: 纯接口调度器
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>

/* --- HAL 抽象接口 (纯虚) --- */
#include "HAL/imu.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include "HAL/app_mode.h"
#include "HAL/chassis.h"

/* --- BSP 注入 (只在main.c出现一次) --- */
#include "imu_mpu6050.h"
#include "disp_oled.h"
#include "log_uart.h"
#include "cmd_keys.h"
#include "cmd_serial.h"
#include "chassis_tb6612.h"

/* --- App 模式 --- */
#include "mode_imu_test.h"
#include "mode_speed.h"
#include "mode_chassis_test.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* --- 依赖注入: 接口指针指向具体实现 --- */
IMU      *g_imu     = &g_imu_mpu6050;
Display  *g_disp    = &g_disp_oled;
Logger   *g_log     = &g_log_uart;
Chassis  *g_chassis = &g_chassis_tb6612;

/* --- 命令源 --- */
static CommandSource *g_sources[] = { &g_src_keys, &g_src_serial };
#define SRC_COUNT 2

/* --- 模式注册 (加新模式只需加一行) --- */
static const AppMode *g_modes[] = { &mode_imu_test, &mode_speed, &mode_chassis_test };
#define MODE_COUNT 3
static uint8_t g_cur_mode = 0;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MPU_Config(void);

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t tick_1ms = 0;
    if (htim != &htim6) return;
    tick_1ms++;
    if (tick_1ms >= 10) {
        tick_1ms = 0;
        g_imu->update();
        g_chassis->update();             /* PID闭环 */
        g_modes[g_cur_mode]->on_isr();
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
  g_disp->init();
  while (g_imu->init()) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_Delay(499);
  }
  g_imu->calibrate(500);
  HAL_Delay(999);
  g_chassis->init();
  setvbuf(stdout, NULL, _IONBF, 0);  /* printf无缓冲, 实时输出 */
  CmdSerial_Init();
  HAL_TIM_Base_Start_IT(&htim6);

  g_modes[g_cur_mode]->on_enter();
  /* USER CODE END 2 */

  while (1)
  {
    /* --- 轮询命令源 --- */
    Command cmd = CMD_NONE;
    char    cmd_data = 0;
    for (int i = 0; i < SRC_COUNT; i++) {
        char d = 0;
        Command c = g_sources[i]->poll(&d);
        if (c != CMD_NONE) { cmd = c; cmd_data = d; }
    }

    /* --- 系统命令: 模式切换 --- */
    if (cmd == CMD_NEXT) {
        g_cur_mode = (g_cur_mode + 1) % MODE_COUNT;
        g_modes[g_cur_mode]->on_enter();
    }
    /* --- 传给当前模式处理 --- */
    else if (cmd != CMD_NONE) {
        g_modes[g_cur_mode]->on_command(cmd, cmd_data);
    }

    /* --- UI 刷新 --- */
    g_modes[g_cur_mode]->on_ui();
    HAL_Delay(10);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
