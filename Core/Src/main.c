/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "mpu6050.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "tb6612.h"
#include "tracking.h"
#include "control.h"
#include "motor_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t mpu6050;
char msg[64];

/* 工作模式: 0=待机, 1=Z轴转弯调试, 2=传感器调试, 3=转向环巡线, 4=速度环, 5=角度环, 6=电机测试 */
uint8_t  g_op_mode      = 0;
int16_t  g_debug_speed  = 200;   /* 速度环调试目标速度 */
float    g_debug_angle  = 90.0f;  /* 角度环调试目标角度，正=右转 */
uint8_t  g_debug_run    = 0;     /* 速度环调试: 1=运行中 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- 模式名 ---- */
static const char *mode_name(uint8_t m)
{
    static const char *names[] = {"READY","Z-TURN","SENSOR","STEER","SPEED","ANGLE","MOTOR"};
    return (m < 7) ? names[m] : "????";
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t tick_1ms = 0;

    if (htim == &htim6)
    {
        tick_1ms++;
        // g_tjunction = Tracking_IsTJunction();
        if (tick_1ms >= 10)
        {
            tick_1ms = 0;
            MPU6050_Read_All(&hi2c2, &mpu6050);

            switch (g_op_mode)
            {
            case 3: /* 转向环巡线: 完整Control() */
                Control();
                break;
            case 4: /* 速度环: g_steer_mode=0, 仅速度PID */
                if (g_debug_run)
                {
                    TB6612_UpdateSpeed();
                    Control_Update();
                }
                break;
            case 5: /* 角度环: g_steer_mode=1, 角度PID */
                TB6612_UpdateSpeed();
                Control_Update();
                break;
            default:
                break;
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY3_Pin)
    {
        /* KEY3: 循环切换模式 */
        g_op_mode = (g_op_mode + 1) % 7;
        g_debug_run = 0;
        Control_Stop();
        TB6612_ResetEncoder();
        if (g_op_mode == 6) MotorTest_Init();
    }
    else if (GPIO_Pin == KEY4_Pin)
    {
        /* KEY4: 模式内操作 */
        switch (g_op_mode)
        {
        case 3: /* 转向环: 启动巡线 */
            g_car_state = STATE_LINE;
            Control_Start();
            break;
        case 4: /* 速度环: 启/停 */
            g_debug_run = !g_debug_run;
            if (g_debug_run)
            {
                g_steer_mode  = 0;
                g_base_speed  = g_debug_speed;
                g_target_left  = g_debug_speed;
                g_target_right = g_debug_speed;
                TB6612_ResetEncoder();
            }
            else
            {
                Control_Stop();
            }
            break;
        case 5: /* 角度环: 触发旋转, 每次切换方向 */
            {
                static int8_t dir = 1;
                Control_SetRelativeAngle(g_debug_angle * dir);
                g_car_state = STATE_TURN;
                dir = -dir;
            }
            break;
        default:
            break;
        }
    }
    else if (GPIO_Pin == KEY1_Pin)
    {
        /* KEY1: 模式相关操作 */
        switch (g_op_mode)
        {
        case 0: /* 待机: 圈数-1 */
            if (g_lap_target > 1) g_lap_target--; else g_lap_target = 5;
            break;
        case 4: /* 速度环: 降速50 */
            if (g_debug_speed > 50) {
                g_debug_speed -= 50;
                if (g_debug_run) { g_base_speed = g_debug_speed;
                                   g_target_left = g_target_right = g_debug_speed; }
            }
            break;
        case 5: /* 角度环: 角度-15° */
            g_debug_angle -= 15.0f;
            if (g_debug_angle < -180) g_debug_angle = -180;
            break;
        default: break;
        }
    }
    else if (GPIO_Pin == KEY2_Pin)
    {
        /* KEY2: 模式相关操作 */
        switch (g_op_mode)
        {
        case 0: /* 待机: 圈数+1 */
            if (g_lap_target < 5) g_lap_target++; else g_lap_target = 1;
            break;
        case 4: /* 速度环: 升速50 */
            if (g_debug_speed < 900) {
                g_debug_speed += 50;
                if (g_debug_run) { g_base_speed = g_debug_speed;
                                   g_target_left = g_target_right = g_debug_speed; }
            }
            break;
        case 5: /* 角度环: 角度+15° */
            g_debug_angle += 15.0f;
            if (g_debug_angle > 180) g_debug_angle = 180;
            break;
        default: break;
        }
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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
  Control_Init();
  MotorTest_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OLED_NewFrame();

    /* ---- 顶部: 模式名 ---- */
    OLED_PrintASCIIString(0, 0, (char*)mode_name(g_op_mode),
                          &afont12x6, OLED_COLOR_NORMAL);

    switch (g_op_mode)
    {
    /* ============ 模式1: Z轴转弯调试 ============ */
    case 1:
      {
        float cur_z  = mpu6050.KalmanAngleZ;
        static float prev_z = 0.0f;
        static float accum  = 0.0f;
        static uint8_t stable = 0;
        static uint8_t turn_cnt = 0;
        float dz = (cur_z > prev_z) ? (cur_z - prev_z) : (prev_z - cur_z);

        if (dz > 1.0f) { accum += dz; stable = 0; }
        else { if (stable < 10) stable++; }

        if (stable == 10 && accum > 60.0f) { turn_cnt++; accum = 0.0f; }
        if (stable == 10) accum = 0.0f;
        prev_z = cur_z;

        sprintf(msg, "Z:%.1f dz:%.1f", cur_z, dz);
        OLED_PrintASCIIString(0, 14, msg, &afont12x6, OLED_COLOR_NORMAL);
        sprintf(msg, "Acc:%.0f St:%d", accum, stable);
        OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
        sprintf(msg, "Turn:%d", turn_cnt);
        OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);

        if (stable == 10 && accum > 60.0f)
          OLED_PrintASCIIString(20, 50, "TURN!", &afont12x6, OLED_COLOR_REVERSED);

        printf("Z=%.1f dz=%.1f acc=%.0f st=%d turn=%d\r\n",
               cur_z, dz, accum, stable, turn_cnt);
      }
      break;

    /* ============ 模式2: 传感器调试 ============ */
    case 2:
      {
        uint8_t raw = Tracking_GetRaw();
        char bits[9];
        for (int i = 0; i < 8; i++) bits[7-i] = (raw & (1<<i)) ? '1' : '0';
        bits[8] = '\0';
        OLED_PrintASCIIString(0, 14, bits, &afont12x6, OLED_COLOR_NORMAL);

        uint8_t left_cnt  = (raw & 0x01)+(raw>>1 & 0x01)+(raw>>2 & 0x01)+(raw>>3 & 0x01);
        uint8_t right_cnt = (raw>>4 & 0x01)+(raw>>5 & 0x01)+(raw>>6 & 0x01)+(raw>>7 & 0x01);
        sprintf(msg, "Off:%d L:%d R:%d", (int)g_tracking_offset, left_cnt, right_cnt);
        OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
        sprintf(msg, "Z:%.1f", mpu6050.KalmanAngleZ);
        OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);

        uint8_t sharp = Tracking_IsSharpTurn();
        if (sharp != 0)
        {
          OLED_PrintASCIIString(20, 50, sharp==1 ? "<<LEFT" : "RIGHT>>",
                                &afont12x6, OLED_COLOR_REVERSED);
        }

        printf("Raw=0x%02X Off=%d L=%d R=%d Z=%.1f",
               raw, (int)g_tracking_offset, left_cnt, right_cnt, mpu6050.KalmanAngleZ);
        if (sharp == 2)      printf("  **RIGHT**");
        else if (sharp == 1) printf("  **LEFT**");
        printf("\r\n");
      }
      break;

    /* ============ 模式0: 待机 ============ */
    case 0:
      sprintf(msg, "LAPS: %d", g_lap_target);
      OLED_PrintASCIIString(20, 20, msg, &afont12x6, OLED_COLOR_NORMAL);
      OLED_PrintASCIIString(15, 35, "KEY4: START", &afont12x6, OLED_COLOR_NORMAL);
      break;

    /* ============ 模式3: 转向环巡线 ============ */
    case 3:
      {
        const char *st = (g_car_state == STATE_LINE) ? "LINE" : (g_car_state == STATE_TURN) ? "TURN" : "STOP";
        sprintf(msg, "%s L%d/%d", st, g_lap_count + 1, g_lap_target);
        OLED_PrintASCIIString(10, 0, msg, &afont12x6, OLED_COLOR_NORMAL);

        /* 8路传感器原始状态: 1=黑 0=白, 直接位显示 */
        uint8_t raw = Tracking_GetRaw();
        char bits[9];
        for (int i = 0; i < 8; i++) bits[7-i] = (raw & (1<<i)) ? '1' : '0';
        bits[8] = '\0';
        OLED_PrintASCIIString(0, 14, bits, &afont12x6, OLED_COLOR_NORMAL);
      }
      sprintf(msg, "T:%d A:%d", g_target_left, g_actual_left);
      OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "T:%d A:%d", g_target_right, g_actual_right);
      OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "B:%d Off:%d", (int)g_base_speed, (int)g_tracking_offset);
      OLED_PrintASCIIString(0, 50, msg, &afont12x6, OLED_COLOR_NORMAL);
      /* 诊断: Z轴角度 + 偏移 + 传感器 + 圈数 */
      printf("Z:%.1f Off:%d Raw:0x%02X T:%d L:%d/%d\r\n",
             mpu6050.KalmanAngleZ, (int)g_tracking_offset, Tracking_GetRaw(),
             g_turn_count, g_lap_count + 1, g_lap_target);
      break;

    /* ============ 模式4: 速度环调试 ============ */
    case 4:
      OLED_PrintASCIIString(50, 14, g_debug_run ? "RUN" : "STOP",
                            &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "Spd:%d K1- K2+", g_debug_speed);
      OLED_PrintASCIIString(0, 14, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TL:%d AL:%d", g_target_left,  g_actual_left);
      OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "TR:%d AR:%d", g_target_right, g_actual_right);
      OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "PWM L:%d R:%d", g_pwm_left, g_pwm_right);
      OLED_PrintASCIIString(0, 50, msg, &afont12x6, OLED_COLOR_NORMAL);
      // printf("%d,%d,%d,%d,%d,%d\r\n", ...); /* 暂时关闭 */
      break;

    /* ============ 模式5: 角度环调试 ============ */
    case 5:
      sprintf(msg, "Set:%.0f K1- K2+", g_debug_angle);
      OLED_PrintASCIIString(0, 14, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "AngleZ:%.1f", mpu6050.KalmanAngleZ);
      OLED_PrintASCIIString(0, 26, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "Err:%.1f %s", g_angle_error,
              (g_steer_mode == 1) ? "ROTATING" : "DONE");
      OLED_PrintASCIIString(0, 38, msg, &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "PWM L:%d R:%d", g_pwm_left, g_pwm_right);
      OLED_PrintASCIIString(0, 50, msg, &afont12x6, OLED_COLOR_NORMAL);
      printf("%.1f,%.1f,%.1f,%d,%d\r\n",
             g_target_angle, mpu6050.KalmanAngleZ, g_angle_error,
             g_pwm_left, g_pwm_right);
      break;

    /* ============ 模式6: 开环电机测试 ============ */
    case 6:
      OLED_PrintASCIIString(20, 14, "MOTOR TEST", &afont12x6, OLED_COLOR_NORMAL);
      sprintf(msg, "Enc L:%ld", (long)g_car.MotorL.prev_encoder);
      OLED_PrintASCIIString(0, 32, msg, &afont12x6, OLED_COLOR_NORMAL);
      MotorTest_Run();
      break;
    }

    OLED_ShowFrame();
    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
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
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
