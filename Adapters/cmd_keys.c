#include "cmd_keys.h"
#include "main.h"

/* KEY3=EXTI, KEY1/2=轮询, KEY4=EXTI */
static uint8_t g_key3_flag = 0;
static uint8_t g_key4_flag = 0;

/* EXTI回调中置标志 */
static void notify_key3(void) { g_key3_flag = 1; }
static void notify_key4(void) { g_key4_flag = 1; }

/* GPIO EXTI 回调: 直接处理按键 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY3_Pin) notify_key3();
    if (GPIO_Pin == KEY4_Pin) notify_key4();
}

static Command key_poll(char *data)
{
    (void)data;  /* 按键不产生自定义数据 */

    /* KEY3: 切换模式 */
    if (g_key3_flag) { g_key3_flag = 0; return CMD_NEXT; }

    /* KEY4: 启动/触发 */
    if (g_key4_flag) { g_key4_flag = 0; return CMD_TOGGLE; }

    /* KEY1/KEY2: 轮询(无EXTI) */
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(30);  /* 消抖 */
        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
            return CMD_DOWN;
    }
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
        HAL_Delay(30);
        if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
            return CMD_UP;
    }

    return CMD_NONE;
}

CommandSource g_src_keys = { .poll = key_poll };
