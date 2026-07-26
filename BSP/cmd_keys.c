#include "cmd_keys.h"
#include "main.h"

/* KEY3=EXTI, KEY1/2=轮询, KEY4=EXTI */
static uint8_t g_key3_flag = 0;
static uint8_t g_key4_flag = 0;

/* EXTI回调中置标志 */
void CmdKeys_NotifyKEY3(void) { g_key3_flag = 1; }
void CmdKeys_NotifyKEY4(void) { g_key4_flag = 1; }

static Command key_poll(void)
{
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
