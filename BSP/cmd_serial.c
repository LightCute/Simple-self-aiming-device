#include "cmd_serial.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static uint8_t  g_rx_byte = 0;
static volatile uint8_t g_rx_flag = 0;
static uint8_t g_rx_buf[16];
static uint8_t g_rx_idx = 0;

/* UART8 接收中断回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart8)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   /* LED翻转=收到字节 */
        char ch = (char)g_rx_byte;
        if (ch == '\r' || ch == '\n')
        {
            g_rx_buf[g_rx_idx] = '\0';
            g_rx_flag = 1;
            g_rx_idx = 0;
        }
        else if (g_rx_idx < 15)
        {
            g_rx_buf[g_rx_idx++] = ch;
        }
        HAL_UART_Receive_IT(&huart8, &g_rx_byte, 1);
    }
}

void CmdSerial_Init(void)
{
    HAL_UART_Receive_IT(&huart8, &g_rx_byte, 1);
}

static Command serial_poll(char *data)
{
    if (!g_rx_flag) return CMD_NONE;
    g_rx_flag = 0;

    char *s = (char*)g_rx_buf;
    if (strcmp(s, "n") == 0) return CMD_NEXT;
    if (strcmp(s, "t") == 0) return CMD_TOGGLE;
    if (strcmp(s, "+") == 0) return CMD_UP;
    if (strcmp(s, "-") == 0) return CMD_DOWN;

    /* 非系统命令 → CMD_CUSTOM, 传首字符 */
    if (s[0]) {
        *data = s[0];
        printf("?%s\r\n", s);  /* 回显未知命令, 方便调试 */
        return CMD_CUSTOM;
    }
    return CMD_NONE;
}

/* 供模式获取原始字符串命令 */
const char *CmdSerial_GetString(void)
{
    return (const char *)g_rx_buf;
}

CommandSource g_src_serial = { .poll = serial_poll };
