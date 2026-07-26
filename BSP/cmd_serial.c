#include "cmd_serial.h"
#include "usart.h"
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
        HAL_UART_Receive_IT(&huart8, &g_rx_byte, 1);  /* 继续接收 */
    }
}

void CmdSerial_Init(void)
{
    HAL_UART_Receive_IT(&huart8, &g_rx_byte, 1);
}

static Command serial_poll(void)
{
    if (!g_rx_flag) return CMD_NONE;
    g_rx_flag = 0;

    /* 解析单字符命令 */
    char *s = (char*)g_rx_buf;
    if (strcmp(s, "n") == 0) return CMD_NEXT;
    if (strcmp(s, "t") == 0) return CMD_TOGGLE;
    if (strcmp(s, "+") == 0) return CMD_UP;
    if (strcmp(s, "-") == 0) return CMD_DOWN;

    return CMD_NONE;
}

CommandSource g_src_serial = { .poll = serial_poll };
