#ifndef BSP_LOG_UART_H
#define BSP_LOG_UART_H
#include "HAL/logger.h"

extern Logger g_log_uart;   /* 注入点: g_log = &g_log_uart */
#endif
