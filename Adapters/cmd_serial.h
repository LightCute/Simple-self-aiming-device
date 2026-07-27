#ifndef BSP_CMD_SERIAL_H
#define BSP_CMD_SERIAL_H
#include <stdint.h>
#include "HAL/command.h"

extern CommandSource g_src_serial;
void CmdSerial_Init(void);
void CmdSerial_FeedByte(uint8_t byte);   /* 由 HAL_UART_RxCpltCallback 喂入每个字节 */
const char *CmdSerial_GetString(void);   /* 获取原始字符串命令 */
#endif
