#ifndef BSP_CMD_SERIAL_H
#define BSP_CMD_SERIAL_H
#include "HAL/command.h"

extern CommandSource g_src_serial;
void CmdSerial_Init(void);
const char *CmdSerial_GetString(void);  /* 获取原始字符串命令 */
#endif
