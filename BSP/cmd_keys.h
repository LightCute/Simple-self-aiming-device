#ifndef BSP_CMD_KEYS_H
#define BSP_CMD_KEYS_H
#include "HAL/command.h"

extern CommandSource g_src_keys;

void CmdKeys_NotifyKEY3(void);
void CmdKeys_NotifyKEY4(void);
#endif
