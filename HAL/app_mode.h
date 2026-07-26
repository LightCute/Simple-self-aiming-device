#ifndef HAL_APP_MODE_H
#define HAL_APP_MODE_H
#include "command.h"

typedef struct AppMode {
    const char *name;
    void (*on_enter)(void);
    void (*on_isr)(void);
    void (*on_ui)(void);
    void (*on_command)(Command cmd, char data);   /* data: CMD_CUSTOM时的输入字符 */
} AppMode;

#endif
