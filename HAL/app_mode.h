#ifndef HAL_APP_MODE_H
#define HAL_APP_MODE_H
#include "command.h"

/* 应用程序模式抽象接口 */
typedef struct AppMode {
    const char *name;               /* OLED 显示名 */
    void (*on_enter)(void);         /* 进入模式时调用 */
    void (*on_isr)(void);           /* TIM6 ISR, 每10ms */
    void (*on_ui)(void);            /* while循环: OLED+日志 */
    void (*on_command)(Command cmd); /* 统一命令入口 */
} AppMode;

#endif
