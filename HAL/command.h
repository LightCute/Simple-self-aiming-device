#ifndef HAL_COMMAND_H
#define HAL_COMMAND_H

/* 统一命令枚举 */
typedef enum {
    CMD_NONE = 0,
    CMD_NEXT,       /* 切换下一模式 */
    CMD_TOGGLE,     /* 启动/停止/触发 */
    CMD_UP,         /* 参数+ */
    CMD_DOWN,       /* 参数- */
} Command;

/* 命令源抽象接口 */
typedef struct {
    Command (*poll)(void);   /* 非阻塞, 无命令返回 CMD_NONE */
} CommandSource;

#endif
