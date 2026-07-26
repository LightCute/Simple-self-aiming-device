#ifndef HAL_COMMAND_H
#define HAL_COMMAND_H

typedef enum {
    CMD_NONE = 0,
    CMD_NEXT,       /* 切换下一模式 */
    CMD_TOGGLE,     /* 启动/停止/触发 */
    CMD_UP,         /* 参数+ */
    CMD_DOWN,       /* 参数- */
    CMD_CUSTOM,     /* 自定义命令, data中传字符 */
} Command;

typedef struct {
    Command (*poll)(char *data);   /* data输出: CMD_CUSTOM时携带的字符 */
} CommandSource;

#endif
