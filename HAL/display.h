#ifndef HAL_DISPLAY_H
#define HAL_DISPLAY_H

typedef struct {
    void (*clear)(void);
    void (*show_str)(int x, int y, const char *s);
    void (*show_num)(int x, int y, float v, int dec);
    void (*flush)(void);
} Display;

#endif
