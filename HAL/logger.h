#ifndef HAL_LOGGER_H
#define HAL_LOGGER_H

typedef struct {
    void (*info)(const char *fmt, ...);
    void (*data)(const char *fmt, ...);
} Logger;

#endif
