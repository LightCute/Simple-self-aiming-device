#include "log_uart.h"
#include "stdio.h"
#include <stdarg.h>

static void log_info(const char *fmt, ...) {
    printf("[INFO] ");
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap); va_end(ap);
    printf("\r\n");
}

static void log_data(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    vprintf(fmt, ap); va_end(ap);
    printf("\r\n");
}

Logger g_log_uart = {
    .info = log_info,
    .data = log_data,
};
