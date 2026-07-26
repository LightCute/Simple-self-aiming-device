#include "disp_oled.h"
#include "oled.h"
#include "stdio.h"

static void disp_clear(void) { OLED_NewFrame(); }

static void disp_show_str(int x, int y, const char *s) {
    OLED_PrintASCIIString(x, y, (char*)s, &afont12x6, OLED_COLOR_NORMAL);
}

static void disp_show_num(int x, int y, float v, int dec) {
    char buf[16];
    if (dec == 0)      sprintf(buf, "%.0f", v);
    else if (dec == 1) sprintf(buf, "%.1f", v);
    else               sprintf(buf, "%.2f", v);
    OLED_PrintASCIIString(x, y, buf, &afont12x6, OLED_COLOR_NORMAL);
}

static void disp_flush(void) { OLED_ShowFrame(); }

Display g_disp_oled = {
    .init     = OLED_Init,
    .clear    = disp_clear,
    .show_str = disp_show_str,
    .show_num = disp_show_num,
    .flush    = disp_flush,
};
