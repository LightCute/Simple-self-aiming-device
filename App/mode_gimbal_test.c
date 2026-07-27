#include "mode_gimbal_test.h"
#include "HAL/gimbal.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include "Adapters/cmd_serial.h"
#include <stdio.h>
#include <string.h>

extern Gimbal  *g_gimbal;
extern Display *g_disp;
extern Logger  *g_log;

static void gmt_enter(void)
{
    extern struct __UART_HandleTypeDef huart4;
    g_gimbal->init(g_gimbal, &huart4);   /* blocking ~1500ms motor boot */
    g_gimbal->sync(g_gimbal);             /* start position sync */
    g_log->info("Enter GIMBAL mode");
}

static void gmt_isr(void)
{
    /* Poll runs in on_ui (calls HAL_UART_Transmit, must be main-loop context) */
}

static void gmt_ui(void)
{
    g_gimbal->poll(g_gimbal);  /* non-blocking, one step per call */

    /* OLED display */
    char buf[32];
    g_disp->clear();
    g_disp->show_str(0, 0, "GIMBAL");
    sprintf(buf, "X:%4.1f->%4.1f",
            g_gimbal->get_current(g_gimbal, GIMBAL_AXIS_X) / 10.0f,
            g_gimbal->get_target(g_gimbal, GIMBAL_AXIS_X) / 10.0f);
    g_disp->show_str(0, 18, buf);
    sprintf(buf, "Y:%4.1f->%4.1f",
            g_gimbal->get_current(g_gimbal, GIMBAL_AXIS_Y) / 10.0f,
            g_gimbal->get_target(g_gimbal, GIMBAL_AXIS_Y) / 10.0f);
    g_disp->show_str(0, 36, buf);
    sprintf(buf, "spd:%d %s",
            (int)g_gimbal->get_speed(g_gimbal),
            g_gimbal->is_init_done(g_gimbal) ? "OK" : "");
    g_disp->show_str(0, 52, buf);
    g_disp->flush();
}

static void gmt_cmd(Command cmd, char data)
{
    (void)data;

    if (cmd == CMD_TOGGLE) {  /* KEY4: re-sync */
        g_gimbal->sync(g_gimbal);
        g_log->info("GIMBAL re-sync");
        return;
    }

    if (cmd == CMD_CUSTOM) {
        const char *s = CmdSerial_GetString();

        /* ---- ? query current position ---- */
        if (s[0] == '?') {
            g_log->info("X=%ld Y=%ld V=%d",
                        (long)g_gimbal->get_current(g_gimbal, GIMBAL_AXIS_X),
                        (long)g_gimbal->get_current(g_gimbal, GIMBAL_AXIS_Y),
                        (int)g_gimbal->get_speed(g_gimbal));
            return;
        }

        /* ---- H home: go to (0, 0) ---- */
        if (s[0] == 'H' || s[0] == 'h') {
            g_gimbal->set_angle(g_gimbal, GIMBAL_AXIS_X, 0);
            g_gimbal->set_angle(g_gimbal, GIMBAL_AXIS_Y, 0);
            g_log->info("GIMBAL home X=0 Y=0");
            return;
        }

        /* ---- EN enable motors ---- */
        if (strcmp(s, "EN") == 0 || strcmp(s, "en") == 0) {
            g_gimbal->enable(g_gimbal);
            g_log->info("GIMBAL motors enabled");
            return;
        }

        /* ---- V <rpm> set speed ---- */
        int ival;
        if (sscanf(s, "V%*[ :]%d", &ival) == 1 || sscanf(s, "v%*[ :]%d", &ival) == 1) {
            if (ival > 0) {
                g_gimbal->set_speed(g_gimbal, (int16_t)ival);
                g_log->info("GIMBAL speed=%d", ival);
            }
            return;
        }

        /* ---- XY <x> <y> set both axes (supports XY:200:0) ---- */
        int v1, v2;
        if (sscanf(s, "XY%*[ :]%d%*[ :]%d", &v1, &v2) == 2 ||
            sscanf(s, "xy%*[ :]%d%*[ :]%d", &v1, &v2) == 2) {
            g_gimbal->set_angle(g_gimbal, GIMBAL_AXIS_X, v1);
            g_gimbal->set_angle(g_gimbal, GIMBAL_AXIS_Y, v2);
            g_log->info("GIMBAL XY=%d,%d", v1, v2);
            return;
        }

        /* ---- X <pos> set X axis (pos in deci-degrees: 200 = 20.0°; supports X:200) ---- */
        if (sscanf(s, "X%*[ :]%d", &ival) == 1 || sscanf(s, "x%*[ :]%d", &ival) == 1) {
            g_gimbal->set_angle(g_gimbal, GIMBAL_AXIS_X, ival);
            g_log->info("GIMBAL X=%d cur=%ld", ival,
                        (long)g_gimbal->get_current(g_gimbal, GIMBAL_AXIS_X));
            return;
        }

        /* ---- Y <pos> set Y axis ---- */
        if (sscanf(s, "Y%*[ :]%d", &ival) == 1 || sscanf(s, "y%*[ :]%d", &ival) == 1) {
            g_gimbal->set_angle(g_gimbal, GIMBAL_AXIS_Y, ival);
            g_log->info("GIMBAL Y=%d cur=%ld", ival,
                        (long)g_gimbal->get_current(g_gimbal, GIMBAL_AXIS_Y));
            return;
        }

        /* ---- DX <delta> relative move X (supports DX:50) ---- */
        if (sscanf(s, "DX%*[ :]%d", &ival) == 1 || sscanf(s, "dx%*[ :]%d", &ival) == 1) {
            g_gimbal->move_delta(g_gimbal, GIMBAL_AXIS_X, ival);
            g_log->info("GIMBAL DX=%d tgt=%ld", ival,
                        (long)g_gimbal->get_target(g_gimbal, GIMBAL_AXIS_X));
            return;
        }

        /* ---- DY <delta> relative move Y ---- */
        if (sscanf(s, "DY%*[ :]%d", &ival) == 1 || sscanf(s, "dy%*[ :]%d", &ival) == 1) {
            g_gimbal->move_delta(g_gimbal, GIMBAL_AXIS_Y, ival);
            g_log->info("GIMBAL DY=%d tgt=%ld", ival,
                        (long)g_gimbal->get_target(g_gimbal, GIMBAL_AXIS_Y));
            return;
        }

        /* ---- S [slot] save bookmark (default slot 0) ---- */
        if (s[0] == 'S' || s[0] == 's') {
            int slot = 0;
            sscanf(s + 1, "%d", &slot);
            if (slot < 0 || slot >= GIMBAL_BOOKMARK_MAX) {
                g_log->info("GIMBAL bookmark slot out of range");
                return;
            }
            if (g_gimbal->save_bookmark(g_gimbal, (uint8_t)slot) == 0)
                g_log->info("GIMBAL saved to slot %d", slot);
            else
                g_log->info("GIMBAL save bookmark failed");
            return;
        }

        /* ---- G [slot] go to bookmark ---- */
        if (s[0] == 'G' || s[0] == 'g') {
            int slot = 0;
            sscanf(s + 1, "%d", &slot);
            if (slot < 0 || slot >= GIMBAL_BOOKMARK_MAX) {
                g_log->info("GIMBAL bookmark slot out of range");
                return;
            }
            int32_t x, y;
            if (g_gimbal->get_bookmark(g_gimbal, (uint8_t)slot, &x, &y)) {
                g_gimbal->go_bookmark(g_gimbal, (uint8_t)slot);
                g_log->info("GIMBAL go slot %d: X=%ld Y=%ld", slot, (long)x, (long)y);
            } else {
                g_log->info("GIMBAL slot %d empty", slot);
            }
            return;
        }

        /* ---- L list all bookmarks ---- */
        if (s[0] == 'L' || s[0] == 'l') {
            for (int i = 0; i < GIMBAL_BOOKMARK_MAX; i++) {
                int32_t x, y;
                if (g_gimbal->get_bookmark(g_gimbal, (uint8_t)i, &x, &y)) {
                    g_log->info("GIMBAL slot %d: X=%ld Y=%ld", i, (long)x, (long)y);
                }
            }
            return;
        }

        /* ---- LIMIT X|Y <min> <max> | LIMIT X|Y OFF ---- */
        if (strncmp(s, "LIMIT ", 6) == 0 || strncmp(s, "limit ", 6) == 0) {
            char axis_ch;
            char rest[32] = {0};
            if (sscanf(s + 6, "%c %31[^\n]", &axis_ch, rest) < 2) {
                g_log->info("GIMBAL LIMIT: usage LIMIT X|Y <min> <max>|OFF");
                return;
            }
            uint8_t axis = 0;
            if (axis_ch == 'X' || axis_ch == 'x') axis = GIMBAL_AXIS_X;
            if (axis_ch == 'Y' || axis_ch == 'y') axis = GIMBAL_AXIS_Y;
            if (!axis) { g_log->info("GIMBAL LIMIT: axis must be X or Y"); return; }

            if (strcmp(rest, "OFF") == 0 || strcmp(rest, "off") == 0) {
                g_gimbal->disable_limit(g_gimbal, axis);
                g_log->info("GIMBAL limit %c disabled", axis_ch);
                return;
            }
            int min, max;
            if (sscanf(rest, "%d %d", &min, &max) == 2) {
                g_gimbal->set_limit(g_gimbal, axis, min, max);
                g_log->info("GIMBAL limit %c: %d ~ %d", axis_ch, min, max);
            }
            return;
        }
    }
}

const AppMode mode_gimbal_test = {
    .name       = "GIMBAL",
    .on_enter   = gmt_enter,
    .on_isr     = gmt_isr,
    .on_ui      = gmt_ui,
    .on_command = gmt_cmd,
};
