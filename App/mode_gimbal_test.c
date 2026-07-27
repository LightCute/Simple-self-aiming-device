#include "mode_gimbal_test.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include "HAL/command.h"
#include "Adapters/cmd_serial.h"
#include "usart.h"          /* huart4 */
#include <stdio.h>
#include <string.h>

extern Display *g_disp;
extern Logger  *g_log;

/* ---- DMA RX buffer (file scope, callback needs it) ---- */
static uint8_t  g_rx_buf[64];
static uint16_t g_rx_len = 0;

/* ---- OLED last-data cache (updated by callback, painted by ui) ---- */
static uint8_t  g_disp_tx[32];   /* last TX hex bytes */
static uint8_t  g_disp_tx_len;
static uint8_t  g_disp_rx[32];   /* last RX hex bytes */
static uint8_t  g_disp_rx_len;
static uint16_t g_disp_rx_total; /* total RX frames */

/* ================================================================ */
/*  Helpers                                                          */
/* ================================================================ */
static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int hex_to_bytes(const char *s, uint8_t *out, int max_len)
{
    int len = 0;
    while (*s && len < max_len) {
        while (*s == ' ' || *s == '\t') s++;
        if (!*s) break;
        int hi = hex_nibble(*s++);
        while (*s == ' ' || *s == '\t') s++;
        int lo = hex_nibble(*s++);
        if (hi < 0 || lo < 0) return -1;
        out[len++] = (uint8_t)((hi << 4) | lo);
    }
    return len;
}

static void rx_restart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_rx_buf, sizeof(g_rx_buf));
}

/* ================================================================ */
/*  Callbacks                                                        */
/* ================================================================ */
static void gimbal_enter(void)
{
    rx_restart();
    g_disp_rx_total = 0;
    g_disp_tx_len   = 0;
    g_disp_rx_len   = 0;
    g_log->info("Enter GIMBAL mode");
}

static void gimbal_isr(void) { /* no-op */ }

static void gimbal_ui(void)
{
    char buf[32];
    g_disp->clear();
    g_disp->show_str(0,  0, "GIMBAL TEST");

    /* last TX */
    if (g_disp_tx_len) {
        int pos = 0;
        for (int i = 0; i < g_disp_tx_len && pos < 30; i++)
            pos += sprintf(buf + pos, "%02X", g_disp_tx[i]);
        buf[pos] = '\0';
    } else {
        sprintf(buf, "TX: --");
    }
    g_disp->show_str(0, 16, buf);

    /* last RX */
    if (g_disp_rx_len) {
        int pos = 0;
        for (int i = 0; i < g_disp_rx_len && pos < 30; i++)
            pos += sprintf(buf + pos, "%02X", g_disp_rx[i]);
        buf[pos] = '\0';
    } else {
        sprintf(buf, "RX: --");
    }
    g_disp->show_str(0, 34, buf);

    /* stats */
    sprintf(buf, "RX frames:%d", (int)g_disp_rx_total);
    g_disp->show_str(0, 52, buf);
    g_disp->flush();
}

static void gimbal_cmd(Command cmd, char data)
{
    (void)data;
    if (cmd != CMD_CUSTOM) return;

    const char *str = CmdSerial_GetString();
    uint8_t buf[64];
    int len = hex_to_bytes(str, buf, sizeof(buf));

    if (len <= 0) {
        g_log->info("[GIMBAL] bad hex: \"%s\"", str);
        return;
    }

    if (HAL_UART_Transmit(&huart4, buf, (uint16_t)len, 100) != HAL_OK) {
        g_log->info("[GIMBAL] TX FAIL");
        return;
    }

    /* cache for OLED */
    memcpy(g_disp_tx, buf, len < 32 ? len : 32);
    g_disp_tx_len = (uint8_t)(len < 32 ? len : 32);

    /* log */
    char hex[96] = {0};
    int pos = 0;
    for (int i = 0; i < len && pos < 90; i++)
        pos += sprintf(hex + pos, " %02X", buf[i]);
    g_log->data("[GIMBAL TX] len=%d:%s", len, hex);
}

const AppMode mode_gimbal_test = {
    .name       = "GIMBAL",
    .on_enter   = gimbal_enter,
    .on_isr     = gimbal_isr,
    .on_ui      = gimbal_ui,
    .on_command = gimbal_cmd,
};

/* ================================================================ */
/*  HAL callbacks                                                    */
/* ================================================================ */
void f32c_uart_send(uint8_t *data, uint8_t len)
{
    (void)data; (void)len;  /* stub, satisfies linker */
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART4) return;

    g_rx_len = Size;
    g_disp_rx_total++;

    /* cache for OLED */
    uint16_t n = Size < 32 ? Size : 32;
    memcpy(g_disp_rx, g_rx_buf, n);
    g_disp_rx_len = (uint8_t)n;

    /* log: raw hex */
    char hex[96] = {0};
    int pos = 0;
    for (uint16_t i = 0; i < Size && pos < 90; i++)
        pos += sprintf(hex + pos, " %02X", g_rx_buf[i]);
    g_log->data("[GIMBAL RX] len=%d:%s", (int)Size, hex);

    rx_restart();
}
