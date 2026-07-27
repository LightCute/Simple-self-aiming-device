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

/* ---- UART4 中断接收: 帧缓冲 ---- */
static uint8_t  g_rx_byte;          /* UART4 单字节接收 */
static uint8_t  g_frame[64];        /* F32C 帧组装缓冲区 */
static uint8_t  g_frame_idx;
static uint8_t  g_in_frame;

/* ---- OLED 缓存 ---- */
static uint8_t  g_disp_tx[32];
static uint8_t  g_disp_tx_len;
static uint8_t  g_disp_rx[32];
static uint8_t  g_disp_rx_len;
static uint16_t g_disp_rx_total;

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

/* ---- 收到完整帧时调用 ---- */
static void on_frame_received(uint8_t len)
{
    g_disp_rx_total++;

    /* 缓存到 OLED */
    uint8_t n = len < 32 ? len : 32;
    memcpy(g_disp_rx, g_frame, n);
    g_disp_rx_len = n;

    /* 日志 */
    char hex[96] = {0};
    int pos = 0;
    for (uint8_t i = 0; i < len && pos < 90; i++)
        pos += sprintf(hex + pos, " %02X", g_frame[i]);
    g_log->data("[GIMBAL RX] len=%d:%s", (int)len, hex);
}

/* ================================================================ */
/*  Callbacks                                                        */
/* ================================================================ */
static void gimbal_enter(void)
{
    g_in_frame  = 0;
    g_frame_idx = 0;
    g_disp_rx_total = 0;
    g_disp_tx_len   = 0;
    g_disp_rx_len   = 0;

    /* 启动 UART4 单字节中断接收 */
    HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1);

    g_log->info("Enter GIMBAL mode");
}

static void gimbal_isr(void) { /* no-op */ }

static void gimbal_ui(void)
{
    char buf[32];
    g_disp->clear();
    g_disp->show_str(0,  0, "GIMBAL TEST");

    if (g_disp_tx_len) {
        int pos = 0;
        for (int i = 0; i < g_disp_tx_len && pos < 30; i++)
            pos += sprintf(buf + pos, "%02X", g_disp_tx[i]);
        buf[pos] = '\0';
    } else {
        sprintf(buf, "TX: --");
    }
    g_disp->show_str(0, 16, buf);

    if (g_disp_rx_len) {
        int pos = 0;
        for (int i = 0; i < g_disp_rx_len && pos < 30; i++)
            pos += sprintf(buf + pos, "%02X", g_disp_rx[i]);
        buf[pos] = '\0';
    } else {
        sprintf(buf, "RX: --");
    }
    g_disp->show_str(0, 34, buf);

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

    memcpy(g_disp_tx, buf, len < 32 ? len : 32);
    g_disp_tx_len = (uint8_t)(len < 32 ? len : 32);

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

/**
  * @brief  统一 UART 接收中断回调 — UART4 (云台帧组装) + UART8 (命令解析)
  *         pRxBuffPtr 指向各 UART 各自的 1 字节接收缓冲区
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t b = *(uint8_t *)huart->pRxBuffPtr;

    if (huart->Instance == UART4) {
        if (b == 0x7A) {
            g_in_frame  = 1;
            g_frame_idx = 0;
        }
        if (g_in_frame && g_frame_idx < sizeof(g_frame)) {
            g_frame[g_frame_idx++] = b;
        }
        if (b == 0x7B && g_in_frame && g_frame_idx >= 9) {
            on_frame_received(g_frame_idx);
            g_in_frame  = 0;
            g_frame_idx = 0;
        }
        HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1);
        return;
    }

    if (huart->Instance == UART8) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        CmdSerial_FeedByte(b);
        /* 重新使能中断, 沿用 CmdSerial_Init 设置的缓冲区指针 */
        HAL_UART_Receive_IT(&huart8, (uint8_t *)huart->pRxBuffPtr, 1);
        return;
    }
}
