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
static uint8_t  g_disp_rx[32];
static uint8_t  g_disp_rx_len;
static uint16_t g_disp_rx_total;

/* ---- 收到完整 F32C 帧时调用 ---- */
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
    g_disp->show_str(0,  0, "GIMBAL UART8->4");

    if (g_disp_rx_len) {
        int pos = 0;
        for (int i = 0; i < g_disp_rx_len && pos < 30; i++)
            pos += sprintf(buf + pos, "%02X", g_disp_rx[i]);
        buf[pos] = '\0';
    } else {
        sprintf(buf, "RX: --");
    }
    g_disp->show_str(0, 20, buf);

    sprintf(buf, "RX frames:%d", (int)g_disp_rx_total);
    g_disp->show_str(0, 40, buf);
    g_disp->flush();
}

static void gimbal_cmd(Command cmd, char data)
{
    (void)data;
    /* 命令系统保留 (n=切换模式, +/-=参数等), CMD_CUSTOM 不处理
       因为 UART8 每个字节已在 HAL_UART_RxCpltCallback 中原样转发到 UART4 */
    if (cmd == CMD_CUSTOM) return;
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
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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

        /* 原封不动转发到 UART4 (云台) */
        HAL_UART_Transmit(&huart4, &b, 1, 10);

        CmdSerial_FeedByte(b);
        HAL_UART_Receive_IT(&huart8, (uint8_t *)huart->pRxBuffPtr, 1);
        return;
    }
}
