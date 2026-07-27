/**
  ******************************************************************************
  * @file    mode_gimbal_test.c
  * @brief   Gimbal debug mode — UART8 hex → UART4 passthrough + RX hex dump
  *
  *          Enter hex bytes on UART8 serial terminal (e.g. "7A 01 06 7D 7B"),
  *          they get sent raw to UART4. UART4 RX data printed as hex dump.
  ******************************************************************************
  */
#include "mode_gimbal_test.h"
#include "HAL/command.h"
#include "Adapters/cmd_serial.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart4;

/* DMA RX buffer — file scope so callback can access it */
static uint8_t g_rx_buf[64];
static uint8_t g_rx_active = 0;  /* 1 = DMA RX is running */

/* ================================================================ */
/*  Helpers                                                          */
/* ================================================================ */

/** Convert a single hex char to nibble, returns -1 on error */
static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/** Parse hex string (with optional spaces) → binary, returns byte count, -1 on error */
static int hex_to_bytes(const char *s, uint8_t *out, int max_len)
{
    int len = 0;
    while (*s) {
        /* skip spaces */
        while (*s == ' ' || *s == '\t') s++;
        if (!*s) break;

        int hi = hex_nibble(*s++);
        if (hi < 0) return -1;

        /* skip spaces between nibbles */
        while (*s == ' ' || *s == '\t') s++;

        int lo = hex_nibble(*s++);
        if (lo < 0) return -1;

        if (len >= max_len) return -1;
        out[len++] = (uint8_t)((hi << 4) | lo);
    }
    return len;
}

static void rx_restart(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_rx_buf, sizeof(g_rx_buf));
    g_rx_active = 1;
}

/* ================================================================ */
/*  Mode callbacks                                                   */
/* ================================================================ */
static void gimbal_enter(void)
{
    rx_restart();
    printf("[GIMBAL] ready — type hex (e.g. 7A 01 06 7D 7B) to send on UART4\r\n");
    fflush(stdout);
}

static void gimbal_isr(void) { /* no-op */ }

static void gimbal_ui(void) { /* no-op — RX callback handles printing */ }

static void gimbal_cmd(Command cmd, char data)
{
    (void)data;

    if (cmd != CMD_CUSTOM) return;

    const char *str = CmdSerial_GetString();
    uint8_t buf[64];
    int len = hex_to_bytes(str, buf, sizeof(buf));

    if (len <= 0) {
        printf("[GIMBAL] bad hex: \"%s\"\r\n", str);
        fflush(stdout);
        return;
    }

    /* Send raw bytes to gimbal via UART4 */
    if (HAL_UART_Transmit(&huart4, buf, (uint16_t)len, 100) == HAL_OK) {
        printf("[GIMBAL TX] len=%d:", len);
        for (int i = 0; i < len; i++) printf(" %02X", buf[i]);
        printf("\r\n");
    } else {
        printf("[GIMBAL TX] FAIL\r\n");
    }
    fflush(stdout);
}

const AppMode mode_gimbal_test = {
    .name       = "GIMBAL",
    .on_enter   = gimbal_enter,
    .on_isr     = gimbal_isr,
    .on_ui      = gimbal_ui,
    .on_command = gimbal_cmd,
};

/* ================================================================ */
/*  HAL callbacks (always active, regardless of mode)                */
/* ================================================================ */

/** F32C protocol layer TX stub — not used yet, satisfies linker */
void f32c_uart_send(uint8_t *data, uint8_t len)
{
    (void)data; (void)len;
}

/** UART4 DMA RX — hex dump received data */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART4) return;

    printf("[GIMBAL RX] len=%d:", (int)Size);
    for (uint16_t i = 0; i < Size && i < sizeof(g_rx_buf); i++) {
        printf(" %02X", g_rx_buf[i]);
    }
    printf("\r\n");
    fflush(stdout);

    rx_restart();
}
