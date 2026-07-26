#include "mode_imu_test.h"
#include "HAL/imu.h"
#include "HAL/display.h"
#include "HAL/logger.h"
#include <stdio.h>

/* 外部注入的全局接口指针 */
extern IMU     *g_imu;
extern Display *g_disp;
extern Logger  *g_log;

static void imu_enter(void) {
    g_log->info("Enter IMU mode");
}

static void imu_isr(void) {
    g_imu->update();
}

static void imu_ui(void) {
    char buf[16];
    g_disp->clear();
    g_disp->show_str(0, 0, "IMU");

    sprintf(buf, "Y:%.1f", (double)g_imu->yaw);
    g_disp->show_str(0, 14, buf);
    sprintf(buf, "P:%.1f", (double)g_imu->pitch);
    g_disp->show_str(0, 26, buf);
    sprintf(buf, "R:%.1f", (double)g_imu->roll);
    g_disp->show_str(0, 38, buf);

    g_disp->flush();

    g_log->data("Y:%.1f P:%.1f R:%.1f",
                (double)g_imu->yaw, (double)g_imu->pitch, (double)g_imu->roll);
}

static void imu_cmd(Command cmd) {
    (void)cmd;  /* 本模式不处理额外命令 */
}

const AppMode mode_imu_test = {
    .name       = "IMU",
    .on_enter   = imu_enter,
    .on_isr     = imu_isr,
    .on_ui      = imu_ui,
    .on_command = imu_cmd,
};
