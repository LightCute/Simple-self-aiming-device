#include "imu_mpu6050.h"
#include "mpu6050.h"
#include "i2c.h"

static MPU6050_t g_mpu;

static uint8_t mpu_init(void) {
    return MPU6050_Init(&hi2c2);
}

static void mpu_calibrate(int samples) {
    MPU6050_CalibrateGyro(&hi2c2, (uint16_t)samples);
}

static void mpu_update(void) {
    MPU6050_Read_All(&hi2c2, &g_mpu);
    g_imu_mpu6050.yaw   = (float)g_mpu.KalmanAngleZ;
    g_imu_mpu6050.pitch = (float)g_mpu.KalmanAngleY;
    g_imu_mpu6050.roll  = (float)g_mpu.KalmanAngleX;
}

IMU g_imu_mpu6050 = {
    .yaw = 0, .pitch = 0, .roll = 0,
    .init      = mpu_init,
    .calibrate = mpu_calibrate,
    .update    = mpu_update,
};
