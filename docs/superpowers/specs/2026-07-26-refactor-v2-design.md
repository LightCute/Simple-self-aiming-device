# 重构v2 设计

## 目标

逐模块重构，第一个模式只做 IMU 姿态角读取+显示+日志。基于接口编程(DIP)，换硬件只改注入行。

## 目录

```
HAL/         抽象接口层 (主逻辑依赖, 不改)
  imu.h        IMU 接口
  display.h    显示接口
  logger.h     日志接口

BSP/         驱动实现层 (换硬件只改这层)
  mpu6050.c/h   MPU6050 驱动 (保留)
  oled.c/h      SSD1306 驱动 (保留)
  imu_mpu6050.c MPU6050 实现 IMU 接口
  disp_oled.c   OLED 实现 Display 接口
  log_uart.c    UART 实现 Logger 接口

App/         应用层
  main.c       调度: ISR→IMU, while→Display+Logger
```

## 接口

```c
// IMU
typedef struct {
    float yaw, pitch, roll;
    uint8_t (*init)(void);
    void    (*update)(void);
} IMU;

// Display
typedef struct {
    void (*clear)(void);
    void (*show_str)(int x, int y, const char *s);
    void (*show_num)(int x, int y, float v, int decimals);
    void (*flush)(void);
} Display;

// Logger
typedef struct {
    void (*info)(const char *fmt, ...);
    void (*data)(const char *fmt, ...);
} Logger;
```

## 模式1: IMU_TEST

- TIM6 ISR 每10ms: imu->update()
- while: disp显示yaw/pitch/roll + log->data输出
