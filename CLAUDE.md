# TICup2026 巡线小车 — STM32H7 工程笔记

## 项目概况

- **平台**: STM32H723 (实际芯片 STM32H750VBTx)
- **IDE**: MDK-ARM (Keil), 由 STM32CubeMX 生成初始代码
- **用途**: TICup2026 竞赛巡线小车

## 架构（重构v2，SOLID + 依赖注入）

```
HAL/          纯抽象接口 (零实现, 零依赖)
  imu.h       IMU 传感器接口
  display.h   显示接口
  logger.h    日志接口
  command.h   命令枚举 + CommandSource 接口
  app_mode.h  应用模式接口 (on_enter/on_isr/on_ui/on_command)
  chassis.h   底盘接口 (set_speeds/stop/brake/get_encoders+PID参数)
  motor.h     电机驱动接口 (set_pwm/update_speed)

BSP/          裸驱动 (只操作寄存器, 换硬件只改这里)
  mpu6050.c/h      MPU6050 I2C 驱动
  oled.c/h         SSD1306 OLED 驱动
  tb6612.c/h       TB6612 电机驱动 + Motor 接口实现
  font.c/h         字库

Adapters/     接口适配器 (实现 HAL 接口, 组合 BSP + Middleware)
  imu_mpu6050.c/h  IMU ← MPU6050
  disp_oled.c/h    Display ← OLED
  log_uart.c/h     Logger ← UART printf
  cmd_keys.c/h     CommandSource ← GPIO EXTI + 轮询
  cmd_serial.c/h   CommandSource ← UART8 中断
  chassis.c/h      Chassis ← Motor + SpeedCtrl + 16bit编码映射

Middleware/   纯算法 (只依赖 HAL, 不碰硬件)
  speed_ctrl.c/h   速度 PID

App/          应用层 (模式 + 调度)
  main.c            调度器 (依赖注入, 命令轮询, 模式切换)
  mode_imu_test.c   模式: IMU 姿态角 OLED+串口
  mode_chassis_test 模式: 底盘调试 (编码器/电机/PID调参)

Core/         CubeMX 生成, 不动
```

## 设计原则

| 原则 | 实现 |
|------|------|
| SRP 单一职责 | 每个模块一个职责 (传感器/显示/底盘/命令) |
| OCP 开闭 | 加新模式 = 新建 mode_*.c + 注册到 g_modes[] |
| LSP 里氏替换 | 所有模式/命令源/驱动实现相同接口 |
| ISP 接口隔离 | AppMode 回调: on_enter/on_isr/on_ui/on_command |
| DIP 依赖倒置 | main.c 只依赖 HAL 接口, 具体实现在 Adapters 注入 |

## 模式系统

```
KEY3: 切换模式
串口 n: 切换模式

当前模式:
  IMU      — 姿态角 Yaw/Pitch/Roll OLED 显示
  CHASSIS  — 底盘调试: 串口命令控制

加新模式:
  1. App/mode_xxx.c 实现 AppMode 接口
  2. main.c: g_modes[] 数组加一行
  3. 完成, main.c 其他代码不动
```

## CHASSIS 模式串口命令

```
encoder:on/off        编码显示开关
set_speed:L,R         左右设速 (正=前进, 负=后退)
stop:on               惯性滑行 (IN1=0 IN2=0)
brake:on              短接制动 (IN1=1 IN2=1)
kp:85                 在线改 Kp
ki:1.7                在线改 Ki
```

## 硬件引脚

### 电机驱动 TB6612
| 信号 | 引脚 | 说明 |
|------|------|------|
| AIN1 | PE15 | 左电机 IN1 |
| AIN2 | PE12 | 左电机 IN2 |
| LPWMA | PE11 (TIM1_CH2) | 左电机 PWM |
| BIN1 | PE10 | 右电机 IN1 |
| BIN2 | PB2 | 右电机 IN2 |
| RPWMB | PE9 (TIM1_CH1) | 右电机 PWM |

### 编码器
| 信号 | 引脚 | 定时器 |
|------|------|--------|
| LE1A/LE2B | PA0/PA1 | TIM5 (左电机) |
| RE1A/RE2B | PA6/PA7 | TIM3 (右电机) |

### 巡线传感器 (8路红外)
| 通道 | 引脚 | 偏差权重 |
|------|------|----------|
| RED1 | PB13 | +12 |
| RED2 | PB14 | +8 |
| RED3 | PB15 | +3 |
| RED4 | PD8 | +1 |
| RED5 | PD9 | -1 |
| RED6 | PD10 | -3 |
| RED7 | PD11 | -8 |
| RED8 | PA8 | -12 |

### 其他外设
| 外设 | 接口/引脚 | 说明 |
|------|-----------|------|
| MPU6050 | I2C2 | 6轴IMU，卡尔曼滤波 Z轴角度 |
| OLED | I2C1 (0x78) | SSD1306 128×64 |
| 蜂鸣器 | PE3 | 到位提示 |
| LED | PA15 | 状态指示 |
| 按键 | PE4(KEY1), PE5(KEY2), PE6(KEY3), PC13(KEY4) | KEY3/4=EXTI, KEY1/2=轮询 |
| 串口 | UART8 (PE1-TX, PE0-RX) | 115200, printf重定向 + 命令接收 |

## 定时器分配

| 定时器 | 功能 | 配置 |
|--------|------|------|
| TIM1 | 左右电机 PWM | CH1=右轮, CH2=左轮, ARR=999 |
| TIM3 | 右轮编码器 | 编码器模式 |
| TIM5 | 左轮编码器 | 编码器模式 |
| TIM6 | 系统时基 | 1ms中断, 每10ms触发控制循环 |

## 编码值约定

- `encoder_dir = -1`: 原始 CNT 乘 -1 翻转
- `get_encoders` 内部做 16bit 映射: `el<0 → 65535+el`, `el>0 → el`
- 前进: 0→65535 (递增), 后退: 65535→0 (递减)
- 速度 (编码脉冲/10ms): 前进=正, 后退=负

## 编译

- MDK-ARM 编译产物在 `MDK-ARM/MPU6050H7/`, **不提交 git**
- printf 通过 `fputc` 重定向到 UART8, 已设 `setvbuf(stdout, NULL, _IONBF, 0)` 无缓冲
- Keil 需添加 include paths: `..\HAL`, `..\BSP`, `..\Adapters`, `..\Middleware`, `..\App`
