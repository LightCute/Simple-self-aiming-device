# TICup2026 巡线小车 — STM32H7 工程笔记

## 项目概况

- **平台**: STM32H723 (实际芯片 STM32H750VBTx)
- **IDE**: MDK-ARM (Keil), 由 STM32CubeMX 生成初始代码
- **用途**: TICup2026 竞赛巡线小车

## 分支说明

| 分支 | 用途 |
|------|------|
| `main` | 主分支 |
| `test1` | 原始完整版：巡线 → T路口检测 → 转弯入库 → 倒车停车（1.8-终版） |
| `feature/line-tracking-only` | 简化版：纯巡线 + T路口停车，去掉入库逻辑 |
| `feature/motor-test` | 电机测试：串口 CSV 输出，测试前进/后退/差速/制动 |

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
| 侧方传感器 | PD6 (SensorL), PB5 (SensorR) | T路口空位检测 |
| 蜂鸣器 | PE3 | 到位提示 |
| LED | PA15 | 状态指示 |
| 按键 | PE4(KEY1), PE5(KEY2), PE6(KEY3), PC13(KEY4) | 用户输入 |
| 串口 | UART8 (PE1-TX, PE0-RX) | 115200, printf重定向 |

## 定时器分配

| 定时器 | 功能 | 配置 |
|--------|------|------|
| TIM1 | 左右电机 PWM | CH1=右轮, CH2=左轮, ARR=999 |
| TIM3 | 右轮编码器 | 编码器模式 |
| TIM5 | 左轮编码器 | 编码器模式 |
| TIM6 | 系统时基 | 1ms中断 |

## 控制架构 (test1 完整版)

三级 PID 级联：
1. **转向环** (外环): 8路传感器偏移量 → PD控制 (Kp=2.12, Kd=0.96)
2. **角度环** (外环): MPU6050 Z轴角度 → PI控制 (Kp=0.11, Ki=0.0047)
3. **速度环** (内环): 编码器反馈 → 左右独立PI (Kp=85, Ki=1.7)

差速公式: `left_target = base_speed - correction`, `right_target = base_speed + correction`

## 电机测试 (feature/motor-test)

- printf 输出 CSV 格式到 UART8 (115200)
- 测试序列: IDLE → FWD_200/400/600/800 → REV_200/400 → TURN_L/R → BRAKE
- 每100ms输出一行: 时间戳, 阶段, 左右目标速度/实际速度/编码器值
- 总时长约18秒，完成后自动刹车

## 编译注意事项

- MDK-ARM 编译产物在 `MDK-ARM/MPU6050H7/` 下，**不要提交到 git**（已存在于历史中）
- `mpu6050.o` 和 `control.o` 有相互依赖：control.c 引用了 `extern MPU6050_t mpu6050`
- printf 通过 `fputc` 重定向到 UART8 (见 `Core/Src/usart.c`)

## 常见问题

1. **电机不转**: 检查 `flag` 是否为 1（需按 KEY3），或 `Control()` 是否在主循环中执行
2. **编译报 Undefined symbol mpu6050**: 确保 main.c 中有 `MPU6050_t mpu6050;` 定义
3. **转弯角度不准**: 调整 `g_angle_tolerance`（默认±8°）和角度环 PID 参数
