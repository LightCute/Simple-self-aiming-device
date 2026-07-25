# 圈数计数功能设计

## 需求

小车在正方形巡线图纸上顺时针运动，四个角均为右直角弯。初始位置在某条边中点。通过按键设定目标圈数 1~5，跑完自动停车。

## 一圈定义

正方形 + 顺时针 = 每圈遇到 **4 个右直角弯**。每检测到一个右直角弯（`Tracking_IsSharpTurn()==2`），转弯计数器 +1。满 4 个 = 1 圈。

## 新增变量

| 变量 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `g_lap_target` | `uint8_t` | 3 | 目标圈数 1~5 |
| `g_lap_count` | `uint8_t` | 0 | 已完成圈数 |
| `g_turn_count` | `uint8_t` | 0 | 当前圈内直角弯数 0~3 |
| `g_turn_is_lap` | `uint8_t` | 0 | 本次转弯是否计入圈数 |

全局变量，放在 `control.c`，`control.h` 声明 extern。

## 交互流程

### 待机模式 (mode 0)

```
OLED:
  LAPS: 3              ← KEY1/KEY2 加减 (范围 1~5, 循环)
  KEY4: START
```

按键逻辑加在 while 循环 mode 0 分支。

### 巡线模式 (mode 1)

```
OLED 第一行:
STEER          L2/3   ← 模式名 + 当前圈/目标圈

其余行不变：传感器位、速度、偏移
```

## 状态机修改

全部在 `control.c`：

1. **Control() —— 右直角弯分支加标记**：
   ```c
   else if (sharp == 2) {
       g_turn_is_lap = 1;    // 标记：这个弯计入圈数
       // 原有弧线转弯逻辑不变
   }
   ```

2. **Control_Update() —— 角度环完成处加圈数逻辑**：
   ```c
   if (g_car_state == STATE_TURN) {
       if (g_turn_is_lap) {
           g_turn_count++;
           if (g_turn_count >= 4) {
               g_turn_count = 0;
               g_lap_count++;
               if (g_lap_count >= g_lap_target) {
                   Control_Stop();
                   g_car_state = STATE_STOP;
                   beep_cnt = 300;      // 长鸣 3s
                   g_turn_is_lap = 0;
                   return;
               }
           }
           g_turn_is_lap = 0;
       }
       // 正常恢复巡线
       g_steer_mode = 2;
       g_base_speed = g_speed_init;
       g_car_state = STATE_LINE;
       // ... 清PID
   }
   ```

3. **Control_Start() —— 启动时清零圈数**：
   ```c
   g_lap_count = 0;
   g_turn_count = 0;
   g_turn_is_lap = 0;
   ```

## 涉及文件

| 文件 | 改动 |
|------|------|
| `BSP/control.c` | 新增 4 个全局变量 + 修改 Control/Control_Update/Control_Start |
| `BSP/control.h` | 新增 4 个 extern 声明 |
| `Core/Src/main.c` | mode 0 加圈数设置界面 + mode 1 OLED 加圈数显示 |

## 不涉及

- 电机驱动、传感器、PID 参数均不变
- 左直角弯、T 路口行为不变
- 模式 2/3/4 不变
