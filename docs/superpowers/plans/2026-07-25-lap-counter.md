# 圈数计数功能实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 小车顺时针跑正方形，通过 KEY1/2 设定 1~5 圈，每圈计 4 个右直角弯，圈满自动停车。

**Architecture:** control.c 新增 4 个全局变量管理圈数状态，Control() 标记右直角弯计入圈数，Control_Update() 转弯完成时计数，Control_Start() 启动时清零。main.c 待机模式加圈数设置界面，巡线模式 OLED 显示当前圈数。

**Tech Stack:** C (STM32 HAL), 现有 control/tracking/OLED 框架

## Global Constraints

- 圈数范围 1~5，循环
- 右直角弯 `sharp==2` 计入，左直角弯不计
- 圈满 → STATE_STOP，长鸣 3s，不复位巡线
- KEY4 可手动恢复

---

### Task 1: control.h — 新增变量声明

**Files:**
- Modify: `BSP/control.h:56`

**Interfaces:**
- Produces: `extern uint8_t g_lap_target, g_lap_count, g_turn_count, g_turn_is_lap`

- [ ] 在 `extern CarState_t g_car_state;` 之后插入 4 个 extern 声明：

```c
extern CarState_t g_car_state;

/* 圈数计数 */
extern uint8_t g_lap_target;   /* 目标圈数 1~5 */
extern uint8_t g_lap_count;    /* 已完成圈数 */
extern uint8_t g_turn_count;   /* 当前圈内直角弯数 0~3 */
extern uint8_t g_turn_is_lap;  /* 本次转弯是否计入圈数 */
```

- [ ] Commit

### Task 2: control.c — 新增全局变量定义

**Files:**
- Modify: `BSP/control.c:7-8` (在 `g_arc_ratio` 之后)

**Interfaces:**
- Consumes: (none new)
- Produces: `g_lap_target=3, g_lap_count=0, g_turn_count=0, g_turn_is_lap=0`

- [ ] 在 `float g_arc_ratio = 0.5f;` 之后插入：

```c
/* 圈数计数 */
uint8_t g_lap_target = 3;   /* 目标圈数 1~5 */
uint8_t g_lap_count  = 0;   /* 已完成圈数 */
uint8_t g_turn_count = 0;   /* 当前圈内右直角弯计数 0~3 */
uint8_t g_turn_is_lap = 0;  /* 本次转弯计入圈数标志 */
```

- [ ] Commit

### Task 3: control.c — Control() 右直角弯加标志

**Files:**
- Modify: `BSP/control.c:284-290` (sharp==2 分支)

**Interfaces:**
- Consumes: `g_turn_is_lap`
- Produces: sets `g_turn_is_lap = 1` before right-angle turn

- [ ] 在右直角弯分支中，`g_car_state = STATE_TURN` 之前加一行：

```c
else if (sharp == 2)
{
    /* 右直角弯 → 弧线右转90° */
    g_turn_is_lap = 1;                             /* 标记计入圈数 */
    g_base_speed = -(int16_t)(g_speed_init * g_arc_ratio);
    g_car_state = STATE_TURN;
    Control_SetRelativeAngle(-90.0f);
}
```

- [ ] Commit

### Task 4: control.c — Control_Update() 转弯完成加圈数逻辑

**Files:**
- Modify: `BSP/control.c:167-176` (STATE_TURN 完成分支)

**Interfaces:**
- Consumes: `g_turn_is_lap, g_turn_count, g_lap_count, g_lap_target`
- Produces: increments counters, stops if laps done

- [ ] 将现有 STATE_TURN 完成分支替换为：

```c
if (g_car_state == STATE_TURN)
{
    if (g_turn_is_lap)
    {
        g_turn_count++;
        if (g_turn_count >= 4)
        {
            g_turn_count = 0;
            g_lap_count++;
            if (g_lap_count >= g_lap_target)
            {
                /* 圈数完成 → 停车 */
                Control_Stop();
                g_car_state = STATE_STOP;
                beep_cnt = 300;
                g_turn_is_lap = 0;
                break;  /* 跳出 if, 跳过巡线恢复 */
            }
        }
        g_turn_is_lap = 0;
    }
    /* 转弯完成 → 恢复巡线 */
    g_steer_mode = 2;
    g_base_speed = g_speed_init;
    g_car_state = STATE_LINE;
    g_pid_steer.integral   = 0.0f;
    g_pid_steer.prev_error = 0.0f;
    g_pid_L.integral = 0.0f;
    g_pid_R.integral = 0.0f;
}
```

> `break` 跳出的是最外层 `if (abs_err < 12.0f)` 的判断体，需要确认不会影响外层循环。当前代码中这个 if 块已经位于 `if (g_car_state == STATE_TURN)` 内部的最后，`break` 后直接到达 `if (g_car_state == STATE_TURN)` 的闭合括号 `}`，不会跳过其他关键逻辑。安全替代方案：用 `return` 代替 `break` 提前退出整个 if 块，或者把巡线恢复代码放入 `else` 分支。

**修正为（使用 goto 或拆分）：**

```c
if (g_car_state == STATE_TURN)
{
    uint8_t done = 0;
    if (g_turn_is_lap)
    {
        g_turn_count++;
        if (g_turn_count >= 4)
        {
            g_turn_count = 0;
            g_lap_count++;
            if (g_lap_count >= g_lap_target)
            {
                Control_Stop();
                g_car_state = STATE_STOP;
                beep_cnt = 300;
                g_turn_is_lap = 0;
                done = 1;
            }
        }
        g_turn_is_lap = 0;
    }
    if (!done)
    {
        /* 转弯完成 → 恢复巡线 */
        g_steer_mode = 2;
        g_base_speed = g_speed_init;
        g_car_state = STATE_LINE;
        g_pid_steer.integral   = 0.0f;
        g_pid_steer.prev_error = 0.0f;
        g_pid_L.integral = 0.0f;
        g_pid_R.integral = 0.0f;
    }
}
```

- [ ] Commit

### Task 5: control.c — Control_Start() 清零圈数

**Files:**
- Modify: `BSP/control.c:251-260` (Control_Start 函数体)

**Interfaces:**
- Consumes: `g_lap_count, g_turn_count, g_turn_is_lap`
- Produces: zeros them

- [ ] 在 `Control_Start()` 函数开头加三行清零：

```c
void Control_Start(void)
{
    g_lap_count  = 0;   /* 清零圈数 */
    g_turn_count = 0;
    g_turn_is_lap = 0;

    g_steer_mode = 2;
    g_base_speed = g_speed_init;
    g_pid_steer.integral   = 0.0f;
    g_pid_steer.prev_error = 0.0f;
    g_pid_L.integral       = 0.0f;
    g_pid_R.integral       = 0.0f;
}
```

- [ ] Commit

### Task 6: main.c — 待机模式 (case 0) 加圈数设置

**Files:**
- Modify: `Core/Src/main.c:199-203` (case 0 分支)

**Interfaces:**
- Consumes: `g_lap_target` (extern from control.h)
- Produces: KEY1/2 adjust `g_lap_target` 1~5, KEY4 starts

- [ ] 将现有 case 0 替换为：

```c
case 0:
    {
        /* KEY1/KEY2 设圈数 */
        static uint8_t k1d = 0, k2d = 0;
        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
        {
            if (k1d < 5) k1d++;
            if (k1d == 5) { if (g_lap_target > 1) g_lap_target--; else g_lap_target = 5; }
        }
        else { k1d = 0; }
        if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
        {
            if (k2d < 5) k2d++;
            if (k2d == 5) { if (g_lap_target < 5) g_lap_target++; else g_lap_target = 1; }
        }
        else { k2d = 0; }
    }
    sprintf(msg, "LAPS: %d", g_lap_target);
    OLED_PrintASCIIString(20, 20, msg, &afont12x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(15, 35, "KEY4: START", &afont12x6, OLED_COLOR_NORMAL);
    break;
```

- [ ] Commit

### Task 7: main.c — 巡线模式 (case 1) OLED 加圈数显示

**Files:**
- Modify: `Core/Src/main.c:206-228` (case 1 分支第一行)

**Interfaces:**
- Consumes: `g_lap_count, g_lap_target` (extern from control.h)

- [ ] 将 mode_name 那行显示改为包含圈数：

在 case 1 分支最前面（OLED_PrintASCIIString 那行）改为：

```c
case 1:
    {
        const char *st = (g_car_state == STATE_LINE) ? "LINE" : (g_car_state == STATE_TURN) ? "TURN" : "STOP";
        sprintf(msg, "%s L%d/%d", st, g_lap_count + 1, g_lap_target);
        OLED_PrintASCIIString(20, 0, msg, &afont12x6, OLED_COLOR_NORMAL);
        // 传感器显示不变
        ...
    }
```

> 注：`g_lap_count + 1` 是因为 `g_lap_count` 是已完成圈数，当前在跑的是第 `g_lap_count + 1` 圈。

- [ ] Commit

### Task 8: 验证 — 全链路审查

- [ ] `grep -rn "g_lap_target\|g_lap_count\|g_turn_count\|g_turn_is_lap" BSP/ Core/` 确认所有引用正确
- [ ] 检查 `Control_Start()` 在 KEY4 模式1 中被调用 → 自动清零
- [ ] 检查圈满后 `STATE_STOP` + 长鸣 → KEY4 可恢复巡线（重置圈数）
- [ ] Commit
