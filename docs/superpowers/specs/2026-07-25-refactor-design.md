# 项目重构设计

## 目标

将混乱的 control.c 拆分为三个独立控制模块，遵守 SOLID 原则，C 语言 struct 封装。保留 BSP 层驱动不变。

## 架构

```
BSP/ (保留不变)
  tb6612.c/h    — 电机驱动 + 编码器
  mpu6050.c/h   — IMU 读取 (KalmanAngleZ)
  oled.c/h      — OLED 显示
  tracking.c/h  — 8路传感器只读

Core/ (新增控制模块)
  speed_ctrl.h/c     — 速度环 (单职责: 让两轮跑到目标速度)
  angle_turn.h/c     — 角度转弯 (单职责: 给定角度, 转到位)
  line_track.h/c     — 巡线 (单职责: 传感器→偏移→返回事件)
  main.c             — 仅做模式调度 + 按键 + OLED
```

## 接口契约

### SpeedCtrl — 速度闭环

```c
void SpeedCtrl_Init(SpeedCtrl *s);
void SpeedCtrl_SetTargets(SpeedCtrl *s, int16_t left, int16_t right);
void SpeedCtrl_Update(SpeedCtrl *s);
// 只读输出: s->actual_l/r, s->pwm_l/r
// 内部直接调 TB6612_Run, 不暴露给上层
```

### AngleTurn — 角度转弯

```c
void  AngleTurn_Init(AngleTurn *a);
void  AngleTurn_Start(AngleTurn *a, float delta_deg); // +90=左转
AngleState AngleTurn_Update(AngleTurn *a);
// 输出: a->correction (差速修正值, 供 SpeedCtrl 使用)
```

### LineTrack — 巡线

```c
void       LineTrack_Init(LineTrack *t);
TrackEvent LineTrack_Update(LineTrack *t);
// 输出: t->correction (差速修正值, 供 SpeedCtrl 使用)
// 返回: TRACK_OK / TRACK_LEFT / TRACK_RIGHT / TRACK_TJUNC
```

## 调度 (main.c)

```
TIM6 ISR (每10ms):
  读 MPU6050
  switch(模式):
    MODE_SPEED:  SpeedCtrl_Update
    MODE_ANGLE:  correction = AngleTurn_Update
                 SpeedCtrl_SetTargets(base ± correction)
                 SpeedCtrl_Update
    MODE_TRACK:  TrackEvent = LineTrack_Update
                 if TRACK_OK:  SpeedCtrl_SetTargets(base ± correction)
                 if TRACK_LEFT/RIGHT: AngleTurn_Start(±90)
                 SpeedCtrl_Update

while(1):
  OLED 显示
  按键处理 (KEY3 切换模式, KEY4 启动/触发, KEY1/2 调参)
```

## SOLID 对照

| 原则 | 实现 |
|------|------|
| SRP 单一职责 | 三个模块各管一件事，main 只管调度 |
| OCP 开闭原则 | 加新功能 = 加新模块，不改已有模块 |
| LSP 里氏替换 | 每个模块通过 `correction` 接口互换 |
| ISP 接口隔离 | SpeedCtrl 不感知谁调用了它 |
| DIP 依赖倒置 | 上层依赖接口(correction)，不依赖具体实现 |

## 模式按键

- Mode 0: SpeedCtrl 调试 (KEY1/2 调速, KEY4 启动)
- Mode 1: AngleTurn 调试 (KEY1/2 调角度, KEY4 触发)
- Mode 2: LineTrack 巡线 (KEY1/2 圈数设定, KEY4 启动)
- KEY3: 循环切换模式
