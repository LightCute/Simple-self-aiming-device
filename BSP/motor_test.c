#include "motor_test.h"
#include "tb6612.h"
#include "stdio.h"

/* ==================== 测试阶段定义 ==================== */
typedef enum {
    PHASE_IDLE = 0,
    PHASE_FWD_200,
    PHASE_FWD_400,
    PHASE_FWD_600,
    PHASE_FWD_800,
    PHASE_REV_200,
    PHASE_REV_400,
    PHASE_TURN_LEFT,
    PHASE_TURN_RIGHT,
    PHASE_BRAKE,
    PHASE_DONE,
    PHASE_COUNT
} TestPhase_t;

typedef struct {
    TestPhase_t phase;
    const char *name;
    int16_t     target_l;
    int16_t     target_r;
    uint32_t    duration_ms;   /* 本阶段持续时间 */
} PhaseDesc_t;

static const PhaseDesc_t g_phases[] = {
    {PHASE_IDLE,       "IDLE",       0,    0,   1000},
    {PHASE_FWD_200,    "FWD_200",  200,  200,   2000},
    {PHASE_FWD_400,    "FWD_400",  400,  400,   2000},
    {PHASE_FWD_600,    "FWD_600",  600,  600,   2000},
    {PHASE_FWD_800,    "FWD_800",  800,  800,   2000},
    {PHASE_REV_200,    "REV_200", -200, -200,   2000},
    {PHASE_REV_400,    "REV_400", -400, -400,   2000},
    {PHASE_TURN_LEFT,  "TURN_L",   200,  600,   2000},
    {PHASE_TURN_RIGHT, "TURN_R",   600,  200,   2000},
    {PHASE_BRAKE,      "BRAKE",      0,    0,   1000},
};

static uint8_t   g_phase_index = 0;
static uint32_t  g_phase_start_tick = 0;
static uint32_t  g_last_print_tick = 0;
static uint8_t   g_started = 0;
static uint8_t   g_done = 0;

/* ==================== 获取系统毫秒时间 ==================== */
static uint32_t GetTick(void)
{
    return HAL_GetTick();
}

/* ==================== 初始化 ==================== */
void MotorTest_Init(void)
{
    g_phase_index = 0;
    g_started = 0;
    g_done = 0;

    /* 打印 CSV 表头 */
    printf("# Motor Test Report\r\n");
    printf("# Time_ms, Phase, Target_L, Actual_L, Enc_L, Target_R, Actual_R, Enc_R\r\n");
}

/* ==================== 运行测试序列 ==================== */
void MotorTest_Run(void)
{
    uint32_t now = GetTick();

    if (g_done)
        return;   /* 测试已完成，不再输出 */

    if (!g_started)
    {
        g_started = 1;
        g_phase_index = 0;
        g_phase_start_tick = now;
        g_last_print_tick = now;

        /* 复位编码器 */
        TB6612_ResetEncoder();
        TB6612_UpdateSpeed();

        printf("0, START, 0, 0, 0, 0, 0, 0\r\n");
    }

    /* 检查当前阶段是否结束 */
    const PhaseDesc_t *p = &g_phases[g_phase_index];
    uint32_t elapsed = now - g_phase_start_tick;

    if (elapsed >= p->duration_ms)
    {
        /* 进入下一阶段 */
        g_phase_index++;
        if (g_phase_index >= (sizeof(g_phases) / sizeof(g_phases[0])))
        {
            /* 所有阶段完成 */
            TB6612_Brake();
            printf("%lu, DONE, 0, 0, 0, 0, 0, 0\r\n", (unsigned long)now);
            g_done = 1;
            return;
        }
        p = &g_phases[g_phase_index];
        g_phase_start_tick = now;
        g_last_print_tick = now;

        /* 设置本阶段电机速度 */
        TB6612_Run(p->target_l, p->target_r);
    }

    /* 每100ms打印一次数据 */
    if (now - g_last_print_tick >= 100)
    {
        g_last_print_tick = now;

        /* 更新编码器速度 */
        TB6612_UpdateSpeed();

        int32_t enc_l, enc_r;
        TB6612_GetEncoder(&enc_l, &enc_r);

        printf("%lu, %s, %d, %ld, %ld, %d, %ld, %ld\r\n",
               (unsigned long)now,
               p->name,
               p->target_l, (long)TB6612_GetLeftSpeed(),  (long)enc_l,
               p->target_r, (long)TB6612_GetRightSpeed(), (long)enc_r);
    }
}
