#include "turn_ctrl.h"
#include "HAL/imu.h"
#include "HAL/chassis.h"
#include "Middleware/angle_pid.h"

static AnglePID  g_angle_pid;
static float     g_ref_angle = 0;    /* 转弯起始参考Z角 */
static float     g_target     = 0;   /* 相对目标角度 */
static int16_t   g_base_spd   = 0;   /* 弧线基速, 0=原地转 */
static TurnState g_state      = TURN_IDLE;

extern IMU     *g_imu;
extern Chassis *g_chassis;

static void tc_init(void *self) {
    (void)self;
    AnglePID_Init(&g_angle_pid);
}

/* 计算角度误差 (处理±180°环绕) */
static float angle_err(float target, float current) {
    float e = target - current;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

static void tc_spot(void *self, float delta) {
    (void)self;
    g_ref_angle = g_imu->yaw;
    g_target    = delta;
    g_base_spd  = 0;              /* 基速=0 = 原地转 */
    AnglePID_Init(&g_angle_pid);
    g_state = TURN_RUNNING;
}

static void tc_arc(void *self, float delta, int16_t base_spd) {
    (void)self;
    g_ref_angle = g_imu->yaw;
    g_target    = delta;
    g_base_spd  = base_spd;       /* 弧线基速 */
    AnglePID_Init(&g_angle_pid);
    g_state = TURN_RUNNING;
}

static void tc_update(void *self) {
    (void)self;
    if (g_state != TURN_RUNNING) return;

    float abs_target = g_ref_angle + g_target;
    float err = angle_err(abs_target, g_imu->yaw);
    float corr = AnglePID_Update(&g_angle_pid, err);

    g_turn_ctrl_inst.correction = corr;
    g_turn_ctrl_inst.angle_err  = err;

    /* 到位判定 */
    if (g_angle_pid.done) {
        g_state = TURN_DONE;
        g_chassis->stop();
        return;
    }

    /* 驱动底盘: 原地转 base=0, 弧线转 base≠0 */
    g_chassis->set_speeds(
        (int16_t)(g_base_spd - corr),
        (int16_t)(g_base_spd + corr));
}

TurnCtrl g_turn_ctrl_inst = {
    .correction = 0, .angle_err = 0, .state = TURN_IDLE,
    .spot   = tc_spot,
    .arc    = tc_arc,
    .update = tc_update,
    .init   = tc_init,
};
