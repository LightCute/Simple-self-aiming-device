#include "control.h"
#include "tb6612.h"
#include "mpu6050.h"
#include "tracking.h"
#include "stdio.h"

#define PWM_MAX  999
#define Z_DZ_MIN 0.6f      /* Z轴转弯检测: 单帧最小变化(度), 10ms周期 */
#define Z_ACCUM_MIN 60.0f  /* Z轴转弯检测: 累计最小变化(度) */
int16_t g_speed_init = 20;
float   g_arc_ratio  = 0.5f;

/* 圈数计数 */
uint8_t g_lap_target = 3;
uint8_t g_lap_count  = 0;
uint8_t g_turn_count = 0;
uint8_t g_turn_is_lap = 0;

extern MPU6050_t mpu6050;

/* ========== 速度环参数 ========== */
float g_speed_Kp_L = 85.0f, g_speed_Ki_L = 1.7f, g_speed_Kd_L = 0.0f;
float g_speed_Kp_R = 85.0f, g_speed_Ki_R = 1.7f, g_speed_Kd_R = 0.0f;

int16_t g_target_left  = 0;
int16_t g_target_right = 0;
int16_t g_actual_left  = 0;
int16_t g_actual_right = 0;
int16_t g_pwm_left     = 0;
int16_t g_pwm_right    = 0;

/* ========== 角度环参数 ========== */
float  g_angle_Kp    = 1.0f;
float  g_angle_Ki    = 0.05f;
float  g_angle_Kd    = 5.0f;
float  g_target_angle    = 0.0f;
float  g_angle_error     = 0.0f;
float  g_angle_tolerance = 8.0f;
static float g_angle_ref = 0.0f;

/* ========== 转向环参数 ========== */
float g_steer_Kp = 1.12f;
float g_steer_Ki = 0.0f;
float g_steer_Kd = 0.96f;

uint8_t g_steer_mode = 2;
int16_t g_base_speed = 10;
CarState_t g_car_state = STATE_LINE;

/* 速度PID */
static PID_t g_pid_L;
static PID_t g_pid_R;

/* 角度PID */
static PID_t g_pid_angle;

/* 转向PID */
static PID_t g_pid_steer;

/* 蜂鸣器 */
static uint16_t beep_cnt = 0;
#define BEEP_ON()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#define BEEP_OFF()  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)

/* ==================== 通用工具 ==================== */

static float PID_Calc(PID_t *pid, float error)
{
    float p, i, d;

    p = pid->Kp * error;

    pid->integral += error;
    if (pid->integral >  pid->integral_max)  pid->integral =  pid->integral_max;
    if (pid->integral < -pid->integral_max)  pid->integral = -pid->integral_max;
    i = pid->Ki * pid->integral;

    d = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    p += i + d;
    if (p >  pid->output_max)  p =  pid->output_max;
    if (p < -pid->output_max)  p = -pid->output_max;

    return p;
}

static float Angle_Error(float target, float current)
{
    float err = target - current;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* ==================== 内环: 速度PID ==================== */

static void Inner_Speed(void)
{
    g_pid_L.Kp = g_speed_Kp_L;  g_pid_L.Ki = g_speed_Ki_L;  g_pid_L.Kd = g_speed_Kd_L;
    g_pid_R.Kp = g_speed_Kp_R;  g_pid_R.Ki = g_speed_Ki_R;  g_pid_R.Kd = g_speed_Kd_R;

    g_actual_left  = TB6612_GetLeftSpeed();
    g_actual_right = TB6612_GetRightSpeed();

    g_pwm_left  = (int16_t)PID_Calc(&g_pid_L, (float)(g_target_left  - g_actual_left));
    g_pwm_right = (int16_t)PID_Calc(&g_pid_R, (float)(g_target_right - g_actual_right));
}

static void Inner_DiffSpeed(float correction)
{
    int16_t left_target, right_target;

    if (g_steer_mode == 1)
    {
        left_target  = g_base_speed + (int16_t)correction;
        right_target = g_base_speed - (int16_t)correction;
    }
    else
    {
        left_target  = g_base_speed - (int16_t)correction;
        right_target = g_base_speed + (int16_t)correction;
    }

    if (left_target  < -PWM_MAX) left_target  = -PWM_MAX;
    if (left_target  >  PWM_MAX) left_target  =  PWM_MAX;
    if (right_target < -PWM_MAX) right_target = -PWM_MAX;
    if (right_target >  PWM_MAX) right_target =  PWM_MAX;

    g_target_left  = left_target;
    g_target_right = right_target;
}

/* ==================== 外环: 角度PID ==================== */

static float Outer_Angle(void)
{
    float abs_target = g_angle_ref + g_target_angle;
    float angle_err = Angle_Error(abs_target, mpu6050.KalmanAngleZ);
    float abs_err   = (angle_err > 0) ? angle_err : -angle_err;
    g_angle_error   = angle_err;

    g_pid_angle.Kp = g_angle_Kp;
    g_pid_angle.Ki = g_angle_Ki;
    g_pid_angle.Kd = g_angle_Kd;

    if (abs_err > 15.0f)
        g_pid_angle.output_max = 700.0f;
    else
        g_pid_angle.output_max = 350.0f;

    float correction = PID_Calc(&g_pid_angle, angle_err);

    /* 每500ms输出一次角度环状态 */
    {
        static uint8_t angle_log_cnt = 0;
        if (++angle_log_cnt >= 50) { angle_log_cnt = 0;
            printf("[ANGLE] err=%.1f corr=%.1f Z=%.1f ref=%.1f target=%.1f max=%.0f\r\n",
                   angle_err, correction, mpu6050.KalmanAngleZ,
                   g_angle_ref, g_target_angle, g_pid_angle.output_max);
        }
    }

    /* 到位退出 */
    if (abs_err < 12.0f)
    {
        printf("[ANGLE-DONE] err=%.1f Z=%.1f -> exit\r\n",
               angle_err, mpu6050.KalmanAngleZ);
        g_pid_angle.integral = 0.0f;
        correction = 0.0f;
        beep_cnt = 50;

        if (g_car_state == STATE_TURN)
        {
            uint8_t done = 0;
            if (g_turn_is_lap)
            {
                g_turn_count++;
                printf("[TURN] Complete, lap=%d/%d turn=%d/4\r\n",
                       g_lap_count + 1, g_lap_target, g_turn_count);
                if (g_turn_count >= 4)
                {
                    g_turn_count = 0;
                    g_lap_count++;
                    printf("[LAP] Lap %d/%d completed!\r\n",
                           g_lap_count, g_lap_target);
                    if (g_lap_count >= g_lap_target)
                    {
                        printf("[DONE] All %d laps done!\r\n", g_lap_target);
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
                g_steer_mode = 2;
                g_base_speed = g_speed_init;
                g_car_state = STATE_LINE;
                g_pid_steer.integral   = 0.0f;
                g_pid_steer.prev_error = 0.0f;
                g_pid_L.integral = 0.0f;
                g_pid_R.integral = 0.0f;
            }
        }
        else
        {
            g_steer_mode = 0;
        }
    }

    return correction;
}

/* ==================== 外环: 转向PID ==================== */

static float Outer_Steering(void)
{
    Tracking_UpdateOffset();
    g_pid_steer.Kp = g_steer_Kp;
    g_pid_steer.Ki = g_steer_Ki;
    g_pid_steer.Kd = g_steer_Kd;
    g_pid_steer.output_max = (float)PWM_MAX;
    return PID_Calc(&g_pid_steer, (float)g_tracking_offset);
}

/* ==================== 检测层: Z轴转弯监测(仅日志, 不计圈) ==================== */

static float   g_prev_z   = 0.0f;
static float   g_z_accum  = 0.0f;
static uint8_t g_z_stable = 0;

static void Detect_ZTurn(void)
{
    float cur_z = mpu6050.KalmanAngleZ;
    float dz    = (cur_z > g_prev_z) ? (cur_z - g_prev_z) : (g_prev_z - cur_z);

    if (dz > Z_DZ_MIN)
    {
        g_z_accum += dz;
        g_z_stable = 0;
    }
    else
    {
        if (g_z_stable < 10) g_z_stable++;
        if (g_z_stable == 10 && g_z_accum > Z_ACCUM_MIN)
        {
            printf("[Z-TURN] accum=%.0f Z=%.1f (ref only)\r\n",
                   g_z_accum, cur_z);
            g_z_accum = 0.0f;
        }
    }
    g_prev_z = cur_z;
}

static void Detect_Reset(void)
{
    g_prev_z   = mpu6050.KalmanAngleZ;
    g_z_accum  = 0.0f;
    g_z_stable = 0;
}

/* ==================== 蜂鸣器 ==================== */

static void Beep_Update(void)
{
    if (beep_cnt > 0) { BEEP_ON();  beep_cnt--; }
    else              { BEEP_OFF(); }
}

/* ==================== 主调度: Control_Update ==================== */

void Control_Update(void)
{
    float correction = 0.0f;

    /* 选外环 */
    if (g_steer_mode == 1)
        correction = Outer_Angle();
    else if (g_steer_mode == 2)
    {
        correction = Outer_Steering();
        Detect_ZTurn();
        if (g_car_state == STATE_STOP) return;  /* 圈满停车后不再输出电机 */
    }

    /* 差速分配 */
    Inner_DiffSpeed(correction);

    /* 内环速度PID */
    Inner_Speed();

    /* 角度环模式输出PWM日志 */
    if (g_steer_mode == 1)
    {
        static uint8_t pwm_log = 0;
        if (++pwm_log >= 50) { pwm_log = 0;
            printf("[PWM] L:%d R:%d TgtL:%d TgtR:%d ActL:%d ActR:%d base:%d corr:%.1f\r\n",
                   g_pwm_left, g_pwm_right,
                   g_target_left, g_target_right,
                   g_actual_left, g_actual_right,
                   (int)g_base_speed, correction);
        }
    }

    /* 蜂鸣器 */
    Beep_Update();

    /* 输出到电机 */
    TB6612_Run(g_pwm_left, g_pwm_right);
}

/* ==================== 公共接口 ==================== */

void Control_Init(void)
{
    g_pid_L.Kp = g_speed_Kp_L;  g_pid_L.Ki = g_speed_Ki_L;  g_pid_L.Kd = g_speed_Kd_L;
    g_pid_L.integral = 0.0f;  g_pid_L.prev_error = 0.0f;
    g_pid_L.integral_max = 500.0f;  g_pid_L.output_max = (float)PWM_MAX;

    g_pid_R.Kp = g_speed_Kp_R;  g_pid_R.Ki = g_speed_Ki_R;  g_pid_R.Kd = g_speed_Kd_R;
    g_pid_R.integral = 0.0f;  g_pid_R.prev_error = 0.0f;
    g_pid_R.integral_max = 500.0f;  g_pid_R.output_max = (float)PWM_MAX;

    g_pid_angle.Kp = g_angle_Kp;  g_pid_angle.Ki = g_angle_Ki;  g_pid_angle.Kd = g_angle_Kd;
    g_pid_angle.integral = 0.0f;  g_pid_angle.prev_error = 0.0f;
    g_pid_angle.integral_max = 200.0f;  g_pid_angle.output_max = 400.0f;

    g_pid_steer.Kp = g_steer_Kp;  g_pid_steer.Ki = g_steer_Ki;  g_pid_steer.Kd = g_steer_Kd;
    g_pid_steer.integral = 0.0f;  g_pid_steer.prev_error = 0.0f;
    g_pid_steer.integral_max = 200.0f;  g_pid_steer.output_max = (float)g_base_speed;
}

void Control_SetRelativeAngle(float delta)
{
    g_angle_ref = mpu6050.KalmanAngleZ;
    g_target_angle = delta;
    g_pid_angle.integral   = 0.0f;
    g_pid_angle.prev_error = 0.0f;
    g_steer_mode = 1;
}

void Control_Stop(void)
{
    g_steer_mode = 0;
    g_base_speed = 0;
    g_target_left  = 0;
    g_target_right = 0;
    TB6612_Brake();
}

void Control_Start(void)
{
    g_lap_count  = 0;
    g_turn_count = 0;
    g_turn_is_lap = 0;
    Detect_Reset();

    g_steer_mode = 2;
    g_base_speed = g_speed_init;
    g_pid_steer.integral   = 0.0f;
    g_pid_steer.prev_error = 0.0f;
    g_pid_L.integral = 0.0f;
    g_pid_R.integral = 0.0f;
}

/* ==================== 控制总逻辑 ==================== */

void Control(void)
{
    if (g_car_state == STATE_LINE)
    {
        uint8_t sharp = Tracking_IsSharpTurn();

        if (sharp == 1)
        {
            g_base_speed = -(int16_t)(g_speed_init * g_arc_ratio);
            g_car_state = STATE_TURN;
            Control_SetRelativeAngle(90.0f);
        }
        else if (sharp == 2)
        {
            g_turn_is_lap = 1;
            g_base_speed = -(int16_t)(g_speed_init * g_arc_ratio);
            g_car_state = STATE_TURN;
            Control_SetRelativeAngle(-90.0f);
        }
        else
        {
            TB6612_UpdateSpeed();
            Control_Update();
        }
    }
    else if (g_car_state == STATE_TURN)
    {
        TB6612_UpdateSpeed();
        Control_Update();
    }
}
