#include "tracking.h"
#include "main.h"
#include "stdio.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} Tracking_Channel_t;

static const Tracking_Channel_t g_tracking_ch[TRACKING_CH_NUM] = {
    {RED1_GPIO_Port, RED1_Pin},
    {RED2_GPIO_Port, RED2_Pin},
    {RED3_GPIO_Port, RED3_Pin},
    {RED4_GPIO_Port, RED4_Pin},
    {RED5_GPIO_Port, RED5_Pin},
    {RED6_GPIO_Port, RED6_Pin},
    {RED7_GPIO_Port, RED7_Pin},
    {RED8_GPIO_Port, RED8_Pin},
};

/* 偏差量数组 */
const int8_t g_tracking_deviation[TRACKING_CH_NUM] = {12, 8, 3, 1, -1, -3, -8, -12};

/* 偏移量 */
int32_t g_tracking_offset = 0;
uint8_t g_tjunction       = 0;   /* T形路口标志 */

/*
 * 检测T形路口：8路传感器全部在黑线上
 */
uint8_t Tracking_IsTJunction(void)
{
    for (uint8_t i = 0; i < TRACKING_CH_NUM; i++)
    {
        if (HAL_GPIO_ReadPin(g_tracking_ch[i].port, g_tracking_ch[i].pin) != GPIO_PIN_SET)
            return 0;
    }
    return 1;
}

/*
 * 扫描8路寻迹灯，将每路电平值乘上对应偏差量后累加为偏移量
 */
void Tracking_UpdateOffset(void)
{
    int32_t sum = 0;
    for (uint8_t i = 0; i < TRACKING_CH_NUM; i++)
    {
        uint8_t level = (HAL_GPIO_ReadPin(g_tracking_ch[i].port, g_tracking_ch[i].pin) == GPIO_PIN_SET) ? 1 : 0;
        sum += (int32_t)level * g_tracking_deviation[i];
    }
    g_tracking_offset = sum;
}

/*
 * 返回8路传感器原始状态: bit0=RED1(左) .. bit7=RED8(右), 1=黑线
 */
uint8_t Tracking_GetRaw(void)
{
    uint8_t raw = 0;
    for (uint8_t i = 0; i < TRACKING_CH_NUM; i++)
    {
        if (HAL_GPIO_ReadPin(g_tracking_ch[i].port, g_tracking_ch[i].pin) == GPIO_PIN_SET)
            raw |= (1 << i);
    }
    return raw;
}

/*
 * 直角弯检测: 边缘判定法
 * 右弯(顺时针): 6,7,8全黑 AND 1必白
 * 左弯(逆时针): 1,2,3全黑 AND 8必白
 * 返回: 0=无, 1=左直角弯, 2=右直角弯
 */
uint8_t Tracking_IsSharpTurn(void)
{
    uint8_t raw  = Tracking_GetRaw();
    uint8_t result = 0;

    /*
     * 顺时针右弯: 线向右拐, 车前行时线偏到传感器左侧
     * → 左侧传感器(1,2,3)见黑, 右端传感器(8)必白
     */
    if ((raw & 0x07) == 0x07 && (raw & 0x80) == 0x00)
        result = 2;

    /*
     * 逆时针左弯: 线向左拐, 车前行时线偏到传感器右侧
     * → 右侧传感器(6,7,8)见黑, 左端传感器(1)必白
     */
    else if ((raw & 0xE0) == 0xE0 && (raw & 0x01) == 0x00)
        result = 1;

    /* printf("[DETECT] ...");  -- 关闭 */

    return result;
}
