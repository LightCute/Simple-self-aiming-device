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
 * 直角弯检测: 半边全黑半边全白
 * 返回: 0=无直角弯, 1=左直角弯(左边4路黑), 2=右直角弯(右边4路黑)
 */
uint8_t Tracking_IsSharpTurn(void)
{
    uint8_t raw = Tracking_GetRaw();
    uint8_t left4  = raw & 0x0F;
    uint8_t right4 = raw & 0xF0;

    uint8_t result = 0;
    if (left4 == 0x0F && right4 == 0x00)
        result = 1;
    else if (right4 == 0xF0 && left4 == 0x00)
        result = 2;

    if (result != 0)
    {
        printf("[DETECT] TURN! Raw=0x%02X L4=0x%02X R4=0x%02X -> %d\r\n",
               raw, left4, right4, result);
    }

    return result;
}
