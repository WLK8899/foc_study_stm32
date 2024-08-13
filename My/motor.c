#include "motor.h"
#include "tim.h"
#include <math.h>
Motor_t my_motor;

float Tpwm = 1;

void motor_init(Motor_t *motor)
{
    motor->dir = -1;
    motor->encoder = as5047_encoder;
    motor->speed = 0.0;
    motor->lastAngle = 0.0;
    // motor->totalCount = 0;
    motor->overflowNum = 0;
    motor->direct = 0;

    motor->foc.u_d=0;
    motor->foc.u_q=2;
    motor->foc.theta=0;

    // 默认关闭读取速度
    HAL_TIM_Base_Stop_IT(&htim6);
    // 开启编码器模式
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    // 这个存在没有必要性，后续需要上电标零（标零使用两种方法）
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

void Set_SVPWM_Compare(int16_t c1,int16_t c2,int16_t c3)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,c1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,c2);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,c3);
}

