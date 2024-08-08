#include "motor.h"
#include "tim.h"

Motor_t my_motor;

void motor_init(Motor_t *motor)
{
    motor->dir = -1;
    motor->encoder = as5047_encoder;
    motor->speed = 0.0;
    motor->lastAngle = 0.0;
   // motor->totalCount = 0;
    motor->overflowNum = 0;
    motor->direct = 0;
     
    //默认关闭读取速度
    HAL_TIM_Base_Stop_IT(&htim6);
    //开启编码器模式
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    //这个存在没有必要性，后续需要上电标零（标零使用两种方法）
    __HAL_TIM_SET_COUNTER(&htim3, 0); 
}
