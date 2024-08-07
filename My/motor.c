#include "motor.h"
#include "tim.h"

Motor_t my_motor;

void motor_init(Motor_t *motor)
{
    motor->dir = -1;
    motor->encoder = as5047_encoder;
    motor->speed = 0.0;
    motor->lastCount = 0;
    motor->totalCount = 0;
    motor->overflowNum = 0;
    motor->direct = 0;
     
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 10000); // 编码器定时器初始值设定为10000
}
