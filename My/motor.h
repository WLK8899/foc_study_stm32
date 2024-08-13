#ifndef MOTOR_H
#define MOTOR_H

#include "AS5047P.h"
#include "foc.h"
#include "current .h"
#define RELOADVALUE_1 10000

extern int Udc;
extern float Tpwm;

typedef struct 

{   
    int dir;
    float lastAngle;   //上一次计数值
    // int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速  每转4000个脉冲

    uint8_t direct;      //旋转方向
    
    AS5047_t encoder;
    FOC_t foc;
    ADC_cur_t adc_u;

} Motor_t;

extern Motor_t my_motor;

void motor_init(Motor_t* motor);
void Set_SVPWM_Compare(int16_t c1,int16_t c2,int16_t c3);
#endif