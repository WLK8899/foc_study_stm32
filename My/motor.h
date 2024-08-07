#ifndef MOTOR_H
#define MOTOR_H


#define RELOADVALUE_1 10000
#include "AS5047P.h"
typedef struct 
{
    int dir;
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速

    uint8_t direct;      //旋转方向
    
    AS5047_t encoder;
} Motor_t;

extern Motor_t my_motor;

void motor_init(Motor_t* motor);


#endif