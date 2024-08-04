#ifndef _VOFA_H
#define _VOFA_H

//....在此替换你的串口函数路径........
#include "usart.h"
//...................................
#include <stdio.h>
#include "stdint.h"
#include <string.h>
#include <stdarg.h>

void Vofa_FireWater(const char *format, ...);
void Vofa_JustFloat(float *_data, uint8_t _num);

/*...........示例..............
    float f1=0.5,f2=114.514;
    Vofa_FireWater("%f,%f\r\n", f1, f2);

    float f3[3]={88.77,0.66,55.44};
    Vofa_JustFloat(f3, 3);
*/ 

#endif