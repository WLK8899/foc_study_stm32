#ifndef CURRENT_H
#define CURRENT_H

#include "main.h"


#define ADC_CONV_FACTOR (3.3f / 4096.0f)
#define CURRENT_CONV_FACTOR (100.0f / 16.5f)
typedef struct
{
    uint8_t adc_stand;

    float Udc;             // 母线电压采样值
    int16_t Iu_Sample;     // U相电流采样值
    int16_t Iv_Sample;     // V相电流采样值
    int16_t Iw_Sample;     // W相电流采样值
    float Iu_Sample_offer; // U相电流校正
    float Iv_Sample_offer;
    float Iw_Sample_offer;
    float Iu_sum_offer; // U相电流校正
    float Iv_sum_offer;
    float Iw_sum_offer;

    int16_t cnt;


    float Iu; // u相采样电流
    float Iv; // v相采样电流
    float Iw; // w相采样电流

    float adc_refer; // 参考0位电压
    // float gain_cur;	//电流增益
    // float gain_udc;  //母线增益

    // int16_t adcstateflag;
    // int16_t ADCcount;   //补偿计时器

    // float currentoffera;
    // float currentofferb;
    // float currentofferc;

    // int16_t currentofferEN;
} ADC_cur_t;
extern int16_t adc_buff[4];
void adc_init(ADC_cur_t *adc);

#endif