#ifndef GET_ADC_H
#define GET_ADC_H
#include "motor.h"
typedef struct 
{
		float Udc; //母线电压采样值
		int16_t Iu_Sample;   // U相电流采样值
		int16_t Iv_Sample;   // V相电流采样值
		int16_t Iw_Sample;   // W相电流采样值
		float Iu_Sample_offer;   // U相电流校正
		float Iv_Sample_offer;
		float Iw_Sample_offer;
		float Iu;    //u相采样电流
		float Iv;  //v相采样电流
		float Iw;  //w相采样电流

	
	
		float adc_refer; //参考0位电压
		// float gain_cur;	//电流增益
		// float gain_udc;  //母线增益
	
		// int16_t adcstateflag;
		// int16_t ADCcount;   //补偿计时器
	
		// float currentoffera;
		// float currentofferb;
		// float currentofferc;
		
		// int16_t currentofferEN;
}ADCGG_t;


typedef struct {
	int a;
}ADC_t;
extern int16_t adc_buff[4];
void adc_init(ADCGG_t* adc);
#endif