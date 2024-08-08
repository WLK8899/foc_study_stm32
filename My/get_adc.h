#ifndef GET_ADC_H
#define GET_ADC_H
#include "main.h"
#include "motor.h"
#include "control.h"
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
		float gain_cur;	//电流增益
		float gain_udc;  //母线增益
	
		int16_t adcstateflag;
		int16_t ADCcount;   //补偿计时器
	
		float currentoffera;
		float currentofferb;
		float currentofferc;
		
		int16_t currentofferEN;
}ADC_t;


void adc_init(ADC_t* adc);
#endif