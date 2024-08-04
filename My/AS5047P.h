#ifndef __AS5047_H
#define __AS5047_H

#include "spi.h" // 根据你使用的STM32系列更改此头文件
#include "main.h"
#include "vofa.h"
/*内容随外界变化的寄存器 可读不可写*/
#define NOP_ADDR      0x0000 //启动读取过程寄存器地址
#define ERRFL_ADDR    0x0001 //错误寄存器地址
#define PROG_ADDR     0x0003 //编程寄存器地址
#define DIAAGC_ADDR   0x3FFC //诊断和AGC寄存器地址
#define MAG_ADDR      0x3FFD //CORDIC寄存器地址
#define ANGLEUNC_ADDR 0x3FFE //无动态角度误差补偿的测量角度寄存器地址
#define ANGLECOM_ADDR 0x3FFF //带动态角度误差补偿的测量角度寄存器地址
 
/*配置选项寄存器 可读可写*/
#define ZPOSM         0x0016
#define ZPOSL         0x0017
#define SETTINGS1     0x0018
#define SETTINGS2     0x0019



typedef struct 
{
    SPI_HandleTypeDef *hspin;
	uint16_t error;

	GPIO_TypeDef *CSNport;
	uint16_t CSNpin;

	GPIO_TypeDef *A_port;
	uint16_t A_pin;
	GPIO_TypeDef *B_port;
	uint16_t B_pin;
	GPIO_TypeDef *Z_port;
	uint16_t Z_pin;
	
	uint16_t angle;




}AS5047_t;

extern AS5047_t as5047_encoder;

// 函数声明
void As5047p_Init();
unsigned int even_check(unsigned int v);
uint16_t send_and_receive_16(uint16_t data);
uint16_t Read_From_AS5047P(uint16_t cmd);
uint16_t Command(uint16_t command, int i);



#endif // __AS5047_H
