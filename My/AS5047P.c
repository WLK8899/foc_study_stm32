#include "AS5047P.h"
#include <stdio.h>
AS5047_t as5047_encoder;

void As5047p_Init(AS5047_t* encoder)
{
    encoder->hspin = &hspi1;
    encoder->CSNport = SPI_CSS_GPIO_Port;
    encoder->CSNpin = SPI_CSS_Pin;

    encoder->A_port = Encode_A_GPIO_Port;
    encoder->A_pin = Encode_A_Pin;
    encoder->B_port = Encoder_B_GPIO_Port;
    encoder->B_pin = Encoder_B_Pin;
    encoder->Z_port = Encoder_Num_GPIO_Port;
    encoder->Z_pin = Encoder_Num_Pin;

    encoder->angle = 0;
    encoder->error = 0;
    encoder->number = 0;
   // encoder->dir = -1;
}

// 计算奇偶函数
unsigned int even_check(unsigned int v)
{
	if (v == 0)
		return 0;
	v ^= v >> 8;
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1;
}
// 命令初始化
uint16_t Command(uint16_t command, int i)
{
	uint16_t cmd = command;
	if (i)
	{
		cmd |= 0x4000;
		if (even_check(cmd) == 1)
			cmd = cmd | 0x8000;
		return cmd;
	}
	else
	{
		// 写数据，暂时用不到
		return 0;
	}
}
// 发送和接收 16 位数据
uint16_t send_and_receive_16(uint16_t data)
{
	uint16_t txData = data;
	uint16_t rxData = 0;

	// CS 低电平使能 SPI 传输
	HAL_GPIO_WritePin(as5047_encoder.CSNport, as5047_encoder.CSNpin, GPIO_PIN_RESET);

	// 发送和接收数据
	if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&txData, (uint8_t *)&rxData, 1, HAL_MAX_DELAY) != HAL_OK)
	{
		printf("spi1_read_error\n");
		HAL_GPIO_WritePin(as5047_encoder.CSNport, as5047_encoder.CSNpin, GPIO_PIN_SET);
		return 0; // 或其他错误标志
	}

	// CS 高电平结束 SPI 传输
	HAL_GPIO_WritePin(as5047_encoder.CSNport, as5047_encoder.CSNpin, GPIO_PIN_SET);
	return rxData;
}
// 读取AS5047角度数据
uint16_t Read_From_AS5047P(uint16_t cmd)
{
	uint16_t command;
	// clear err
	command = Command(ERRFL_ADDR, 1);
	(void)send_and_receive_16(command);

	unsigned int data = 0;
	command = Command(cmd, 1);
	(void)send_and_receive_16(command); // 发送指令

	command = Command(NOP_ADDR, 1);
	data = send_and_receive_16(NOP_ADDR); // 返回的是上一次命令返回的数据 也就是获取的数据

	// 3.检查错误及校验
	if (data & (1 << 14))
	{
		// 有错误 发送清除错误指令
		(void)send_and_receive_16(ERRFL_ADDR);
		// 获取错误原因
		as5047_encoder.error = send_and_receive_16(NOP_ADDR);
		as5047_encoder.error = as5047_encoder.error & 0x0003;
		printf("as5047_encoder.error: %d\n", as5047_encoder.error);
		return 0;
	}
	else // 无错误
	{
		// 偶校验最高位 通过则读取数据
		if ((data >> 15) == even_check(data & 0x7FFF))
		{
			return (data & 0x3FFF);
		}
		else
		{
			return 0;
		}
	}
}

#define MAX_RETRY 100
double read_angle(uint16_t cmd)
{
	uint16_t data;
	double angle;
	int i = 0;

	do
	{
		data = Read_From_AS5047P(cmd);
		i++;
	} while (data == 0 && i < MAX_RETRY);

	if (i >= MAX_RETRY)
	{
		// 超过最大重试次数, 可以抛出异常或返回一个错误标志
		printf("angle read error");
		return 0.0;
	}

	static const double ANGLE_COEF = 360.0 / 16384.0;
	angle = (double)data * ANGLE_COEF;
	return angle;
}



