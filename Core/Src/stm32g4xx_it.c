/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AS5047P.h"
#include "motor.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Encoder_Num_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

// 防止上次按下操作未完成就频繁开启定时器
volatile uint8_t button_pressed = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == KEY_Pin)
  {
    if (button_pressed == 0)
    {
      button_pressed = 1;
      // 启动定时器进行防抖处理
      HAL_TIM_Base_Start_IT(&htim7);
    }
  }
  else if (GPIO_Pin == Encoder_Num_Pin)
  {
    int cnt = TIM3->CNT;
    // 获取电机旋转方向

    my_motor.dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
    // printf("dir: %d\n", my_motor.dir);
    // 正转
    if (my_motor.dir == 0)
    {
      ++my_motor.encoder.number;
      // printf("turn right: %d\n", my_motor.encoder.number);
    }
    // 反转
    else if (my_motor.dir == 1)
    {
      --my_motor.encoder.number;
      // printf("turn left: %d\n", my_motor.encoder.number);
    }
    my_motor.dir = -1;
  }
}
int sw;
// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM7)
  {
    HAL_TIM_Base_Stop_IT(htim); // 停止定时器

    // 检查按键状态
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      if (sw == 0)
      {
        HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);
        sw = 1;
      }
      else
      {
        HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);
        sw = 0;
      }
    }

    button_pressed = 0; // 复位按键状态
  }
  else if (htim->Instance == TIM6)
  {
    float now_angle = read_angle_ABZ();
    float delta_angle = now_angle - my_motor.lastAngle;
    my_motor.lastAngle = now_angle;
    int dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);

    // 正转
    if (dir == 0)
    {
      if (delta_angle < 0)
      {
        delta_angle += 360.0f;
      }
      my_motor.speed = delta_angle / 360.0f * 1000; // 1000是时间单位
    }
    // 反转
    else if (dir == 1)
    {
      if (delta_angle > 0)
      {
        delta_angle -= 360.0f;
      }
      my_motor.speed = delta_angle / 360.0f * 1000; // 1000是时间单位
    }

    // Vofa_FireWater("%.2f\r\n",my_motor.speed);
  }
  else if (htim->Instance == TIM1)
  {

#define ADC_CONV_FACTOR (3.3f / 4096.0f)
#define CURRENT_CONV_FACTOR (100.0f / 16.5f)

    static int adc_sta = 0; // 用于控制校准和采集状态
    float adc_converted;

    if (adc_sta == 0)
    { // 校准状态
      if (my_motor.adc_u.cnt < 500)
      {
        // 记录上电值
        adc_buff[0] = ADC1->JDR1;
        adc_buff[1] = ADC1->JDR2;
        adc_buff[2] = ADC1->JDR3;
        adc_buff[3] = ADC1->JDR4;

        // 分别计算Iu, Iv, Iw的累计和
        my_motor.adc_u.Iu_sum_offer += ((float)(-adc_buff[0]) * ADC_CONV_FACTOR - my_motor.adc_u.adc_refer) * CURRENT_CONV_FACTOR;
        my_motor.adc_u.Iv_sum_offer += ((float)(-adc_buff[1]) * ADC_CONV_FACTOR - my_motor.adc_u.adc_refer) * CURRENT_CONV_FACTOR;
        my_motor.adc_u.Iw_sum_offer += ((float)(-adc_buff[2]) * ADC_CONV_FACTOR - my_motor.adc_u.adc_refer) * CURRENT_CONV_FACTOR;

        my_motor.adc_u.cnt++;
        // printf("Calibration cnt: %d\n", my_motor.adc_u.cnt);
      }
      else if (my_motor.adc_u.cnt == 500)
      {
        // 计算均值
        my_motor.adc_u.Iu_Sample_offer = my_motor.adc_u.Iu_sum_offer / 500.0f;
        my_motor.adc_u.Iv_Sample_offer = my_motor.adc_u.Iv_sum_offer / 500.0f;
        my_motor.adc_u.Iw_Sample_offer = my_motor.adc_u.Iw_sum_offer / 500.0f;

        // printf("Calibration done: %.2f, %.2f, %.2f\r\n", my_motor.adc_u.Iu_Sample_offer, my_motor.adc_u.Iv_Sample_offer, my_motor.adc_u.Iw_Sample_offer);

        // 切换到采集状态
        adc_sta = 1;
      }
    }

    if (adc_sta == 1)
    { // 采集状态
      adc_buff[0] = ADC1->JDR1;
      adc_buff[1] = ADC1->JDR2;
      adc_buff[2] = ADC1->JDR3;
      adc_buff[3] = ADC1->JDR4;

      my_motor.adc_u.Iu_Sample = -adc_buff[0]; // 反向，一般电流取流入电机方向为正
      my_motor.adc_u.Iv_Sample = -adc_buff[1];
      my_motor.adc_u.Iw_Sample = -adc_buff[2];

      // 实际值计算
      my_motor.adc_u.Iu = (((float)my_motor.adc_u.Iu_Sample * ADC_CONV_FACTOR - my_motor.adc_u.adc_refer) * CURRENT_CONV_FACTOR) - my_motor.adc_u.Iu_Sample_offer;
      my_motor.adc_u.Iv = (((float)my_motor.adc_u.Iv_Sample * ADC_CONV_FACTOR - my_motor.adc_u.adc_refer) * CURRENT_CONV_FACTOR) - my_motor.adc_u.Iv_Sample_offer;
      my_motor.adc_u.Iw = (((float)my_motor.adc_u.Iw_Sample * ADC_CONV_FACTOR - my_motor.adc_u.adc_refer) * CURRENT_CONV_FACTOR) - my_motor.adc_u.Iw_Sample_offer;

      my_motor.adc_u.Udc = ((float)adc_buff[3] * ADC_CONV_FACTOR) * 25.0f;

      printf("%.2f, %.2f, %.2f\r\n", my_motor.adc_u.Iu, my_motor.adc_u.Iv, my_motor.adc_u.Iw);
      my_motor.foc.theta += 0.1;
      ipark(&my_motor.foc);
      svpwm(&my_motor.foc);
      Set_SVPWM_Compare(my_motor.foc.t_a, my_motor.foc.t_b, my_motor.foc.t_c);
    }
  }
}
/* USER CODE END 1 */
