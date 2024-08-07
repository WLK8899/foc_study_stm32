/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_IU_Pin GPIO_PIN_0
#define ADC_IU_GPIO_Port GPIOA
#define ADC_IV_Pin GPIO_PIN_1
#define ADC_IV_GPIO_Port GPIOA
#define ADC_IW_Pin GPIO_PIN_2
#define ADC_IW_GPIO_Port GPIOA
#define ADC_Udc_Pin GPIO_PIN_3
#define ADC_Udc_GPIO_Port GPIOA
#define KEY_Pin GPIO_PIN_4
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI4_IRQn
#define Encode_A_Pin GPIO_PIN_6
#define Encode_A_GPIO_Port GPIOA
#define Encoder_B_Pin GPIO_PIN_7
#define Encoder_B_GPIO_Port GPIOA
#define Encoder_Num_Pin GPIO_PIN_0
#define Encoder_Num_GPIO_Port GPIOB
#define Encoder_Num_EXTI_IRQn EXTI0_IRQn
#define ERR_Pin GPIO_PIN_11
#define ERR_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define PWM1N_Pin GPIO_PIN_13
#define PWM1N_GPIO_Port GPIOB
#define PWM2N_Pin GPIO_PIN_14
#define PWM2N_GPIO_Port GPIOB
#define PWM3N_Pin GPIO_PIN_15
#define PWM3N_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_11
#define ENABLE_GPIO_Port GPIOA
#define SPI_CSS_Pin GPIO_PIN_15
#define SPI_CSS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
