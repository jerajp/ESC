/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define LED_ON  GPIOB->BSRR|=0x00000100  	//B8  1 << (8)
#define LED_OFF GPIOB->BSRR|=0x01000000     //B8  1 << (16+8)

#define ALowON  GPIOA->BSRR|=0x00000008  	//A3  1 << (3)
#define ALowOFF GPIOA->BSRR|=0x00080000  	//A3  1 << (16+3)

#define BLowON  GPIOA->BSRR|=0x00000010 	//A5  1 << (4)
#define BLowOFF GPIOA->BSRR|=0x00100000  	//A5  1 << (16+4)

#define CLowON  GPIOA->BSRR|=0x00000020 	//A5  1 << (5)
#define CLowOFF GPIOA->BSRR|=0x00200000  	//A5  1 << (16+5)

#define ACROSSSTAT GPIOB->IDR & GPIO_IDR_IDR0
#define BCROSSSTAT GPIOB->IDR & GPIO_IDR_IDR1
#define CCROSSSTAT GPIOB->IDR & GPIO_IDR_IDR10

#define MANUALSPINSTATEDELAY 4   //Delay between states in [ms] for manual spin
#define MINSTARTTRESHOLD	50   //(Range 0 counts of 1000)
#define MAUNALPWMSTART		100  //(Range 0 counts of 1000)
#define MANUALTOAUTOTHRESHOULD 100 //Zero crossings
#define PWM_MAX_LIMIT 1000

//Motor Spec-> 7 state sequence changes per Turn

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AHigh_Pin GPIO_PIN_0
#define AHigh_GPIO_Port GPIOA
#define Bhigh_Pin GPIO_PIN_1
#define Bhigh_GPIO_Port GPIOA
#define Chigh_Pin GPIO_PIN_2
#define Chigh_GPIO_Port GPIOA
#define ALow_Pin GPIO_PIN_3
#define ALow_GPIO_Port GPIOA
#define BLow_Pin GPIO_PIN_4
#define BLow_GPIO_Port GPIOA
#define CLow_Pin GPIO_PIN_5
#define CLow_GPIO_Port GPIOA
#define Azero_Pin GPIO_PIN_0
#define Azero_GPIO_Port GPIOB
#define Azero_EXTI_IRQn EXTI0_IRQn
#define Bzero_Pin GPIO_PIN_1
#define Bzero_GPIO_Port GPIOB
#define Bzero_EXTI_IRQn EXTI1_IRQn
#define Czero_Pin GPIO_PIN_10
#define Czero_GPIO_Port GPIOB
#define Czero_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
