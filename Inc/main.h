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

#define MANUALSPINSTATEDELAYMIN  100      	//minimum delay between states x10us
#define MANUALSPINSTATEDELAY 	 397      	//Delay between states in 10us cyles for 360RPM -> 6turn/s ->x42 state changes/turn -time between states 1/252 seconds -->3968us delay ->397 (10us) cycles
#define MAUNALPWM				 120      	//(Range 0 counts of 1000)
#define MANAULDELAYDECREASE		 1			//PWM decrease per cycle
#define CYCLESWITHMANUALRPM		 2000       //20ms time in 10us quants in MANUAL MODE
#define MINRPMAUTORPMTHRESHOLD 	 500 		//RPM switch off
#define CYCLESWITHMINTRPM  		 5000 	  	//50ms with MIN RPM ->switch to MANUAL
#define MOTORSTATUSONETIMEOUT    3000 		//50msec if Motor doesn't start return back to Zero State

#define MINSTARTTRESHOLD	     50       	//(Range 0 counts of 1000)
#define PWM_MAX_LIMIT 		     1000

#define STALLCOUNT			     10000 		//10us loop period x8000=100ms with no ratation latch motor

//PWM REFERENCE CLOSING IN TO ACTUAL PWM
#define PWMSTEP 1
#define SMOOTHCYCLES 50 //10 too fast falls out of sync


//RPM TIMING
#define MEASURETIMEMILISECOND 10 	 //10ms measure window
#define ONESECONDTOMILISECOND 1000   //1000ms
#define RPMTORPS  			  60     //1 RPM-->60 Rounds per second
#define ZEROCROSSPERTURN      42     //6 zero cross per 1/7 mechanical turn->42 crossings per turn


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
