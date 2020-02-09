/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "funkcije.h"
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
uint32_t Pulsewidth_us=0;
uint32_t PulsewidthCalc_us=0;
uint32_t PulsewidthCalc_us_limited=0;
uint32_t statedelaycount=0;
uint32_t MotorStateManual=0;
uint32_t MotorStatus=0; //0-OFF, 1-MANUAL, 2-AUTO
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;

extern uint32_t watch1;
extern uint32_t watch2;
extern uint32_t watch3;
extern uint32_t watch4;
extern uint32_t watch5;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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

  //GET INPUT PWM
  Pulsewidth_us=HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);//numbers 1000-2000 [us] +-3 possible
  if(Pulsewidth_us<1000)Pulsewidth_us=1000;						//Saturate max Value
  else if(Pulsewidth_us>2000)Pulsewidth_us=2000;				//Saturate min Value
  PulsewidthCalc_us=Pulsewidth_us-1000;							//numbers 0-1000 [us]

  //limit VALUE TO MAX PWM
  PulsewidthCalc_us_limited=PulsewidthCalc_us;
  if(PulsewidthCalc_us_limited>=200)PulsewidthCalc_us_limited=200;

  //Manual spinning conditon
  if(MotorStatus==0 && PulsewidthCalc_us >= MINSTARTTRESHOLD)
  {
	  MotorStatus=1;
	  MotorStateManual=0;
  }

  //Stop condition
  if(MotorStatus!=0 && PulsewidthCalc_us < MINSTARTTRESHOLD)
  {
	  MotorStatus=0;
	  AllPhaseOFF();
  }

  //Manually change state to achieve spin
  if(MotorStatus==1)
  {
	  statedelaycount++;
	  if(statedelaycount>= MANUALSPINSTATEDELAY)
	  {
		  set_next_step(MotorStateManual,MAUNALPWMSTART);
		  MotorStateManual++;
		  if(MotorStateManual==6){MotorStateManual=0;}
		  statedelaycount=0;
	  }
  }

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  LED_ON;
  TIM3->SR &=~(TIM_SR_UIF); //clear UIF flag

  static uint32_t Acrossstate;
  static uint32_t Bcrossstate;
  static uint32_t Ccrossstate;

  uint32_t AcrossstateHist;
  uint32_t BcrossstateHist;
  uint32_t CcrossstateHist;

  static uint32_t CurrentState=0;

  uint32_t StateChangeValid=0;
  uint32_t StateChange;
  uint32_t StateChangeFlag;

  static uint32_t cycles=0;
  static uint32_t StateChangeCount=0;
  static uint32_t DelayNextStep=0;

  AcrossstateHist=Acrossstate;
  BcrossstateHist=Bcrossstate;
  CcrossstateHist=Ccrossstate;

  Acrossstate=ACROSSSTAT;
  Bcrossstate=BCROSSSTAT;
  Ccrossstate=CCROSSSTAT;

  if(Acrossstate!=AcrossstateHist)
  {
	  if(Acrossstate)
	  {
		  StateChange=4;
	  }
	  else
	  {
		  StateChange=1;
	  }

	  StateChangeFlag=1;
  }

  else if(Bcrossstate!=BcrossstateHist)
  {
	  if(Bcrossstate)
	  {
		  StateChange=0;
	  }
	  else
	  {
		  StateChange=3;
	  }

	  StateChangeFlag=1;
  }

  else if(Ccrossstate!=CcrossstateHist)
  {
	  if(Ccrossstate)
	  {
		  StateChange=2;
	  }
	  else
	  {
		  StateChange=5;
	  }
	  StateChangeFlag=1;
  }

  else StateChangeFlag=0;

  if(StateChangeFlag)
  {
	  switch(StateChange)
	  {
	  	  case 0 : {
	  		  	  	  if(CurrentState==5)StateChangeValid=1;
	  	  	  	  	  else StateChangeValid=0;
	  	  	  	   }break;
	  	  case 1 : {
	  		  	  	  if(CurrentState==0)StateChangeValid=1;
	  	  	  	  	  else StateChangeValid=0;
	  	  	  	   }break;
	  	  case 2 : {
	  		  	  	  if(CurrentState==1)StateChangeValid=1;
	  	  	  	  	  else StateChangeValid=0;
	  	  	  	   }break;
	  	  case 3 : {
	  		  	  	  if(CurrentState==2)StateChangeValid=1;
	  	  	  	  	  else StateChangeValid=0;
	  	  	  	   }break;
	  	  case 4 : {
	  		  	  	  if(CurrentState==3)StateChangeValid=1;
	  	  	  	  	  else StateChangeValid=0;
	  	  	  	   }break;
	  	  case 5 : {
	  		  	  	  if(CurrentState==4)StateChangeValid=1;
	  	  	  	  	  else StateChangeValid=0;
	  	  	  	   }break;
	  }
  }

  //Estimate if motor is spinning enought to switch to AUTO state management
  //Check 100ms time window 10us x 10000

  if(MotorStatus==1)
  {
	  if(StateChangeFlag)
	  {
		  StateChangeCount++;
	  }
	  cycles++;

	  if(StateChangeCount>MANUALTOAUTOTHRESHOULD)MotorStatus=2;

	  else if(cycles>10000) //reset not enough rpm in 100ms
	  {
		  StateChangeCount=0;
		  cycles=0;
	  }
  }
  else if (MotorStatus==2)
  {
	  if(StateChangeValid && StateChangeFlag)
	  {
		  DelayNextStep=STEPPHASEDELAY;
	  }
	  if(DelayNextStep>=0)DelayNextStep--;
	  else set_next_step(StateChange,PulsewidthCalc_us_limited);


  }


  LED_OFF;
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
