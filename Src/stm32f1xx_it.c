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

extern uint32_t watchState0;
extern uint32_t watchState1;
extern uint32_t watchState2;
extern uint32_t watchState3;
extern uint32_t watchState4;
extern uint32_t watchState5;
extern uint32_t watchState0Err;
extern uint32_t watchState1Err;
extern uint32_t watchState2Err;
extern uint32_t watchState3Err;
extern uint32_t watchState4Err;
extern uint32_t watchState5Err;
extern uint32_t watchState;
extern uint32_t watch1;
extern uint32_t watch2;
extern uint32_t watch3;
extern uint32_t watchArray[1000];

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
  printf("MS=%u \n",MotorStatus);
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
  static uint32_t ChangeStateFlag=0;


  uint32_t ZeroCrossValid=0;
  uint32_t ZeroCrossFlag;
  static uint32_t ZeroCrossStateChange;

  static uint32_t cycles=0;
  static uint32_t ZeroCrossCount=0;
  static uint32_t DelayNextStep=0;

  AcrossstateHist=Acrossstate;
  BcrossstateHist=Bcrossstate;
  CcrossstateHist=Ccrossstate;

  Acrossstate=ACROSSSTAT;
  Bcrossstate=BCROSSSTAT;
  Ccrossstate=CCROSSSTAT;

  static uint32_t testcount=0;


  if(Acrossstate!=AcrossstateHist)
  {
	  if(Acrossstate)
	  {
		  ZeroCrossStateChange=2;//4;
		  if(MotorStatus==1){CurrentState=1;}
		  if(MotorStatus!=0)
		  {
			  watchArray[testcount]=ZeroCrossStateChange;
			  if(testcount<999)testcount++;
		  }

	  }
	  else
	  {
		  ZeroCrossStateChange=3;//1;
		  if(MotorStatus==1){CurrentState=2;}
		  if(MotorStatus!=0)
		  {
			  watchArray[testcount]=ZeroCrossStateChange;
			  if(testcount<999)testcount++;
		  }
	  }

	  ZeroCrossFlag=1;
  }

  else if(Bcrossstate!=BcrossstateHist)
  {
	  if(Bcrossstate)
	  {
		  ZeroCrossStateChange=4;//0;
		  if(MotorStatus==1){CurrentState=3;}
		  if(MotorStatus!=0)
		  {
			  watchArray[testcount]=ZeroCrossStateChange;
			  if(testcount<999)testcount++;
		  }
	  }
	  else
	  {
		  ZeroCrossStateChange=5;//3;
		  if(MotorStatus==1){CurrentState=4;}
		  if(MotorStatus!=0)
		  {
			  watchArray[testcount]=ZeroCrossStateChange;
			  if(testcount<999)testcount++;
		  }
	  }

	  ZeroCrossFlag=1;
  }

  else if(Ccrossstate!=CcrossstateHist)
  {
	  if(Ccrossstate)
	  {
		  ZeroCrossStateChange=0;//2;
		  if(MotorStatus==1){CurrentState=5;}
		  if(MotorStatus!=0)
		  {
			  watchArray[testcount]=ZeroCrossStateChange;
			  if(testcount<999)testcount++;
		  }
	  }
	  else
	  {
		  ZeroCrossStateChange=1;//5;
		  if(MotorStatus==1){CurrentState=0;}
		  if(MotorStatus!=0)
		  {
			  watchArray[testcount]=ZeroCrossStateChange;
			  if(testcount<999)testcount++;
		  }
	  }
	  ZeroCrossFlag=1;
  }

  else ZeroCrossFlag=0;


  if(ZeroCrossFlag)
  {
	  switch(ZeroCrossStateChange)
	  {
	  	  case 0 : {
	  		  	  	  if(CurrentState==5)
	  		  	  	  {
	  		  	  		  ZeroCrossValid=1;
	  		  	  		  watchState0++;
	  		  	  	  }
	  	  	  	  	  else
	  	  	  	  	  {
	  	  	  	  		  ZeroCrossValid=0;
	  	  	  	  	      watchState0Err++;
	  	  	  	  	  }
	  	  	  	   }break;
	  	  case 1 : {
	  		  	  	  if(CurrentState==0)
	  		  	  	  {
	  		  	  		  ZeroCrossValid=1;
	  		  	  		  watchState1++;
	  		  	  	  }
	  	  	  	  	  else
	  	  	  	  	  {
	  	  	  	  		  ZeroCrossValid=0;
	  	  	  	  		  watchState1Err++;
	  	  	  	  	  }
	  	  	  	   }break;
	  	  case 2 : {
	  		  	  	  if(CurrentState==1)
	  		  	  	  {
	  		  	  		  ZeroCrossValid=1;
	  		  	  		  watchState2++;
	  		  	  	  }
	  	  	  	  	  else
	  	  	  	      {
	  	  	  	  		  ZeroCrossValid=0;
	  	  	  	  		  watchState2Err++;
	  	  	  	      }
	  	  	  	   }break;
	  	  case 3 : {
	  		  	  	  if(CurrentState==2)
	  		  	  	  {
	  		  	  		  ZeroCrossValid=1;
	  		  	  		  watchState3++;
	  		  	  	  }
	  	  	  	  	  else
	  	  	  	      {
	  	  	  	  		  ZeroCrossValid=0;
	  	  	  	  		  watchState3Err++;
	  	  	  	      }
	  	  	  	   }break;
	  	  case 4 : {
	  		  	  	  if(CurrentState==3)
	  		  	  	  {
	  		  	  		  ZeroCrossValid=1;
	  		  	  		  watchState4++;
	  		  	  	  }
	  	  	  	  	  else
	  	  	  	  	  {
	  	  	  	  		  ZeroCrossValid=0;
	  	  	  	  		  watchState4Err++;
	  	  	  	  	  }
	  	  	  	   }break;
	  	  case 5 : {
	  		  	  	  if(CurrentState==4)
	  		  	  	  {
	  		  	  		  ZeroCrossValid=1;
	  		  	  		  watchState5++;
	  		  	  	  }
	  	  	  	  	  else
	  	  	  	      {
	  	  	  	  		  ZeroCrossValid=0;
	  	  	  	  		  watchState5Err++;
	  	  	  	      }
	  	  	  	   }break;
	  }
  }

  //Estimate if motor is spinning enough to switch to AUTO state management
  //Check 100ms time window 10us x 10000

  if(MotorStatus==1)
  {
	  if(ZeroCrossFlag)
	  {
		  ZeroCrossCount++;
	  }
	  cycles++;

	  if(ZeroCrossCount>MANUALTOAUTOTHRESHOULD)
	  {
		  MotorStatus=2; 		//start auto spin
		  ZeroCrossCount=0;		//reset
		  cycles=0;				//reset
	  }

	  else if(cycles>10000) //reset not enough rpm in 100ms
	  {
		  ZeroCrossCount=0;
		  cycles=0;
	  }
  }
  else if (MotorStatus==2)
  {
	  watch1++;

	  if(ZeroCrossValid && ZeroCrossFlag)
	  {
		  watch2++;
		  ChangeStateFlag=1; 			//Trigger State change
		  DelayNextStep=STEPPHASEDELAY; //State change after delay
	  }

	  else if(ChangeStateFlag)
	  {

		  if(DelayNextStep>0)DelayNextStep--;
		  else
		  {
			  set_next_step(ZeroCrossStateChange,PulsewidthCalc_us_limited);
			  CurrentState=ZeroCrossStateChange; //set new state
			  ChangeStateFlag=0; 				//reset
			  watch3++;
		  }
	  }
  }

  watchState=CurrentState;
  LED_OFF;
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
