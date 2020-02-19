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

//PWM
uint32_t Pulsewidth_us=0;
uint32_t PulsewidthCalc_us=0;
uint32_t PulsewidthCalc_us_limited=0;
uint32_t PulsewidthCalc_us_limited_smooth=0;

uint32_t MotorStatus=0; //0-OFF, 1-MANUAL, 2-AUTO
uint32_t ZeroCrossCount=0;
uint32_t MotorRPM=0;
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
  if(PulsewidthCalc_us_limited>=PWM_MAX_LIMIT)PulsewidthCalc_us_limited=PWM_MAX_LIMIT;

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
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  EXTI->PR |=(EXTI_PR_PR0); //clear IT flag
  watch1++;
  SetNextState(&MotorStatus, &PulsewidthCalc_us_limited_smooth);
  ZeroCrossCount++;

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  EXTI->PR |=(EXTI_PR_PR1); //clear IT flag
  watch2++;
  SetNextState(&MotorStatus, &PulsewidthCalc_us_limited_smooth);
  ZeroCrossCount++;

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  LED_ON;
  TIM3->SR &=~(TIM_SR_UIF); //clear UIF flag

  static uint32_t timestep=0;
  static uint32_t ManualRPMstablecount=0;
  static uint32_t AutoRPMnotstablecount=0;
  static uint32_t ManualSpinstatedelay=0;
  static uint32_t MotorStateManual=0;
  int32_t DeltaPWMsign=0;


  //MANUAL SPIN ROUTINE
  if(MotorStatus==0 && PulsewidthCalc_us >= MINSTARTTRESHOLD)
  {
	  MotorStatus=1;
	  MotorStateManual=0;
	  ManualRPMstablecount=0;
	  AutoRPMnotstablecount=0;
  }

  //Manually change state to achieve spin
  if(MotorStatus==1)
  {
	  ManualSpinstatedelay++;
	  if(ManualSpinstatedelay>= MANUALSPINSTATEDELAY)
	  {
		  set_next_step(MotorStateManual,MAUNALPWMSTART);
		  MotorStateManual++;
		  if(MotorStateManual==6){MotorStateManual=0;}
		  ManualSpinstatedelay=0;
	  }
  }

  //Estimate if motor is with ENOUGH RPM FOR ENOUGH TIME TO SWITCH TO AUTO SPIN(MotorStatus=2)
  if(MotorStatus==1 && MotorRPM>MANUALTOAUTORPMTHRESHOLD)
  {
	  ManualRPMstablecount++;
	  if(ManualRPMstablecount>CYCLESWITHMINTRPM)
	  {
		  MotorStatus=2;
		  PulsewidthCalc_us_limited_smooth=MAUNALPWMSTART;
	  }
  }

  //APPROACH SET PWM----------------------------------------------------------
  if(MotorStatus==2)
  {
	  DeltaPWMsign=PulsewidthCalc_us_limited-PulsewidthCalc_us_limited_smooth;

	  if(DeltaPWMsign < (int32_t)(0)) //PWM increase
	  {
		  if(  (uint32_t)(DeltaPWMsign*(-1)) > PWMSTEP)
		  {
			  if(PulsewidthCalc_us_limited_smooth>PWMSTEP)
			  {
				  PulsewidthCalc_us_limited_smooth-=PWMSTEP;
			  }
		  }
	  }
	  else
	  {
		  if(  (uint32_t)(DeltaPWMsign) > PWMSTEP)
		  {
			  PulsewidthCalc_us_limited_smooth+=PWMSTEP;

			  if(PulsewidthCalc_us_limited_smooth>PulsewidthCalc_us_limited)
			  {
				  PulsewidthCalc_us_limited_smooth=PulsewidthCalc_us_limited;
			  }
		  }
	  }
  }//------------------------------------------------------------------------

  //Keep ZeroCross value at 0
  if(MotorStatus==0)
  {
	  ZeroCrossCount=0;
  }

  //Measure RPM-----STAT 1----------------------------
  if(MotorStatus==2 || MotorStatus==1)
  {
	  timestep++;
	  if(timestep==(MEASURETIMEMILISECOND*100) ) //100x10us (IT cycle) =1ms
	  {
		MotorRPM=(ZeroCrossCount*RPMTORPS*ONESECONDTOMILISECOND)/(MEASURETIMEMILISECOND*ZEROCROSSPERTURN);
		ZeroCrossCount=0;
		timestep=0;
	  }
  }

  //MOTOR STATUS ->0 PWM TOO LOW---------------------------------------
  if(MotorStatus!=0 && PulsewidthCalc_us < MINSTARTTRESHOLD)
  {
	  MotorStatus=0;
	  AllPhaseOFF();
  }

  //MOTOR STATUS ->0 RPM TOO LOW--------------------------------------
//  if(MotorStatus==2)
//  {
//	  if(MotorRPM<MANUALTOAUTORPMTHRESHOLD)
//	  {
//		  AutoRPMnotstablecount++;
//	  }
//
//	  if(AutoRPMnotstablecount>CYCLESWITHMINTRPM)
//	  {
//		  MotorStatus=0;
//	  }
//  }

  LED_OFF;
  /* USER CODE END TIM3_IRQn 0 */
  /* USER CODE BEGIN TIM3_IRQn 1 */
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  EXTI->PR |=(EXTI_PR_PR10); //clear IT flag
  watch3++;
  SetNextState(&MotorStatus, &PulsewidthCalc_us_limited_smooth);
  ZeroCrossCount++;

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
