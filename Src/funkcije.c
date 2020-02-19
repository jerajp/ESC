
#include "funkcije.h"
#include "main.h"

//DEBUGGING--------------------------------------------------

//void AH_BL(uint32_t Pulsewidth)
//{
//
//	TIM2->CCR1=0;		 	//Phase A   0
//	TIM2->CCR2=0;		   	//Phase B   0
//	TIM2->CCR3=0;			//Phase c   0
//
//	ALowOFF;
//	BLowOFF;
//	CLowOFF;
//}
//
//void AH_CL(uint32_t Pulsewidth)
//{
//	TIM2->CCR1=0;		 	//Phase A   0
//	TIM2->CCR2=0;		   	//Phase B   0
//	TIM2->CCR3=0;			//Phase c   0
//
//	ALowOFF;
//	BLowOFF;
//	CLowOFF;
//}
//
//void BH_CL(uint32_t Pulsewidth)
//{
//	TIM2->CCR1=0;		 	//Phase A   0
//	TIM2->CCR2=0;		   	//Phase B   0
//	TIM2->CCR3=0;			//Phase c   0
//
//	ALowOFF;
//	BLowOFF;
//	CLowOFF;
//}
//
//void BH_AL(uint32_t Pulsewidth)
//{
//	TIM2->CCR1=0;		 	//Phase A   0
//	TIM2->CCR2=0;		   	//Phase B   0
//	TIM2->CCR3=0;			//Phase c   0
//
//	ALowOFF;
//	BLowOFF;
//	CLowOFF;
//}
//
//void CH_AL(uint32_t Pulsewidth)
//{
//	TIM2->CCR1=0;		 	//Phase A   0
//	TIM2->CCR2=0;		   	//Phase B   0
//	TIM2->CCR3=0;			//Phase c   0
//
//	ALowOFF;
//	BLowOFF;
//	CLowOFF;
//}
//
//void CH_BL(uint32_t Pulsewidth)
//{
//	TIM2->CCR1=0;		 	//Phase A   0
//	TIM2->CCR2=0;		   	//Phase B   0
//	TIM2->CCR3=0;			//Phase c   0
//
//	ALowOFF;
//	BLowOFF;
//	CLowOFF;
//}


void AH_BL(uint32_t Pulsewidth)
{

	TIM2->CCR1=Pulsewidth; 	//Phase A   PWM
	TIM2->CCR2=0;		   	//Phase B   0
	TIM2->CCR3=0;			//Phase c   0

	ALowOFF;
	BLowON;
	CLowOFF;
}

//Phase A PWM, C low, B open
void AH_CL(uint32_t Pulsewidth)
{
	TIM2->CCR1=Pulsewidth; 	//Phase A	PWM
	TIM2->CCR2=0;		   	//Phase B   0
	TIM2->CCR3=0;			//Phase c   0

	ALowOFF;
	BLowOFF;
	CLowON;
}

//Phase B PWM, C low, A open
void BH_CL(uint32_t Pulsewidth)
{
	TIM2->CCR1=0; 			//Phase A 	0
	TIM2->CCR2=Pulsewidth;	//Phase B   PWM
	TIM2->CCR3=0;			//Phase c   0

	ALowOFF;
	BLowOFF;
	CLowON;
}

//Phase B PWM, A low, C open
void BH_AL(uint32_t Pulsewidth)
{
	TIM2->CCR1=0; 			//Phase A 	0
	TIM2->CCR2=Pulsewidth;	//Phase B   PWM
	TIM2->CCR3=0;			//Phase c   0


	ALowON;
	BLowOFF;
	CLowOFF;
}

//Phase C PWM, A low, B open
void CH_AL(uint32_t Pulsewidth)
{
	TIM2->CCR1=0; 			//Phase A 	0
	TIM2->CCR2=0;			//Phase B   0
	TIM2->CCR3=Pulsewidth;	//Phase c   PWM

	ALowON;
	BLowOFF;
	CLowOFF;
}

//Phase C PWM, B low, C open
void CH_BL(uint32_t Pulsewidth)
{
	TIM2->CCR1=0; 			//Phase A 	0
	TIM2->CCR2=0;			//Phase B   0
	TIM2->CCR3=Pulsewidth;	//Phase c   PWM

	ALowOFF;
	BLowON;
	CLowOFF;
}

void AllPhaseOFF()
{
	TIM2->CCR1=0;		 	//Phase A   0
	TIM2->CCR2=0;		   	//Phase B   0
	TIM2->CCR3=0;			//Phase c   0

	ALowOFF;
	BLowOFF;
	CLowOFF;
}

void set_next_step(uint32_t state, uint32_t Pulsewidth)
{
  switch(state)
  {
    case 0:
      AH_BL(Pulsewidth);
      break;
    case 1:
      AH_CL(Pulsewidth);
      break;
    case 2:
      BH_CL(Pulsewidth);
      break;
    case 3:
      BH_AL(Pulsewidth);
      break;
    case 4:
      CH_AL(Pulsewidth);
      break;
    case 5:
      CH_BL(Pulsewidth);
      break;
  }
}

void SetNextState(uint32_t * MotoStat, uint32_t * PWMValue)
{
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

	//Save previous States
	AcrossstateHist=Acrossstate;
	BcrossstateHist=Bcrossstate;
	CcrossstateHist=Ccrossstate;

	//Read Zero Cross Pins
	Acrossstate=ACROSSSTAT;
	Bcrossstate=BCROSSSTAT;
	Ccrossstate=CCROSSSTAT;


	if(Acrossstate!=AcrossstateHist)
	{
		if(Acrossstate)
		{
			ZeroCrossStateChange=1;

			if(*MotoStat==1)
			{
				CurrentState=0;
			}
		}
		else
		{
			ZeroCrossStateChange=2;

			if(*MotoStat==1)
			{
				CurrentState=1;
			}

		}
		ZeroCrossFlag=1;
	}

	else if(Bcrossstate!=BcrossstateHist)
	{
		if(Bcrossstate)
		{
			ZeroCrossStateChange=3;

			if(*MotoStat==1)
			{
				CurrentState=2;
			}
		}
		else
		{
			ZeroCrossStateChange=4;

			if(*MotoStat==1)
			{
				CurrentState=3;
			}
		}
		ZeroCrossFlag=1;
	}

	else if(Ccrossstate!=CcrossstateHist)
	{
		if(Ccrossstate)
		{
			ZeroCrossStateChange=5;

			if(*MotoStat==1)
			{
				CurrentState=4;
			}
		}
		else
		{
			ZeroCrossStateChange=0;

			if(*MotoStat==1)
			{
				CurrentState=5;
			}
		}
		ZeroCrossFlag=1;
	}

	else ZeroCrossFlag=0;


	if(ZeroCrossFlag)
	{
		switch(ZeroCrossStateChange)
		{
			case 0 :{
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
			case 1 :{
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
			case 2 :{
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
			case 3 :{
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
			case 4 :{
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
			case 5 :{
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

	if (*MotoStat==2)
	{
		if(ZeroCrossValid && ZeroCrossFlag)
		{

			ChangeStateFlag=1; 				//Trigger State change
		}

		if(ChangeStateFlag)
		{
			set_next_step(ZeroCrossStateChange,* PWMValue);
			CurrentState=ZeroCrossStateChange; 	//set new state
			ChangeStateFlag=0; 					//reset
		}
	}


}
