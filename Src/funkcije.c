
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
