#include "stm32f1xx_hal.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim2;

void AH_BL(uint32_t Pulsewidth);
void AH_CL(uint32_t Pulsewidth);
void BH_CL(uint32_t Pulsewidth);
void BH_AL(uint32_t Pulsewidth);
void CH_AL(uint32_t Pulsewidth);
void CH_BL(uint32_t Pulsewidth);

void AllPhaseOFF();

void set_next_step(uint32_t state, uint32_t Pulsewidth);
void SetNextState(uint32_t * MotoStat, uint32_t * PWMValue, uint32_t * ZeroCrossCnt);


extern uint32_t watch1;
extern uint32_t watch2;
extern uint32_t watch3;

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
