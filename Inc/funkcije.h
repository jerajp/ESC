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
