
#ifndef _TIM_H_
#define _TIM_H_

#ifdef __cplusplus
 extern "C" {
#endif


#include "main.h"
#include "stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_tim_ex.h"



void TIM1_init(void);

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle);

void TIM1_UpdateCCR(uint32_t time);
uint32_t TIM1_GetCCR(void);
uint32_t TIM1_GetCounter(void);


#ifdef __cplusplus
}
#endif

#endif


