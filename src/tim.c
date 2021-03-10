
#include "tim.h"

TIM_HandleTypeDef Tim1Handle;


void TIM1_Init(uint32_t period)
{

	TIM_OC_InitTypeDef Tim1OConfig;
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  Tim1Handle.Instance = TIM1;
  Tim1Handle.Init.Prescaler = 200-1;
  Tim1Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  Tim1Handle.Init.Period = period;
  Tim1Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  Tim1Handle.Init.RepetitionCounter = 0;
  Tim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&Tim1Handle) != HAL_OK)
  {
    Error_Handler();
  }


	if(HAL_TIM_Base_Start_IT(&Tim1Handle) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&Tim1Handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&Tim1Handle) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&Tim1Handle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  Tim1OConfig.OCMode = TIM_OCMODE_TIMING;
  Tim1OConfig.Pulse = 0;
  Tim1OConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  Tim1OConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  Tim1OConfig.OCFastMode = TIM_OCFAST_DISABLE;
  Tim1OConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  Tim1OConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	
  if (HAL_TIM_OC_ConfigChannel(&Tim1Handle, &Tim1OConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

	if(HAL_TIM_OC_Start_IT(&Tim1Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)
	{	
	  /* TIMx Peripheral clock enable */
	  __HAL_RCC_TIM1_CLK_ENABLE();
	  
	  /*##-2- Configure the NVIC for TIMx ########################################*/
	  /* Set the TIMx priority */
	  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 1);
	  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 1);
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  }
} 



void TIM1_UpdateCCR(uint32_t time)
{
	 __HAL_TIM_SET_COMPARE(&Tim1Handle, TIM_CHANNEL_1, time);	
}


uint32_t TIM1_GetCCR(void)
{
	return (HAL_TIM_ReadCapturedValue(&Tim1Handle, TIM_CHANNEL_1));
}

uint32_t TIM1_GetCounter(void)
{
	return (__HAL_TIM_GET_COUNTER(&Tim1Handle));
}




