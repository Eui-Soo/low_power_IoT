/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _APP_MAIN_H
#define _APP_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_gpio.h"






/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);


/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB


/* 	LL-ULP RFM I/F GPIO		*/
#if 1
/*
#define FRZ_RX_WK_Pin			GPIO_PIN_10
#define FRZ_WK_GPIO_Port		GPIOB
#define FRZ_WK_EXTI_LINE		EXTI_IMR1_IM10
#define FRZ_WK_EXTI_IRQn		EXTI15_10_IRQn

#define FRZ_RX_WK_Pin			GPIO_PIN_3
#define FRZ_WK_GPIO_Port		GPIOA
#define FRZ_WK_EXTI_LINE		EXTI_IMR1_IM3
#define FRZ_WK_EXTI_IRQn		EXTI3_IRQn
*/
#define FRZ_RX_WK_Pin			GPIO_PIN_10
#define FRZ_WK_GPIO_Port		GPIOC
#define FRZ_WK_EXTI_LINE		EXTI_IMR1_IM10
#define FRZ_WK_EXTI_IRQn		EXTI15_10_IRQn

#else
#define FRZ_RX_WK_Pin			GPIO_PIN_14
#define FRZ_WK_GPIO_Port	GPIOG
#define FRZ_WK_EXTI_LINE	EXTI_IMR1_IM14
#endif

#define FRZ_TX_DATA				GPIO_PIN_11
//#define RF_SWITCH_Pin			GPIO_PIN_11	// SJPARK
#define FRZ_TX_GPIO_Port		GPIOB

//#define FRZ_RX_SER				GPIO_PIN_12
//#define FRZ_ADC_CLK_Pin			GPIO_PIN_13
#define FRZ_RX_SER				GPIO_PIN_2
#define FRZ_ADC_CLK_Pin			GPIO_PIN_15
#define FRZ_RX_ADC_GPIO_Port	GPIOE




#define PA_EN_Pin				GPIO_PIN_13
#define PA_GAIN0_Pin 			GPIO_PIN_11
#define PA_GAIN1_Pin 			GPIO_PIN_14
#define FRZ_PA_GPIO_Port		GPIOE

#define FRZ_RX_DATA0_Pin		GPIO_PIN_8
#define FRZ_RX_DATA1_Pin		GPIO_PIN_7
#define FRZ_RX_DATA2_Pin		GPIO_PIN_10
#define FRZ_RX_DATA3_Pin		GPIO_PIN_12
#define FRZ_RX_DATA4_Pin		GPIO_PIN_6
#define FRZ_RX_DATA_GPIO_Port	GPIOE

#define FRZ_RX_SLROUT_Pin		GPIO_PIN_10
#define FRZ_RX_SLROUT_GPIO_Port		GPIOB

#define TEST1_Pin				GPIO_PIN_12

#define FRZ_ADC_CLK_PIN			FRZ_ADC_CLK_Pin
#define FRZ_ADC_EXTI_LINE		EXTI_IMR1_IM15
#define FRZ_ADC_EXTI_IRQn		EXTI15_10_IRQn

#define FRZ_WK_EXTI_LINE_ENABLE()	SET_BIT(EXTI_D1->IMR1, FRZ_WK_EXTI_LINE);	
#define FRZ_WK_EXTI_LINE_DISABLE()	CLEAR_BIT(EXTI_D1->IMR1, FRZ_WK_EXTI_LINE);

#define FRZ_ADC_EXTI_LINE_ENABLE()		SET_BIT(EXTI_D1->IMR1, FRZ_ADC_EXTI_LINE);
#define FRZ_ADC_EXTI_LINE_DISABLE()		CLEAR_BIT(EXTI_D1->IMR1, FRZ_ADC_EXTI_LINE);

#if defined(FEATURE_FIRENZE_RX_MODE)
//#define RFM_WAKEUP_IRQHANDLER								EXTI3_IRQHandler
#define RFM_RX_IRQHANDLER									EXTI15_10_IRQHandler
#elif defined(FEATURE_FIRENZE_TX_MODE)
#define RFM_TX_IRQHANDLER									EXTI15_10_IRQHandler
#endif

#define LEDn                                    3

typedef enum
{
  LED1 = 0,
  LED_GREEN = LED1,
  LED2 = 1,
  LED_YELLOW = LED2,
  LED3 = 2,
  LED_RED = LED3
}Led_TypeDef;

void LED_On(Led_TypeDef Led);
void LED_Off(Led_TypeDef Led);
void LED_Toggle(Led_TypeDef Led);


#ifdef __cplusplus
}
#endif

#endif /* _APP_MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
