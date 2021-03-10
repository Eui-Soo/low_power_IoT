/**
 * \addtogroup mbxxx-platform
 *
 * @{
 */

/*
 * Copyright (c) 2010, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
* \file
*			Real-timer specific implementation for STM32W.
* \author
*			Salvatore Pitrulli <salvopitru@users.sourceforge.net>
*/


#include <stm32h7xx.h>
#include "stm32h7xx_hal_tim.h"
#include "sys/energest.h"
#include "sys/rtimer.h"
#include "tim.h"
 
static uint32_t time_msb = 0;
static rtimer_clock_t next_rtimer_time = 0;


/* Ÿ�̸� ī���Ͱ� �����÷ο� �Ǵ� ��� ȣ�� �� : 1msec */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1 && (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) !=RESET))
  {
		rtimer_clock_t now, clock_to_wait, clock_to_remain, tCounter;

		time_msb++;

		/* ���� Ÿ�̸� ī���͸� �����´�. ī������ ������ 0 ~ 999 �̴�. */
		tCounter = TIM1_GetCounter();

		/* ���� �ð��� ���Ѵ�. */
		now = (rtimer_clock_t)((time_msb * (RTIMER_ARCH_PRESCALER+1)) + tCounter);

		/* ���� Ÿ�̸� �ð��� �����ͼ�, ���� �ð��� ���Ѵ�. */
		clock_to_wait = next_rtimer_time - now;

		/* ���� Clock Count�� ���Ѵ�.  */
		clock_to_remain = RTIMER_ARCH_PRESCALER-now;

		if(clock_to_wait <= clock_to_remain)
		{
			/*
			*  ���� Ÿ�̸��� ���� �ð��� ���� clock count�� �����÷ο� �Ǳ� ���̶��,
			*	Timer Compare Register�� �����Ѵ�.
			*/
			TIM1_UpdateCCR(clock_to_wait+tCounter);
		}
  }
}



/* Ÿ�̸� ī���Ͱ� CCRx �������� �� ������ ��� ȣ�� �� */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 &&
		htim->Instance == TIM1 && htim->Instance->CCR1 != 0)
  {
		ENERGEST_ON(ENERGEST_TYPE_IRQ);

		rtimer_run_next();

		ENERGEST_OFF(ENERGEST_TYPE_IRQ);
  }
}




void  rtimer_arch_init(void)
{
	TIM1_Init(RTIMER_ARCH_PRESCALER);

}


rtimer_clock_t rtimer_arch_now(void)
{

	return ((rtimer_clock_t)((time_msb * 1000) + TIM1_GetCounter()));
}


void rtimer_arch_schedule(rtimer_clock_t t)
{
	rtimer_clock_t now, clock_to_wait, clock_to_remain, tCaptureTime;

	next_rtimer_time = t;

	/* ĸ�ĵ� ī���͸� �����´�. */
	tCaptureTime = TIM1_GetCCR();

	/* ���� �ð��� ���Ѵ� */
	now = (rtimer_clock_t)((time_msb * (RTIMER_ARCH_PRESCALER+1)) + tCaptureTime);

	/* ���� Ÿ�̸� �ð��� �����ͼ�, ���� �ð��� ���Ѵ�. */
	clock_to_wait = t - now;

	/* ���� Clock Count�� ���Ѵ�. count�� ������ 0 ~ 999 �̴�. */
	clock_to_remain = RTIMER_ARCH_PRESCALER-now;

	/*
	*  ���� Ÿ�̸��� ���� �ð��� ���� clock count�� �����÷ο� �Ǳ� ���̶��,
	*	Timer Compare Register�� �����Ѵ�.
	*/
	if(clock_to_wait <= clock_to_remain)
	{
		/* We must now set the Timer Compare Register. */
		TIM1_UpdateCCR(clock_to_wait+tCaptureTime);
	}
	else
	{
		/* compare register will be set at overflow interrupt closer to the rtimer event. */
		TIM1_UpdateCCR(0);
	}
}

