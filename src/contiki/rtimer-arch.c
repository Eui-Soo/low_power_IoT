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


/* 타이머 카운터가 오버플로우 되는 경우 호출 됨 : 1msec */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1 && (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) !=RESET))
  {
		rtimer_clock_t now, clock_to_wait, clock_to_remain, tCounter;

		time_msb++;

		/* 현재 타이머 카운터를 가져온다. 카운터의 범위는 0 ~ 999 이다. */
		tCounter = TIM1_GetCounter();

		/* 현재 시간을 구한다. */
		now = (rtimer_clock_t)((time_msb * (RTIMER_ARCH_PRESCALER+1)) + tCounter);

		/* 다음 타이머 시간을 가져와서, 남은 시간을 구한다. */
		clock_to_wait = next_rtimer_time - now;

		/* 남은 Clock Count를 구한다.  */
		clock_to_remain = RTIMER_ARCH_PRESCALER-now;

		if(clock_to_wait <= clock_to_remain)
		{
			/*
			*  다음 타이머의 남은 시간이 현재 clock count가 오버플로우 되기 전이라면,
			*	Timer Compare Register를 설정한다.
			*/
			TIM1_UpdateCCR(clock_to_wait+tCounter);
		}
  }
}



/* 타이머 카운터가 CCRx 레지스터 와 동일한 경우 호출 됨 */
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

	/* 캡쳐된 카운터를 가져온다. */
	tCaptureTime = TIM1_GetCCR();

	/* 현재 시간을 구한다 */
	now = (rtimer_clock_t)((time_msb * (RTIMER_ARCH_PRESCALER+1)) + tCaptureTime);

	/* 다음 타이머 시간을 가져와서, 남은 시간을 구한다. */
	clock_to_wait = t - now;

	/* 남은 Clock Count를 구한다. count의 범위는 0 ~ 999 이다. */
	clock_to_remain = RTIMER_ARCH_PRESCALER-now;

	/*
	*  다음 타이머의 남은 시간이 현재 clock count가 오버플로우 되기 전이라면,
	*	Timer Compare Register를 설정한다.
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

