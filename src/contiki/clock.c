#include <stm32h7xx.h>
#include <sys/clock.h>
#include <sys/cc.h>
#include <sys/etimer.h>

static volatile clock_time_t current_clock = 0;
static volatile unsigned long current_seconds = 0;
static unsigned int second_countdown = CLOCK_SECOND;

void HAL_SYSTICK_Callback(void)
{
	/*
	*  sys tick duration : 1msec
	*/
	SCB->ICSR = SCB_ICSR_NMIPENDSET_Pos;
	current_clock = HAL_GetTick();

	if( etimer_pending() && etimer_next_expiration_time() <= current_clock )
	{
		etimer_request_poll();
		 //printf("%d,%d\n", clock_time(),etimer_next_expiration_time());
	}

	/*
	*  second clock duration : 10msec
	*/
	if(--second_countdown == 0)
	{
		current_seconds++;
		second_countdown = CLOCK_SECOND;
	}
}

void clock_init()
{

}

clock_time_t clock_time(void)
{
    return current_clock;
}

unsigned long clock_seconds(void)
{
    return current_seconds;
}
