/**
 * \file
 *         Header file for the STM32F103-specific rtimer code
 * \author
 *         Simon Berg <ksb@users.sourceforge.net>
 */

#ifndef RTIMER_ARCH_H_
#define RTIMER_ARCH_H_

#include "sys/rtimer.h"

/* One tick: 1msec */
#define RTIMER_ARCH_PRESCALER (1000-1)

#define RTIMER_ARCH_SECOND 1000

void rtimer_arch_set(rtimer_clock_t t);

rtimer_clock_t rtimer_arch_now(void);

#endif /* RTIMER_ARCH_H_ */
