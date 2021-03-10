#ifndef _MAC_H_
#define _MAC_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"
#include "net/rpl/rpl.h"

#include "net/ipv6/multicast/uip-mcast6-engines.h"

#include "rfm_def.h"
#include "ipc.h"
#include "mac.h"
#include "contiki.h"

#define MAX_PAYLOAD_LEN 120
#define MCAST_SINK_UDP_PORT 3001 /* Host byte order */
#define SEND_INTERVAL CLOCK_SECOND /* clock ticks */
#define ITERATIONS 100 /* messages */

/* Start sending messages START_DELAY secs after we start so that routing can
 * converge */
#define START_DELAY 60

static struct uip_udp_conn * mcast_conn;
static char buf[MAX_PAYLOAD_LEN];
static uint32_t seq_id;

/* Change this to switch engines. Engine codes in uip-mcast6-engines.h */
#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_ROLL_TM

/* For Imin: Use 16 over NullRDC, 64 over Contiki MAC */
#define ROLL_TM_CONF_IMIN_1         64

#undef UIP_CONF_IPV6_RPL
#undef UIP_CONF_ND6_SEND_RA
#undef UIP_CONF_ROUTER
#define UIP_CONF_ND6_SEND_RA         0
#define UIP_CONF_ROUTER              1
#define UIP_MCAST6_ROUTE_CONF_ROUTES 1

#undef UIP_CONF_TCP
#define UIP_CONF_TCP 0

/* Code/RAM footprint savings so that things will fit on our device */
#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#undef UIP_CONF_MAX_ROUTES
#define NBR_TABLE_CONF_MAX_NEIGHBORS  10
#define UIP_CONF_MAX_ROUTES           10

#endif



static void multicast_send(void);


