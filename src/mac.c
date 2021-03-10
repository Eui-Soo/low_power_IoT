#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

#include "net/rpl/rpl-private.h"
//#include "net/ipv6/multicast/uip-mcast6.h"
#include "lib/random.h"
#include "sys/ctimer.h"


#include "net/ip/uip-debug.h"

#include <string.h>

#include "net/ip/uip-debug.h"
#include "net/rpl/rpl.h"

#include "app.h"
#include "ipc.h"
#include "clock.h"
#include "contiki.h"
//#include "gps.h"
#include "main.h"
//#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* ULP radio driver */
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "dev/watchdog.h"

#include "dev/leds.h"

#include "modem.h"

#include "net/rime/rime.h"



#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
//#include "ULP.h"
#include "ipc.h"
#include "process.h"
#include "rfm_def.h"
#include "rfm.h"

#include "mac.h"

#define DEBUG DEBUG_NONE

/**********************************************************************
*
*		For TX
*
**********************************************************************/
//static struct uip_udp_conn * mcast_conn;

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  printf("broadcast message received from %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/**********************************************************************
*
*		For RX
*
**********************************************************************/

//#define MCAST_SINK_UDP_PORT 3001 /* Host byte order */
//#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

/*
#if !NETSTACK_CONF_WITH_IPV6 || !UIP_CONF_ROUTER ||  !UIP_CONF_IPV6_MULTICAST || !UIP_CONF_IPV6_RPL
#error "This example can not work with the current contiki configuration"
#error "Check the values of: NETSTACK_CONF_WITH_IPV6, UIP_CONF_ROUTER, UIP_CONF_IPV6_RPL"
#endif
*/



PROCESS(mac_process, "mac_procedure");
/*AUTOSTART_PROCESSES(&mac_process);
*/

PROCESS_THREAD(mac_process, ev, data)
{

	IPC_PKT *ipc = (IPC_PKT *)data;

	unsigned char payload[128] = {0};
	unsigned char payload_len = 0;

	extern bool ferTestOn;

	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();

	broadcast_open(&broadcast, 129, &broadcast_call);

    //prepare_mcast();

	while(1)
	{

		// Wait untill recieved event from App Layer
		PROCESS_WAIT_EVENT();

		// Get payload length from IPC length field
		payload_len = IPC_GET_LEN(ipc);
		//printf("[MAC] payload_len = %d bytes\n", payload_len);

		// Copy payload  from IPC databuffer
		memcpy(payload, IPC_GET_DATA_PTR(ipc), IPC_GET_LEN(ipc));

#if 0
		// Check whether the contents of packet of application layer is identical
		for (int i = 0 ; i<payload_len; i++)
		{
			printf("%d \n", payload[i]);
		}
#endif

		if(ferTestOn == true)
		{
			return 0;
		}

		// Send payload to transport layer
		packetbuf_copyfrom(payload, payload_len);
		broadcast_send(&broadcast);

	}

	PROCESS_END();
}


/*

{
	PROCESS_BEGIN();

	while(1)
	{

		PROCESS_WAIT_EVENT();


		payload_len = IPC_GET_LEN(ipc);


		memcpy(payload, IPC_GET_DATA_PTR(ipc), IPC_GET_LEN(ipc));

		printf("[MAC] payload_len = %d\n", payload_len);


		printf("[MAC] Recieved Payload to MAC = \r\n");
		for(i = 0 ; i < payload_len; i++)
		{
			printf("%0x02x ", ipc.data[i]);
		}
		printf("\n");

		process_post_synch(&app_process, PROCESS_EVENT_MSG, &ipc);
	}

	PROCESS_END();
}
#endif
*/



/**********************************************************************
*
*		Functions
*
**********************************************************************/











