
#include "common.h"
#include "app.h"
#include "ipc.h"
#include "clock.h"
#include "contiki.h"
#include "rfm.h"
#include "rfm_def.h"

/**********************************************************************
*
*		Defined
*
**********************************************************************/



/**********************************************************************
*
*		Variable
*
**********************************************************************/

/**********************************************************************
*
*		Extern
*
**********************************************************************/
extern _ULP_FLASH_PARAM ulp_flash_param;
bool appProcessRun = FALSE;
/**********************************************************************
*
*		Funcions
*
**********************************************************************/
PROCESS(tx_app_process, "tx_application_procedure");
//AUTOSTART_PROCESSES(&tx_app_process);


PROCESS_THREAD(tx_app_process, ev, data)
{
	extern struct process mac_process;
	extern bool ferTestOn;
	
	IPC_PKT ipc;
	int latitude, longitude;
	
	
	unsigned static char testPayLoad[20] = 
	{
			0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 
			0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14
	};
	
	unsigned static char testPayLoadLen = 20;
	static struct etimer etimer;
	uint8_t i;
	
	PROCESS_BEGIN();

	while(1)
	{
		if(ferTestOn == true)
		{
			return 0;
		}

		if(appProcessRun == TRUE)
		{
			memset(ipc.data, 0, sizeof(ipc.data));
			
			/* Set payload length to IPC length field*/
			IPC_SET_LEN(&ipc, testPayLoadLen);

			printf("\n\n**************************************************************\n");
			


			/* Copy payload to IPC data buffer */
			memcpy(ipc.data, testPayLoad, testPayLoadLen);

			printf("[APP] payload_len = %d bytes \n", ipc.hdr.msg_len);
			
			for(i = 0; i < ipc.hdr.msg_len; i++)
			{
				printf("0x%02x", ipc.data[i]);

				if((i + 1) % 8 == 0)	printf("\n");
				else								printf(" ");
			}
			printf("\n");
		
			// Send payload to MAC Layer
			process_post_synch(&mac_process, PROCESS_EVENT_MSG, &ipc);
		}

		etimer_set(&etimer, CLOCK_SECOND * 5);	// 5 sec
		PROCESS_WAIT_UNTIL(etimer_expired(&etimer));
	}

	PROCESS_END();
}

PROCESS(rx_app_process, "rx_application_procedure");
PROCESS_THREAD(rx_app_process, ev, data)
{
	IPC_PKT *ipc = (IPC_PKT *)data;

	unsigned static char testPayLoad[20] = 
	{
			0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 
			0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14
	};
	unsigned char payload[128] = {0};
	unsigned char payload_len = 0;
	char oledbuffer[31] = { 0, };
	unsigned char i;
	uint32_t temp;
	
	PROCESS_BEGIN();

	if(ulp_flash_param.sleep)
	{
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		SystemClock_Config();
	}

	while(1)
	{
		/* Wait untill recieved event from MAC Layer */
		PROCESS_WAIT_EVENT();

		/* Get payload length from IPC length field */
		payload_len = IPC_GET_LEN(ipc);

		/* Copy payload  from IPC databuffer */
		memcpy(payload, IPC_GET_DATA_PTR(ipc), IPC_GET_LEN(ipc));

		/* Processing App */
		printf("[APP] Recieved Payload to MAC = %d\r\n", payload_len);
		for(i = 0 ; i < payload_len; i++)
		{
			printf("0x%02x ", ipc->data[i]);

			if((i + 1) % 8 == 0)	printf("\n");
			else								printf("  ");
		}
		printf("\n");

		if(!memcmp(ipc->data, testPayLoad, 20))
		{
			printf("\nRecieved test payload from TX\n");
	
#if 0			
		  sprintf(oledbuffer, "%d %d %d %d %d %d %d %d %d %d", ipc->data[0], ipc->data[1], ipc->data[2], ipc->data[3], ipc->data[4],
				ipc->data[5], ipc->data[6], ipc->data[7], ipc->data[8], ipc->data[9]);
		  oled_string(1,0, oledbuffer);
#endif			
		}


	}

	PROCESS_END();
}


void App_Init(void)
{

}





