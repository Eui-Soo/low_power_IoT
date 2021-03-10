

#include "modem.h"
#include "rfm_def.h"
#include "ipc.h"
#include "modem.h"
#include "contiki.h"

extern uint16_t ferCounter;
extern uint16_t ferTryCounter;
extern bool ferTestOn;
extern uint8_t ferDuration;
extern const uint8_t PEAMBLE_PATTERN[8];

extern struct process ULP_process;

extern _ULP_FLASH_PARAM ulp_flash_param;

/**********************************************************************
*
*		MODEM MAIN THREAD
*
**********************************************************************/

PROCESS(tx_modem_process, "tx_modem_procedure");
PROCESS(fer_process, "fer_procedure");
PROCESS(rx_modem_process, "rx_modem_procedure");


PROCESS_THREAD(tx_modem_process, ev, data)
{
	IPC_PKT *ipc = (IPC_PKT *)data;
	
	unsigned char payload[RFM_DATA_LEN/2] = {0};
	unsigned char RF_data[RFM_DATA_LEN] = {0};
	unsigned char payload_len = 0;
	extern bool ferTestOn;
	
	PROCESS_BEGIN();

	while(1)
	{	
		/* Wait untill recieved event from MAC Layer */
		PROCESS_WAIT_EVENT();

		if(ferTestOn == true)
		{
			continue;
		}

		/* Get payload length from IPC length field */
		payload_len = IPC_GET_LEN(ipc);

		/* Copy payload  from IPC databuffer */
		memcpy(payload, IPC_GET_DATA_PTR(ipc), IPC_GET_LEN(ipc));

#if 0
		printf("[MODEM] payload_len = %d\n", payload_len);
		// To check the data from application layer
		for (int i = payload_len-20 ; i<payload_len; i++)
		{
			printf(" payload[%d] = %d\n", i, payload[i]);
		}
#endif


		/* Processing Modem */
		Modem_EncPayload(payload, payload_len, RF_data);


		/* Write payload to PHY send buffer*/
		//printf("[MODEM] RF send start\n");
	  while(RFM_TX_Busy_Check());
	  printf("\n[MODEM] Ready to send data\n");

	  if(ulp_flash_param.pn_len != 1)
	  {
		  RFM_TX_Buff_In(RF_data,RFM_DATA_LEN);
	  }
	  else
	  {
		  RFM_TX_Buff_In(RF_data,24+RFM_PRM_LEN);
	  }
	}

	PROCESS_END();
}





PROCESS_THREAD(fer_process, ev, data)
{
	static struct etimer e;


#if 0
	uint8_t ferData[RFM_DATA_LEN] = 
	{
		0x37,  0xc8,  0x97,  0xf4,  0x9c,  0x37,  0x23,  0x0c,
		0x49,  0xbf,  0x84,  0xff,  0xff,  0xff,  0x19,  0x70,
		0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,
		0x00,  0x00,  0x00,  0xe8,  0x0f,  0xe4,  0x4b,  0x78,
		0xa0,  0x31,  0x12,  0x70
	};
#else

	uint8_t ferData[RFM_DATA_LEN] =
	{
		0x2c,  0x1f,  0x5c,  0x20,  0xab,  0x1a,  0x2b,  0x90,
		0xcb,  0x33,  0x0d,  0x52,  0xcb,  0xf1,  0x4b,  0x09,
		0x01,  0x96,  0x86,  0x8e,  0x1e,  0x73,  0x17,  0x1a,
		0x0e,  0xf6,  0x1d,  0xc2,  0x90,  0x0d,  0x10,  0xf5,
		0x9e,  0xf1,  0x99,  0xc9,  0x8e,  0x5c,  0x85,  0x17,
		0x19,  0x20,  0x80,  0xbe,  0x97,  0x2d,  0x96,  0xd5,
		0x9b,  0x0d,  0x9d,  0x34,  0x09,  0x2d,  0x13,  0x77
	};

	uint8_t ferData2[8+24] =
	{
		0x2c,  0x1f,  0x5c,  0x20,  0xab,  0x1a,  0x2b,  0x90,
		0x81,  0x00,  0x00,  0x00,  0x01,  0x02,  0x03,  0x04,
		0x05,  0x06,  0x07,  0x08,  0x09,  0x0A,  0x0B,  0x0C,
		0x0D,  0x0E,  0x0F,  0x10,  0x11,  0x12,  0x13,  0x14
	};
#endif
	
	PROCESS_BEGIN();

	while(1)
	{
		if(ferTryCounter == 0)
		{
			printf("FER Test Start : %d times in %d second cycle\n\n", ferCounter, ferDuration);
		}
		
		if((ferTryCounter) >= ferCounter)
		{
			continue;
		}
	
		/* Write payload to PHY send buffer*/
	  while(RFM_TX_Busy_Check());
	  printf("FER Data : %03d....", ++ferTryCounter);

	  if(ulp_flash_param.pn_len != 1)
	  {
		  if(ulp_flash_param.duty_cycle)
			  RFM_TX_Buff_In(ferData, RFM_DATA_LEN);
		  else
			  RFM_TX_Buff_In(&ferData[8], RFM_DATA_LEN-8);
	  }
	  else
	  {
		  if(ulp_flash_param.duty_cycle)
			  RFM_TX_Buff_In(ferData2, 24+8);
		  else
			  RFM_TX_Buff_In(&ferData2[8], 24);
	  }
		etimer_set(&e, CLOCK_SECOND*ferDuration);
		PROCESS_WAIT_UNTIL(etimer_expired(&e));
	}

	PROCESS_END();
}


PROCESS_THREAD(rx_modem_process, ev, data)
{	
	PROCESS_BEGIN();

	while(1)
	{
		while(RFM_RX_GetDataBufState()==false);

		ULP_rx_interrupt();

		/* Send payload to MAC Layer */
		process_post_synch(&ULP_process, PROCESS_EVENT_MSG, NULL);				

#if 0		
		/* Start RX WK Interrupt  */
		printf("Start WK Status\n");	

		RFM_RX_WK_Start();
#endif	

	}

	PROCESS_END();
}




/**********************************************************************
*
*		Encoding payload received from MAC & Send to PHY Layer
*
**********************************************************************/

void Modem_EncPayload(uint8_t* payload, uint8_t payload_len, uint8_t* RF_data)
{
	int i;
  unsigned char output[SIZE*2] = {0}; // 부호화 data 변수
  unsigned char output_enc_prm[SIZE*2 + 64] = {0}; // 부호화 data 변수
  unsigned char output_enc[SIZE*2]={0}; // 확산 data 변수
  unsigned char sig[SIZE]={0};
  unsigned char payload_bits[payload_len*8]; // packet을 bits 단위로 저장한 변수
  unsigned char preamble_bits[64];
  unsigned char output_sp[SIZE*2*SG + (64*SG)]={0};

	/* 상위 Layer에서 받은 data 확인하는 print문 */
	printf("\n[Modem] Recieved Data from MAC = [%d]\n", payload_len);
	
	for(i=0;i<payload_len;i++)
	{
		printf("0x%02x",payload[i]);

		if((i+1)%8 == 0)
			printf("\n");
		else
			printf("  ");
	}
	printf("\n");
	
	/* bytes 단위를 bites 단위를 바꿔주는 부분  */
	ArraygetAbit(payload, payload_len, payload_bits);


	/* payload_bits 확인하는 부분 */
	for (int i = 0; i < payload_len*8; i++)
	{
		 sig[i] = payload_bits[i];
	}

	printf("\nModem buffer = \n");
	for(i=0;i<payload_len*8;i++)
	{
		printf("%d",sig[i]);

		if((i+1)%8 == 0)
		{
		  if((i+1)%64 == 0)
			  printf("\n");
		  else
			  printf("  ");
		}
	}

	printf("\n");

	if(ulp_flash_param.pn_len != 1)
	{
		/* 부호화 과정 및 확인 과정  */
		FEC_enc(sig, output);

		printf("\nFEC_enc_data = \n");

		for(i=0;i<payload_len*8*2;i++)
		{
			printf("%d",output[i]);

			if((i+1)%8 == 0)
			{
			  if((i+1)%64 == 0)
				  printf("\n");
			  else
				  printf("  ");
			}
		}
		printf("\n");

		/* �솗�궛 怨쇱젙 諛� �솗�씤 怨쇱젙 */
		interleaver(output, output_enc);

		printf("\nInterleaver Data = \n");

		for(i=0;i<payload_len*8*2;i++)
		{
			printf("%d",output_enc[i]);

			if((i+1)%8 == 0)
			{
			  if((i+1)%64 == 0)
				  printf("\n");
			  else
				  printf("  ");
			}
		}

	}
	else
	{

		/* payload_bits 확인하는 부분 */
		for (int i = 0; i < payload_len*8; i++)
		{
			output_enc[i] = sig[i];
		}
	}

	if(ulp_flash_param.duty_cycle)
	{

		for(i=0;i<payload_len*8*2;i++)
		{
			output_enc_prm[i+64] = output_enc[i];
		}
		/* bytes 단위를 bites 단위를 바꿔주는 부분  */
		ArraygetAbit(PEAMBLE_PATTERN, 64, preamble_bits);

		for(i=0; i<64; i++)
		{
			output_enc_prm[i] = preamble_bits[i];
		}

		printf("\nInterleaver Data with Preamble = \n");

		for(i=0;i<(payload_len*8*2)+64;i++)
		{
			printf("%d",output_enc_prm[i]);

			if((i+1)%8 == 0)
			{
			  if((i+1)%64 == 0)
				  printf("\n");
			  else
				  printf("  ");
			}
		}

		//Spreading(output_enc, &output_sp[64]);
		Spreading(output_enc_prm, output_sp);


		printf("\nSpreading_data = \n");


		for(i=0;i<(payload_len*8*2*SG)+(64*SG);i++)
		{
			printf("%d",output_sp[i]);

			if((i+1)%8 == 0)
			{
			  if((i+1)%64 == 0)
				  printf("\n");
			  else
				  printf("  ");
			}
		}

		printf("\n");

		/* bits 단위 bytes 단위로 바꿔주는 부분 및 확인 과정 */
		ArraygetAHex(output_sp,payload_len*8*2*SG+(64*SG),RF_data);
		printf("\nSend data to RF chip from Modem = \n");

		for(i=0;i<payload_len*2*SG+(8*SG);i++)
		{
			printf("0x%02x ",RF_data[i]);

			if((i+1)%8 == 0)
				  printf("\n");
			else
				  printf("  ");

		}
		printf("\n");

	}
	else
	{
		Spreading(output_enc,output_sp);
		printf("\nInterleaver_data = \n");

		for(i=0;i<payload_len*8*2*SG;i++)
		{
			printf("%d",output_sp[i]);

			if((i+1)%8 == 0)
			{
			  if((i+1)%64 == 0)
				  printf("\n");
			  else
				  printf("  ");
			}
		}

		printf("\n");

		// packet preamble add


		/* bits 단위 bytes 단위로 바꿔주는 부분 및 확인 과정 */
		ArraygetAHex(output_sp,payload_len*8*2*SG,RF_data);
		printf("\nSend data to RF chip from Modem = \n");

		for(i=0;i<payload_len*2*SG;i++)
		{
			printf("0x%02x ",RF_data[i]);

			if((i+1)%8 == 0)
				  printf("\n");
			else
				  printf("  ");

		}
		printf("\n");

	}


}

