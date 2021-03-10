#include "rfm_def.h"
#include "rfm.h"

#include "contiki.h"

/**********************************************************************
*
*		Defined
*
**********************************************************************/
#define SET_WK_DET(p)					((p)->wkDet = 1)
#define CLEAR_WK_DET(p)					((p)->wkDet = 0)	
#define CHECK_WK_DET(p)					(((p)->wkDet == 1) ? 1 : 0)

#define SET_PR_DET(p, val)				((p)->prDet = val)
#define CLEAR_PR_DET(p)					((p)->prDet = 0)
#define CHECK_PR_DET(p)					(((p)->prDet >= 1) ? 1 : 0)
#define GET_PR_DET(p)					((p)->prDet)

#define DEBUG	0

#define WAKEUP_SYNC_CNT	27

#define PREAMBLE				0x2C1F5C20AB1A2B90
//#define IsSleepModeOn() ((ulp_flash_param.bSleepModeOn == 0) ? 0 : 1)


/**********************************************************************
*
*		Variable
*
**********************************************************************/
M_RFM_RX_DATA m_rxData;

//uint8_t rxDataBuf[RFM_DATA_BIT_LEN];


#if (DEBUG==1)
uint8_t debugDataBuffer[4096];
uint32_t rxCnt = 0;
#endif


/**********************************************************************
*
*		Extern
*
**********************************************************************/
extern _ULP_FLASH_PARAM ulp_flash_param;
extern uint8_t PN_1[8];
extern uint8_t PN_0[8];
extern bool IS_STANDBY_MODE;
extern bool IS_WAKEUP_MODE;
extern struct process rx_modem_process;
extern uint8_t rfm_preamble_cycle;
extern unsigned char preamble_buf[64]={0};
/**********************************************************************
*
*		RFM RX Driver Funcions
*
**********************************************************************/
void RFM_RX_Init(void)
{
	LED_Off(LED2);

	/* Initialize RX GPIO  */
	ULP_RX_GPIO_Init();

	/* Initialize RFM Resource */
	RFM_RX_Resource_Init();

	/*
	* StandBy 모드에서 Wake up으로 부팅이 된경우에는 ULP 초기화를 진행하지 않는다. 
	*/
	if(IS_WAKEUP_MODE == false)
	{
		/* Initialize Modem */
		if(RFM_RX_Modem_Init() == false)
		{
			printf("Fail RFM ULP RX Modem Setting\r\n");
			return;
		}
		
		/* Set RX Default Rester Value */
		if(RFM_Set_RxDefaultRegister() == false)
		{
			printf("Fail RFM ULP RX Register Setting\r\n");
			return;
		}

		WMO_Set_DataRate(ulp_flash_param.data_rate);

		printf("SET FIRENZE RFM RX MODE");

		printf("/ %d.%dKhz Data rate\n\n", (250 >> ulp_flash_param.data_rate), ((25000/(1<<ulp_flash_param.data_rate))%100));

	}


	/* Initialize RF Clock Interrupt */
	Configure_EXTI_RX_CLK();

///	if(IsSleepModeOn() == 0)
	{
		/* Initialize RF Wakeup Interrupt */
		Configure_EXTI_RX_WK();
	}

	RFM_RX_Start();
}




void RFM_RX_Resource_Init(void)
{
		M_RFM_RX_DATA *pData = &m_rxData;
	
		/* Initialize DATA management */
		pData->recvDataLen = 0; 								// data recieve length
		pData->dataIdx = 0; 										// data index
		pData->bufFull = false; 								// data buffer state
		pData->bitPos = 0;												// PN bit index
		pData->wkDet = 0; 												// wakeup detect
		pData->raw_recvDataLen = 0;
		memset(pData->buff, 0, RFM_RX_DATA_BIT_LEN_MAX*2); // data buffer
	
#if !defined(MULTIBIT_DECISON_IN_MODEM)	
		pData->matchCnt_pn0 = 0;
		pData->matchCnt_pn1 = 0;
#endif	
}


bool RFM_RX_Modem_Init(void)
{
	/* Set Wakeup modem Rester Value */
	if(WMO_Set_DefaultRegister() == false)
	{
		printf("Fail RFM Wakeup Modem Default Setting\r\n");
		return false;
	}
	
	/* Set Wakeup modem Rester Value */
	if(WMO_Set_Corr(SET) == false)
	{
		printf("Fail Modem Corr Setting\r\n");
		return false;
	}

	/* Set Wakeup modem Rester Value */
	WMO_ClearIntr();


	/* Set Wakeup modem Rester Value */
	if(WMO_Set_AlwaysOn(SET) == false)
	{
		printf("Fail Modem AlwaysOn Setting\r\n");
		return false;
	}

	/* Set Wakeup modem Threshold Value */
	if(WMO_Set_WakeupTH(ulp_flash_param.wk_th) == false)
	{
		printf("Fail Modem Threshold Setting\r\n");
		return false;
	}
}


void Configure_EXTI_RX_CLK(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = FRZ_ADC_CLK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(FRZ_RX_ADC_GPIO_Port, &GPIO_InitStruct);

	/* Enable and set EXTI lines 15 to 10 Interrupt to the highest priority */
	HAL_NVIC_SetPriority(FRZ_ADC_EXTI_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FRZ_ADC_EXTI_IRQn);
	FRZ_ADC_EXTI_LINE_DISABLE();
	
}


void Configure_EXTI_RX_WK(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = FRZ_RX_WK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(FRZ_WK_GPIO_Port, &GPIO_InitStruct);

	/* Enable WAKEUP lines  Interrupt to the highest priority */
    HAL_NVIC_SetPriority(FRZ_WK_EXTI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FRZ_WK_EXTI_IRQn);

}

void RFM_WAKEUP_IRQHANDLER(void)
{
	M_RFM_RX_DATA *pData = &m_rxData;

	if(__HAL_GPIO_EXTI_GET_IT(FRZ_RX_WK_Pin) != RESET)
	{
		LED_On(LED3);

		/* Disable wake-up Interrupt */
		RFM_RX_WK_Stop();

		if(CHECK_WK_DET(pData))
		{
			printf("[WK_DET] BUSY\n");
			__HAL_GPIO_EXTI_CLEAR_IT(FRZ_RX_WK_Pin);

			return;
		}

		//printf("[WK_DET]ON\n");

		/* Set WK_UP Detect Flag */
		SET_WK_DET(pData);


    	/* Enable ADC Rx Interrupt */
		RFM_RX_ADC_Start();
		__HAL_GPIO_EXTI_CLEAR_IT(FRZ_RX_WK_Pin);

	}
}


uint8_t GetBitsCount(uint64_t x)
{
	x = x - ((x >> 1) & 0x5555555555555555);
	x = (x & 0x3333333333333333) + ((x >> 2) & 0x3333333333333333);
	x = (x + (x >> 4)) & 0x0F0F0F0F0F0F0F0F;

	x = x + (x >> 8);
	x = x + (x >> 16);
	x = x + (x >> 32);
	return x & 0xFF;
}

void RFM_RX_IRQHANDLER(void)
{
	M_RFM_RX_DATA *pData = &m_rxData;
	static int recv_counter = 0;

	uint8_t adcData = 0x0;

	if(__HAL_GPIO_EXTI_GET_IT(FRZ_RX_WK_Pin) != RESET)
	{
		LED_Toggle(LED3);

		/* Disable wake-up Interrupt */
		RFM_RX_WK_Stop();

		if(CHECK_WK_DET(pData))
		{
			printf("[WK_DET] BUSY\n");
			__HAL_GPIO_EXTI_CLEAR_IT(FRZ_RX_WK_Pin);

			return;
		}

//		printf("[WK_DET]ON\n");

		/* Set WK_UP Detect Flag */
		SET_WK_DET(pData);


		/* Enable ADC Rx Interrupt */
		RFM_RX_ADC_Start();
		__HAL_GPIO_EXTI_CLEAR_IT(FRZ_RX_WK_Pin);

	}
	else if(__HAL_GPIO_EXTI_GET_IT(FRZ_ADC_CLK_Pin) != RESET)

//	if(__HAL_GPIO_EXTI_GET_IT(FRZ_ADC_CLK_Pin) != RESET)
	{
		//LED_ON(LD2_Pin);

		/* RFM recieve RX_DATA from ADC 5pin */ 
		uint32_t gpioData = 0x0;
		uint16_t rxData = 0x0;

		/******************************************************************************************************
		*	Get data state : if wakeup detected, data is saved to data buffer untill rx data size
		*  
		*  - 1bit receive data saved in 1byte rx buffer 
		*******************************************************************************************************/

		if(pData->dataIdx++ < WAKEUP_SYNC_CNT)
		{
			//LED_Toggle(LED2);
			__HAL_GPIO_EXTI_CLEAR_IT(FRZ_ADC_CLK_Pin);
			return;
		}


			/* Recieve next data after syncronization.			*/
		if(pData->dataIdx % OVERSAMPLE_CNT == 0)
		{
			//LED_On(LED1);

			/******************************************************************************************************
			* Recieve 5bit RX DATA and get adc value
			*******************************************************************************************************/
			/* Decision the ADC data using only most significant bit */
			if(HAL_GPIO_ReadPin(FRZ_RX_DATA_GPIO_Port, FRZ_RX_DATA4_Pin) > 0) rxData = 0x10;
			if(HAL_GPIO_ReadPin(FRZ_RX_DATA_GPIO_Port, FRZ_RX_DATA3_Pin) > 0) rxData += 0x08;
			if(HAL_GPIO_ReadPin(FRZ_RX_DATA_GPIO_Port, FRZ_RX_DATA2_Pin) > 0) rxData += 0x04;
			if(HAL_GPIO_ReadPin(FRZ_RX_DATA_GPIO_Port, FRZ_RX_DATA1_Pin) > 0) rxData += 0x02;
			if(HAL_GPIO_ReadPin(FRZ_RX_DATA_GPIO_Port, FRZ_RX_DATA0_Pin) > 0) rxData += 0x01;
			//if(HAL_GPIO_ReadPin(FRZ_RX_SLROUT_GPIO_Port, FRZ_RX_SLROUT_Pin) > 0) rxData = 0x01;

			//printf("%02X,", rxData);

#if defined(MULTIBIT_DECISON_IN_MODEM)
			/* Add buffer to multiBitData */
			pData->buff[pData->recvDataLen] += rxData;
#else
			/* Compare PN Pattern0 or PN Pattern1. Then increase PN counter*/
			if(rxData == PN_0[pData->bitPos])
			{
				pData->matchCnt_pn0++;
			}
			else
			{
				pData->matchCnt_pn1++;
			}
#endif
			
			/* Decision '1' or '0' from PN counter	*/				
			if(pData->bitPos == (ulp_flash_param.pn_len-1))
			{
#if defined(MULTIBIT_DECISON_IN_MODEM)
//				printf("%2x\t", pData->buff[pData->recvDataLen]);

				pData->recvDataLen++;
				pData->raw_recvDataLen++;

#else
				pData->buff[pData->recvDataLen++] =
					((pData->matchCnt_pn0 > pData->matchCnt_pn1) ? 0x0 : 0x1);

				pData->matchCnt_pn0 = 0;
				pData->matchCnt_pn1 = 0;
#endif

				if(ulp_flash_param.duty_cycle && !CHECK_PR_DET(pData))
				{
					uint64_t tmp;
					uint8_t sum = 0;

					pData->preamble += ((pData->buff[pData->recvDataLen-1] >= ulp_flash_param.decision_val) ? 0x01 : 0x00);

					tmp = ~(pData->preamble^PREAMBLE);

				/*	for(int i= 0; i<64; i++)
					{
						if((tmp >> i) & 0x01)
							sum++;
					}
*/
					//if(sum > 48)
					if(GetBitsCount(tmp) > 48)
					//if(pData->preamble == PREAMBLE)
					{
						//temp_preamble = 0;
						SET_PR_DET(pData, pData->recvDataLen);
						pData->bitPos = 0;
						__HAL_GPIO_EXTI_CLEAR_IT(FRZ_ADC_CLK_Pin);
						return;
					}
#if(1)
					if(ulp_flash_param.pn_len == 1 && pData->recvDataLen >= 64)
					{
						pData->recvDataLen--;
						memcpy((void*)&pData->buff[0], (void*)&pData->buff[1], pData->recvDataLen);

						pData->buff[pData->recvDataLen] = 0;
					}
#endif
					pData->preamble <<= 1;
				}

				pData->bitPos = 0;
#if(1)
				if(ulp_flash_param.pn_len == 1 && pData->raw_recvDataLen == RFM_RX_DATA_BIT_RAW_LEN_MAX && !CHECK_PR_DET(pData))
				{
					printf("FAIL: PREMBLE = %d\n", GET_PR_DET(pData));

					RFM_RX_ADC_Stop();
					/* Set data buffer full flag */
					pData->bufFull = true;

					/* Print RX data */
					RFM_RX_PrintData(pData->buff, (8+24)*8, 0);

				}else
#endif
				if(!ulp_flash_param.duty_cycle && pData->recvDataLen == RFM_DATA_BIT_LEN)
				{
					/* Stop RX Interrupt untill modem get data */
					RFM_RX_ADC_Stop();

					/* Set data buffer full flag */
					pData->bufFull = true;

					// Send payload to Modem Layer
					//process_post_synch(&rx_modem_process, PROCESS_EVENT_MSG, NULL);

					/* Print RX data */
					RFM_RX_PrintData(pData->buff, RFM_DATA_BIT_LEN, 0);

				}
				else if(pData->recvDataLen == GET_PR_DET(pData) + RFM_RX_DATA_BIT_LEN && CHECK_PR_DET(pData))
				{
					SET_PR_DET(pData, GET_PR_DET(pData) - RFM_PRM_BIT_LEN);

					//printf("PREMBLE = %d\n", GET_PR_DET(pData));
					/* Stop RX Interrupt untill modem get data */
					RFM_RX_ADC_Stop();

					/* Set data buffer full flag */
					pData->bufFull = true;
					// Send payload to Modem Layer
					//process_post_synch(&rx_modem_process, PROCESS_EVENT_MSG, NULL);

					/* Print RX data */
					RFM_RX_PrintData(pData->buff, pData->recvDataLen, 0);
				}
				else if(pData->recvDataLen == RFM_RX_DATA_BIT_LEN_MAX)
				{
					//printf("FAIL: PREMBLE = %d\n", GET_PR_DET(pData));

					RFM_RX_ADC_Stop();
					/* Set data buffer full flag */
					pData->bufFull = true;

					/* Print RX data */
					RFM_RX_PrintData(pData->buff, pData->recvDataLen, 0);
				}
			}
			else
			{
				(pData->bitPos++);

			}

		}
		else
		{
			// No Processing
		}

#if (DEBUG==1)
		if(rxCnt == 4096)
		{
			RFM_RX_ADC_Stop();

			for(uint16_t i = 0 ; i < 4096; i++)
			{
				if(i != 0 && (i % 4 == 0)) printf("\r\n");
				printf("%02d ", debugDataBuffer[i]);
			}

			printf("\n*********************\n");
			memset(debugDataBuffer, 0, 4096);

			rxCnt = 0;
			
			/* Clear WK flag */
			CLEAR_WK_DET(pPr);

			/* Restart WK interrupt */
			RFM_RX_WK_Start();
		}
#endif		

		__HAL_GPIO_EXTI_CLEAR_IT(FRZ_ADC_CLK_Pin);
		//LED_Off(LED3);
		//LED_Off(LED2);
	 	//LED_Off(LED1);
		return;

	}
	else if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
	{
		 LED_Toggle(LED2);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
	}

}

BOOL RFM_RX_GetDataBufState(void)
{
	return m_rxData.bufFull;
}


//int16_t RFM_RX_GetData(uint8_t *input)
int16_t RFM_RX_GetData(uint16_t *input)
{
	M_RFM_RX_DATA *pData = &m_rxData;

	if(pData->bufFull == false)
	{
		return 0;
	}
	else
	{
		//memcpy(input, &pData->buff[pData->prDet], RFM_DATA_BIT_LEN);
		memcpy(input, &pData->buff[pData->prDet], (pData->recvDataLen-pData->prDet)*2);
	}

	return pData->recvDataLen-pData->prDet;
}

void RFM_RX_PrintData(uint16_t *data, uint64_t len, uint16_t th)
{
	uint64_t i;
	const uint16_t decisionVal = ulp_flash_param.decision_val;// 45;
	extern bool IS_STOP_MODE;

	uint16_t temp =0;
	printf("\n\nRecieve data from Sender[Len:%d/TH:%d] =\n", len, th);

#if defined(MULTIBIT_DECISON_IN_MODEM)
#if (0)		// printf raw data
	for(i = 0 ; i < len; i++)
	{
		temp += data[i];
		if(i != 0 && (i % 8 == 0)) printf(" ");
		if(i != 0 && (i % 64 == 0))
		{
			printf(" [%3d:", temp);
			if(temp>decisionVal) printf("1]");
			else 		printf("0]");
			printf("\n");

			temp = 0;
		}
		printf("%d ", data[i]);
	}				
	printf("\n");
#else		// print Correction data

	uint16_t buf[RFM_RX_DATA_BIT_LEN_MAX] = {0x0,};
	uint16_t j = 0;


	for(i = 0; i < len; i++)
	{
		if(data[i] >= decisionVal)		temp = 0x1;
		else												temp = 0x0;

		buf[j] |= (temp << (7 - (i % 8)));

		if((7 - (i % 8)) == 0)
		{
			if(j % 8 == 0 && j != 0)
			{
				printf(" [%d, %d]\n", i, j);
			}

			printf("0x%03x ", buf[j++]);

			if(i == len-1)
			{
				printf("\n");
			}
		}
	}
#endif

#else
	uint8_t buf[RFM_RX_DATA_BIT_LEN_MAX] = {0x0,};
	uint16_t j = 0;

	for(i = 0; i < len; i++)
	{
		buf[j] |= (data[i] << (7 - (i % 8)));

		/*if((7 - (i % 8)) == 0)
		{
			if(j % 8 == 0 && j != 0)
			{
				printf(" [%d, %d]\n", i, j);
			}

			printf("0x%02x ", buf[j++]);		

			if(i == len-1)
			{
				printf("\n");
			}
		}*/
	}
	printf("END\n ");

#endif
}


void RFM_RX_Start(void)
{
	M_RFM_RX_DATA *pData = &m_rxData;
	
	pData->bufFull = false;
	pData->dataIdx = 0;
	pData->recvDataLen = 0;
	pData->bitPos = 0;
	pData->preamble = 0;

#if !defined(MULTIBIT_DECISON_IN_MODEM)	
	pData->matchCnt_pn0 = 0;
	pData->matchCnt_pn1 = 0;
#endif	

	pData->raw_recvDataLen = 0;
	memset(pData->buff, 0, RFM_RX_DATA_BIT_LEN_MAX*2);


	/* Clear wakeup modem interrupt  */
	WMO_ClearIntr();
	
	CLEAR_PR_DET(pData);
	/* Start rx wakeup */
	CLEAR_WK_DET(pData);
	RFM_RX_WK_Start();

}


void RFM_RX_WK_Start(void)
{
	FRZ_WK_EXTI_LINE_ENABLE();
}

void RFM_RX_WK_Stop(void)
{
	FRZ_WK_EXTI_LINE_DISABLE();
}


void RFM_RX_ADC_Start(void)
{
	FRZ_ADC_EXTI_LINE_ENABLE();
}

void RFM_RX_ADC_Stop(void)
{
	FRZ_ADC_EXTI_LINE_DISABLE();
}


void RFM_RX_Stop(void)
{
	FRZ_WK_EXTI_LINE_DISABLE();
	FRZ_ADC_EXTI_LINE_DISABLE();
}



