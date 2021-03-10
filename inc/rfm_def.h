#ifndef _RFM_DEF_H_
#define _RFM_DEF_H_

#include "main.h"
#include "common.h"
#include "flash_if.h"
#include "rfm.h"
/**************************************
*
*			Defined	
*
***************************************/

#define BYTE_TO_BIT					8


#define OVERSAMPLE_CNT			4


#define RFM_PREAMBLE_CYCLE_MAX			15//102

/* 	PREAMBLE field		*/
#define RFM_PREAMBLE_LEN	512						// 512byte, 4096bit


#define RFM_PRM_LEN			8
#define RFM_PRM_BIT_LEN		((RFM_PRM_LEN)*(BYTE_TO_BIT))

/*		DUMMY field		*/
#define RFM_DUMMY_DATA		0x00
#define RFM_DUMMY_LEN			1

/*		SYNC FIELD		*/
#define RFM_SYNC_DATA					0xDB6F
#define SYNC_LEN						2
#define SYNC_BIT_LEN					((SYNC_LEN)*(BYTE_TO_BIT))


/*		DATA FIELD		*/
#define RFM_DATA_LEN					(48+RFM_PRM_LEN)
#define RFM_DATA_BIT_LEN				((RFM_DATA_LEN)*(BYTE_TO_BIT))	

#define RFM_RX_DATA_LEN					(48)
#define RFM_RX_DATA_BIT_LEN				((RFM_RX_DATA_LEN)*(BYTE_TO_BIT))


#define RFM_RX_PRM_LEN_MAX				((RFM_PREAMBLE_LEN)*(RFM_PREAMBLE_CYCLE_MAX-1))
#define RFM_RX_DATA_LEN_MAX				(((RFM_RX_PRM_LEN_MAX)/(64))+(RFM_DATA_LEN) + 8)
#define RFM_RX_DATA_RAW_LEN_MAX			((RFM_RX_PRM_LEN_MAX)+(RFM_DATA_LEN) + 8)
#define RFM_RX_DATA_BIT_LEN_MAX			((RFM_RX_DATA_LEN_MAX)*(BYTE_TO_BIT))
#define RFM_RX_DATA_BIT_RAW_LEN_MAX			((RFM_RX_DATA_RAW_LEN_MAX)*(BYTE_TO_BIT))

#define _RFM_DECISION_VAL				(40)

#define TEST										0

extern uint8_t rfm_preamble_cycle;

/**************************************
*
*			Defined structure / enum			
*
***************************************/
#if 0
typedef struct{
	int in_ptr;
	int out_ptr;
	int size;
	volatile unsigned char *buff;
}M_RFM_FIFO;
#endif


/**************************************
*	 Data structure
***************************************/
typedef struct
{
	uint64_t recvDataLen;
	uint32_t dataIdx;
	uint32_t	bitPos;
	volatile BOOL bufFull;
	uint8_t wkDet;
	uint16_t prDet;
	uint64_t preamble;
#if defined(MULTIBIT_DECISON_IN_MODEM)
	float multiBitData;
#else
	uint8_t	matchCnt_pn0;
	uint8_t	matchCnt_pn1;
#endif
	uint64_t raw_recvDataLen;
//	volatile uint8_t buff[RFM_DATA_BIT_LEN];
//	volatile uint8_t buff[RFM_RX_DATA_BIT_LEN_MAX];
	volatile uint16_t buff[RFM_RX_DATA_BIT_LEN_MAX];
}M_RFM_RX_DATA;
/****************************************/



typedef struct{
	int8_t bit_ptr;
	int byte_ptr;
	int len;
	bool busy;
	int buffSize;
	bool initialized;
	volatile unsigned char *buff;
}M_RFM_TX_INFO;


enum
{
	RX_MODE = 0,
	TX_MODE
}RF_MODE;


/**************************************
*
*			functions			
*
***************************************/

/*		RX	*/
void RFM_RX_Init(void);
void Configure_EXTI_RX_CLK(void);
void Configure_EXTI_RX_WK(void);
void RFM_RX_Resource_Init(void);
BOOL RFM_RX_GetDataBufState(void);
int16_t RFM_RX_GetData(uint16_t *input);
void RFM_RX_Start(void);
void RFM_RX_WK_Start(void);
void RFM_RX_WK_Stop(void);
void RFM_RX_ADC_Start(void);
void RFM_RX_ADC_Stop(void);
void RFM_RX_PrintData(uint16_t *data, uint64_t len, uint16_t th);
bool RFM_RX_Modem_Init(void);


/*		TX		*/
void RFM_TX_Init(void);
void RFM_TX_Resource_Init(void);
int RFM_TX_Buff_In(char* data , int len);
uint8_t RFM_TX_Buff_Out(uint8_t type);
bool RFM_TX_Busy_Check(void);;
void Configure_EXTI_ADC_CLK(void);

#endif
