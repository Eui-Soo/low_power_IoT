#include "rfm.h"
#include "rfm_def.h"

#include "main.h"
#include "spi.h"
#include "stm32h7xx_hal_gpio.h"
#include "ap_lib.h"
#include "flash_if.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>



/**********************************************************************
*
*		Defined
*
**********************************************************************/

/**************************************
*		RF/WMODEM Register		
***************************************/
#define RFM_TX_MODULATION_REG			0x27
#define RFM_TX_MODULATION_POS			0x10
#define RFM_OFFSET_REG					0x25
#define RFM_VCO_REG						0x22
#define RFM_BB_GAIN_REG					0x0B
#define RFM_DATA_RATE					0x08


#define WMO_TH_LO_REG					0x400
#define WMO_TH_HI_REG					0x401

#define WMO_CTR_REG						0x600
#define WMO_CTR_CORREN_POS				0x01
//#define WMO_CTR_CLRINTR_POS				0x10
#define WMO_CTR_CLRINTR_POS				0x01


#define WMO_CLKCTR_REG					0x601
#define WMO_CLKCTR_ALWAYSON_POS			0x01


#define RFM_SPI1_CS_LOW()	HAL_GPIO_WritePin(RFM_SPI_CS_PORT, RFM_SPI_CS_PIN, GPIO_PIN_RESET)
#define RFM_SPI1_CS_HIGH()	HAL_GPIO_WritePin(RFM_SPI_CS_PORT, RFM_SPI_CS_PIN, GPIO_PIN_SET)

extern SPI_HandleTypeDef RFM_SPI;
/**********************************************************************
*
*		Variable
*
**********************************************************************/
/*
*	!! PN must be aligned in 8byte unit
*/

#if(PN_LEN == 64)

uint8_t PN_1[PN_LEN] =
{
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
};

uint8_t PN_0[PN_LEN] =
{
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
};
#else
uint8_t PN_1[PN_LEN] =
{
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
};

uint8_t PN_0[PN_LEN] =
{
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
};
#endif
/*
uint8_t PN_1[64] =
{
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
}; 

uint8_t PN_0[64] = 
{
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,		
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,		
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,		
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,		
	0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
};
*/

bool ferTestOn = false;
uint16_t ferCounter = 0;
uint16_t ferTryCounter = 0;
uint8_t ferDuration = 0;

uint8_t rfm_preamble_cycle  = 1;

uint8_t ferData[RFM_DATA_LEN/2] = 
{
	0x81,  0x00,  0x00,  0x00,  0x01,  0x02,  0x03,  0x04,
	0x05,  0x06,  0x07,  0x08,  0x09,  0x0a,  0x0b,  0x0c,
	0x0d,  0x0e,  0x0f,  0x10,  0x11,  0x12,  0x13,  0x14
};

bool bFerModeOn = false;
uint16_t ferRecvCounter = 0;
uint16_t ferMissCounter = 0;


extern struct process fer_process;

extern const int8_t TX_DEFAULT_REG[RFM_ADDRESS_MAX][2];
extern const int8_t RX_DEFAULT_REG[RFM_ADDRESS_MAX][2];

extern _ULP_FLASH_PARAM ulp_flash_param;
extern const uint16_t MODEM_WAKEUP_PATTERN[WMODEM_ADDRESS_MAX][2];



extern void RFM_RX_Init(void);
extern void RFM_TX_Init(void);


/**********************************************************************
*
*		RFM Driver Funcions
*
**********************************************************************/
bool MODEM_WriteRregister(uint16_t addr, uint8_t data)
{
	uint8_t i, readData;

	for(i = 0; i < 3; i++)
	{
		MODEM_Write(addr, data);

		readData = MODEM_Read(addr);	
		
		if(readData == data) return true;
		else continue;
	}

	printf("[MDATA MISMATCH] ADDR(0x%04X) WRITE DATA : 0x%02X, READ DATA : 0x%02X \r\n", 
		addr, data, readData);
	
	return false;
}


void MODEM_Write(uint16_t addr, uint8_t data)
{
	if(addr < WMODEM_START_ADDRESS)
	{
		printf("MODEM Address is invalid\n");
		return;
	}
	
	uint8_t txData[3];

	if(addr < WMODEM_START_ADDRESS)
	{
		printf("MODEM Address is invalid\n");
		return;
	}

	txData[0] = ((addr & 0x7F00) >> 8);
	txData[1] = (addr & 0xFF);
	txData[2] = data;

	RFM_SPI1_CS_LOW();

	HAL_SPI_Transmit(&RFM_SPI, txData ,3, 100);

	RFM_SPI1_CS_HIGH();
}


uint8_t MODEM_Read(uint16_t addr)
{
	uint8_t readData = 0;
	uint8_t txData[2];

	if(addr < WMODEM_START_ADDRESS)
	{
		printf("MODEM Address is invalid\n");
		return 0;
	}

	txData[0] = ( ((addr | 0x8000) & 0xFF00) >> 8 );
	txData[1] = (addr & 0xFF);

	RFM_SPI1_CS_LOW();

	HAL_SPI_Transmit(&RFM_SPI, txData , 2, 100);

	HAL_SPI_Receive(&RFM_SPI, &readData, 1, 100);

	RFM_SPI1_CS_HIGH();

	return readData;
	
}


void WMO_Show_Register(void)
{
	int16_t i;

	printf("=======================================\r\n");
	printf("	 RFM Wakeup Modem Register\r\n");
	printf("=======================================\r\n");

	printf("Control[0x%04x] = 0x%02X\r\n", WMO_CTR_REG, MODEM_Read(WMO_CTR_REG));
	printf("Clock Control[0x%04x] = 0x%02X\r\n", WMO_CLKCTR_REG, MODEM_Read(WMO_CLKCTR_REG));
	printf("Wakeup Threshold[0x%04x] = %d\r\n\r\n", WMO_TH_LO_REG, WMO_Read_WakeupTH());
	
	for(i = 0 ; i < WMODEM_ADDRESS_MAX; i++)
	{
		if(i % 8 == 0)	printf("[0x%04X] ", MODEM_WAKEUP_PATTERN[i][0]);
		printf("0x%02X ", MODEM_Read(MODEM_WAKEUP_PATTERN[i][0]));
		if((i+1) % 8 == 0)	printf("\r\n");
		RFM_Wait(100);
	} 

	printf("=======================================\r\n");
}


bool WMO_Set_DefaultRegister(void)
{
	const uint8_t MAX_WR_CNT = 3;
	
	int16_t i, j;
	bool wr_ok;

	for(i = 0 ; i < WMODEM_ADDRESS_MAX; i++)
	{
		wr_ok = false;
		
		for(j = 0 ; j < MAX_WR_CNT; j++)
		{
			if(MODEM_WriteRregister(MODEM_WAKEUP_PATTERN[i][0], MODEM_WAKEUP_PATTERN[i][1]) == true)
			{
				wr_ok = true;
				break;
			}
		}

		if(wr_ok == false)	return false;
	}



	return true;
}


bool WMO_Set_Corr(uint8_t onoff)
{
	uint8_t data;
	
	data = MODEM_Read(WMO_CTR_REG);

	if(onoff == SET)	data |= WMO_CTR_CORREN_POS;
	else						data &= (~WMO_CTR_CORREN_POS);
	
	if(MODEM_WriteRregister(WMO_CTR_REG, data) == false)
	{
		return false;
	}

	printf("\nWMODEM Corr %s [Addr:0x%02x, Value:0x%02x]\n", 
		((onoff == 1) ? "Enabled" : "Disabled"), RFM_OFFSET_REG, onoff);
	
	return true;
}

bool WMO_Set_CLRIntr(uint8_t onoff)
{
	uint8_t data = 0;
	
	data = MODEM_Read(WMO_CTR_REG);

	data &= ~(0x80);		// Prevent wrong read at address 0x10 

	data |= 0x06;

	if(onoff == SET)
	{
		data |= WMO_CTR_CLRINTR_POS;
		HAL_GPIO_WritePin(FRZ_PA_GPIO_Port, PA_EN_Pin, GPIO_PIN_SET);
	}
	else
	{
		data &= (~WMO_CTR_CLRINTR_POS);
		HAL_GPIO_WritePin(FRZ_PA_GPIO_Port, PA_EN_Pin, GPIO_PIN_RESET);
	}
	
	if(MODEM_WriteRregister(WMO_CTR_REG, data) == false)
	{
		return false;
	}
//	printf("MODEM[%x] = %x\r\n", WMO_CTR_REG, MODEM_Read(WMO_CTR_REG));
	return true;
}


bool WMO_ClearIntr(void)
{
	WMO_Set_CLRIntr(RESET);
	RFM_Wait(500);
	WMO_Set_CLRIntr(SET);
	//printf("WMODEM Interrupt Cleared\n");
}


bool WMO_Set_AlwaysOn(uint8_t onoff)
{
	if(MODEM_WriteRregister(WMO_CLKCTR_REG, onoff) == false)
	{
		return false;
	}

	printf("WMODEM Always %s [Addr:0x%02x, Value:0x%02x]\n", 
		((onoff == 1) ? "On" : "Off"),	WMO_CLKCTR_REG, onoff);
	
	return true;
}


bool WMO_Set_WakeupTH(uint16_t th)
{
	uint8_t lo_th, hi_th;

	lo_th = th & 0xff;
	
	if(MODEM_WriteRregister(WMO_TH_LO_REG, lo_th) == false)
	{
		return false;
	}

	if(th > 255)
	{
		hi_th = (th >> 8) & 0x0f;
		
		if(MODEM_WriteRregister(WMO_TH_HI_REG, hi_th) == false)
		{
			return false;
		}
	}

	printf("Set WMODEM Wakeup Threshold [TH:%d]\n", th);
	
	return true;
}

uint16_t WMO_Read_WakeupTH(void)
{
	uint8_t lo_th, hi_th;
	uint16_t data = 0x0;

	lo_th = MODEM_Read(WMO_TH_LO_REG);
	hi_th = MODEM_Read(WMO_TH_HI_REG);

	data = (hi_th << 8) | lo_th;
	
	return data;
}

bool WMO_Set_DataRate(uint8_t dr)
{
	uint8_t data;

	if(dr == 0)
	{
		data = 0x07;
	}
	else if(dr == 1)
	{
		data = 0x05;
	}
	else if(dr == 2)
	{
		data = 0x03;
	}
	else if(dr == 3)
	{
		data = 0x01;
	}
	else
		return false;

	if(MODEM_WriteRregister(WMO_CLKCTR_REG, data) == false)
	{

		return false;
	}

	RFM_Wait(100);

	return true;
}

uint8_t WMO_Read_DataRate(void)
{
	uint8_t data;

	data = MODEM_Read(WMO_CLKCTR_REG);

	return data;
}
	
void RFM_Show_RegisterAll(void)
{
	int8_t i;

	printf("===============================\r\n");
	printf("   RFM Current Register\r\n");
	printf("===============================\r\n");
	
	for(i = 0 ; i < RFM_ADDRESS_MAX; i++)
	{
		printf("[0x%04X] 0x%02X\r\n", RX_DEFAULT_REG[i][0], RFM_Read(RX_DEFAULT_REG[i][0]));
		RFM_Wait(30);
	}	

	printf("===============================\r\n");
}


bool RFM_Set_RxDefaultRegister(void)
{
	int8_t i;
	uint8_t regData = 0x0;

	for(i = 0 ; i < RFM_ADDRESS_MAX; i++)
	{
		switch(RX_DEFAULT_REG[i][0])
		{

			case RFM_OFFSET_REG:
				regData = ulp_flash_param.rf_offset;
				break;

			case RFM_VCO_REG:
				regData = ulp_flash_param.rf_vco;
				break;
#if(0)
			case RFM_BB_GAIN_REG:
				regData = ulp_flash_param.bb_gain;
				break;
#endif
			case RFM_DATA_RATE:
				regData = (ulp_flash_param.data_rate) << 4;
				break;

			default:
				regData = RX_DEFAULT_REG[i][1];
				break;

		}



		if(RFM_WriteRregister(RX_DEFAULT_REG[i][0], regData) == false)
		{
			return false;
		}

		RFM_Wait(30);
	}


#if(0)
	if(RFM_WriteRregister(RX_DEFAULT_REG[0][0], 0x7F) == false)
	{
		return false;
	}
#endif
	if(RFM_WriteRregister(0x0021, 0x00) == false)
	{
		return false;
	}
	RFM_Wait(30);
	if(RFM_WriteRregister(0x0000, 0x2E) == false)
	{
		return false;
	}
	RFM_Wait(30);

	if(RFM_WriteRregister(0x0000, 0x3E) == false)
	{
		return false;
	}
	RFM_Wait(30);

	if(WMO_Set_DataRate(ulp_flash_param.data_rate) == false)
	{
		return false;
	}

	return true;
}


bool RFM_Set_TxDefaultRegister(void)
{
	int8_t i;
	uint8_t regData = 0x0;

	for(i = 0 ; i < RFM_ADDRESS_MAX; i++)
	{
		switch(TX_DEFAULT_REG[i][0])
		{
#if(0)
			case RFM_OFFSET_REG:
				regData = ulp_flash_param.rf_offset;
				break;

			case RFM_VCO_REG:
				regData = ulp_flash_param.rf_vco;
				break;

			case RFM_BB_GAIN_REG:
				regData = ulp_flash_param.bb_gain;
				break;
#endif
			case RFM_DATA_RATE:
				regData = (ulp_flash_param.data_rate) << 4;
				break;

			default:
				regData = TX_DEFAULT_REG[i][1];
				break;

		}

		//regData = TX_DEFAULT_REG[i][1];

		if(RFM_WriteRregister(TX_DEFAULT_REG[i][0], regData) == false)
		{
			return false;
		}

		RFM_Wait(30);


	}


#if(0)
	if(RFM_WriteRregister(TX_DEFAULT_REG[0][0], 0x9F) == false)
	{
		return false;
	}
#endif

	if(RFM_WriteRregister(0x0021, 0x00) == false)
	{
		return false;
	}
	RFM_Wait(30);

	if(RFM_WriteRregister(0x0000, 0x8F) == false)
	{
		return false;
	}
	RFM_Wait(30);

	if(RFM_WriteRregister(0x00, 0x9F) == false)
	{
		return false;
	}

	RFM_Wait(30);

	if(WMO_Set_DataRate(ulp_flash_param.data_rate) == false)
	{
		return false;
	}


	return true;
}

bool RFM_Set_TxModulation(uint8_t onoff)
{
	uint16_t data;
	
	data = RFM_Read(RFM_TX_MODULATION_REG);

	if(onoff == SET)	data |= RFM_TX_MODULATION_POS;
	else				data &= (~RFM_TX_MODULATION_POS);
	
	RFM_Wait(30);

	if(RFM_WriteRregister(RFM_TX_MODULATION_REG, data) == false)
	{
		return false;
	}

	printf("Tx Modulation Mode %s\n", (onoff == SET)?"ON":"OFF");
	
	return true;
}


bool RFM_Set_Offset(uint8_t data)
{
	if(RFM_WriteRregister(RFM_OFFSET_REG, data) == false)
	{
		return false;
	}

	printf("Set Offset Value[Addr:0x%02x, Value:0x%02x]\n", RFM_OFFSET_REG, data);
	
	return true;
}

bool RFM_Set_DataRate(uint8_t dr)
{
	uint8_t data;

	data = (dr << 4) & 0xF0;

	if(RFM_WriteRregister(RFM_DATA_RATE, data) == false)
	{
		return false;
	}

	printf("Set Data Rate[Addr:0x%02x, Value:0x%02x]\n", RFM_DATA_RATE, data);

	return true;

}



bool RFM_Set_Vco(uint8_t data)
{
	if(RFM_WriteRregister(RFM_VCO_REG, data) == false)
	{
		return false;
	}

	printf("Set Vco Value[Addr:0x%02x, Value:0x%02x]\n", RFM_VCO_REG, data);
	
	return true;
}

bool RFM_Set_BBGain(uint8_t data)
{
	if(RFM_WriteRregister(RFM_BB_GAIN_REG, data) == false)
	{
		return false;
	}

	printf("Set BB_Gain Value[Addr:0x%02x, Value:0x%02x]\n", RFM_BB_GAIN_REG, data);
	
	return true;
}



bool RFM_WriteRregister(uint16_t addr, uint8_t data)
{
	uint8_t i, readData;

	for(i = 0; i < 3; i++)
	{
		RFM_Write(addr, data);
		
		RFM_Wait(5);

		readData = RFM_Read(addr);	
		
		if(readData == data) return true;
		else continue;
	}

	printf("[DATA MISMATCH] ADDR(0x%04X) WRITE DATA : 0x%02X, READ DATA : 0x%02X \r\n", 
		addr, data, readData);
	
	return false;
}


void RFM_Write(uint16_t addr, uint8_t data)
{
	uint8_t txData[3];

	txData[0] = ((addr & 0x7F00) >> 8);
	txData[1] = (addr & 0xFF);
	txData[2] = data;

	RFM_SPI1_CS_LOW();

	HAL_SPI_Transmit(&RFM_SPI, txData ,3, 100);

	RFM_SPI1_CS_HIGH();
}

uint8_t RFM_Read(uint16_t addr)
{
	uint8_t readData;
	uint8_t txData[2];

	txData[0] = ( ((addr | 0x8000) & 0xFF00) >> 8 );
	txData[1] = (addr & 0xFF);

	RFM_SPI1_CS_LOW();

	HAL_SPI_Transmit(&RFM_SPI, txData , 2, 100);

	HAL_SPI_Receive(&RFM_SPI, &readData, 1, 100);

	RFM_SPI1_CS_HIGH();

	return readData;

}

void RFM_Wait(int32_t delay)
{
	volatile int i,j;
	
	for(int i = 0; i < delay; i++)
	{
		for(j = 0; j < 16; j++);
	}
}


void RFM_Init(void)
{	
#if defined(FEATURE_FIRENZE_RX_MODE)
	RFM_RX_Init();
#elif defined(FEATURE_FIRENZE_TX_MODE)
	RFM_TX_Init();
#else
	printf("FIRENZE RFM Error... Select TX or RX Mode\r\n");
#endif



}



/**********************************************************************
*
*		RFM Command Line
*
**********************************************************************/
void RFM_Cmd(char * str)
{
#define MAX_CMD_LEN					30
#define CMD_IDENTIFIER_POS	5
#define CMD_TYPE_POS				6
#define SUB_CMD_POS					8

#define MAX_PARAM_ROW				4
#define MAX_PARAM_COLUMN		10

	uint8_t param[MAX_PARAM_ROW][MAX_PARAM_COLUMN] = { 0, };
	uint8_t paramRowIdx = 0;
	uint8_t paramColumnIdx = 0;
	uint8_t strIdx = 0;
	
	BOOL bInvalidCmd = false;



	uint8_t rfm_cmd_tasking = 1;
	//char * tempstr = str + 4;
	char cut[20] = { 0, };
	volatile int i = 0, k = 0, rfm_convert = 0;

	uint8_t rw_flag = 0;
	uint16_t addr = 0;
	uint8_t data = 0;

#if 0
	for(i = 0 ; i < MAX_CMD_LEN; i++)
	{
		printf("%d] %c, %x\r\n", i, str[i], str[i]);
	}
#endif		

	if(str[CMD_IDENTIFIER_POS] != '-')
	{
		printf("Invalid Identifier['-']\r\n");
		return;
	}

	strIdx = SUB_CMD_POS;
	
	while(1)
	{
		param[paramRowIdx][paramColumnIdx] = str[strIdx];

		//printf("%d:%d] (%d) %c, %x\n", paramRowIdx, paramColumnIdx, strIdx, str[strIdx], str[strIdx]);

		if(str[strIdx] == 0x20)		// space
		{
			strIdx++;
			paramRowIdx++;
			paramColumnIdx = 0;
			continue;
		}
		else if(str[strIdx] == '\r')
		{
			break;
		}

		strIdx++;
		paramColumnIdx++;

		if(paramRowIdx == MAX_PARAM_ROW)
		{
			break;
		}

		if(paramColumnIdx == MAX_PARAM_COLUMN)
		{
			break;
		}
	}


#if 0
	printf("paramRowIdx: %d\n", paramRowIdx);

	for(i = 0 ; i <= paramRowIdx; i++)
	{
		for(k = 0 ; k < paramColumnIdx; k++)	
		{
			printf("%c", param[i][k]);
		}
		printf("\r\n");
	}
#endif

	
	if(str[CMD_TYPE_POS] == 'r')
	{
		if(!ap_strnicmp(param[0], "all", 3))
		{
			RFM_Show_RegisterAll();
		}
		else if(!strncmp(param[0], "offset", 5))
		{
			data =  RFM_Read(RFM_OFFSET_REG);
			printf("RF Offset : 0x%02x\r\n", data);
		}
		else if(!strncmp(param[0], "vco", 3))
		{
			data =  RFM_Read(RFM_VCO_REG);
			printf("VCO : 0x%02x\r\n", data);
		}
		else if(!ap_strnicmp(param[0], "bb", 2))
		{
				data =  RFM_Read(RFM_BB_GAIN_REG);
				printf("BB_GAIN = 0x%02x\n", data); 	
		}

		else if(!strncmp(param[0], "modem", 5))
		{
			WMO_Show_Register();
		}

		else if(!strncmp(param[0], "wk_th", 5))
		{
			printf("Wakeup Threshold : %d\r\n", WMO_Read_WakeupTH());
		}
		else if(!strncmp(param[0], "dr", 2))
		{
			printf("RF Data Rate : 0x%02x\r\n", RFM_Read(RFM_DATA_RATE));
			printf("MODEM Data Rate : 0x%02x\r\n", WMO_Read_DataRate());
		}

		else
		{
			addr = ap_atoi(param[0]);	
			data = RFM_Read(addr);
			
			printf("RFM READ : [0x%04X] 0x%02X\r\n", addr, data);
		}
	}
	
	else if(str[CMD_TYPE_POS] == 'w')
	{
		if(!ap_strnicmp(param[0], "offset", 6))
		{
			data = ap_atoi(param[1]);	

			if(RFM_Set_Offset(data) == false)
			{
				NVIC_SystemReset();	
			}

			ulp_flash_param.rf_offset = data;

			printf("\nRF Ofsset is saved : 0x%02x\r\n", data);
			
			ULP_Flash_Write();
		}
		else if(!ap_strnicmp(param[0], "vco", 3))
		{
			data = ap_atoi(param[1]);	

			if(RFM_Set_Vco(data) == false)
			{
				NVIC_SystemReset();	
			}

			ulp_flash_param.rf_vco = data;

			printf("\nRF Vco is saved : 0x%02x\r\n", data);
			
			ULP_Flash_Write();
		}
		else if(!ap_strnicmp(param[0], "mod_off", 6))
		{
			if(RFM_Set_TxModulation(RESET) == true)
			{
				printf("success\r\n");
			}
		}
		else if(!ap_strnicmp(param[0], "mod_on", 6))
		{
			if(RFM_Set_TxModulation(SET) == true)
			{
				printf("success\r\n");
			}
		}
		else if(!ap_strnicmp(param[0], "txmode", 6))
		{
			//HAL_GPIO_WritePin(GPIOA, RF_SWITCH_Pin, GPIO_PIN_SET);
			printf("Set Tx Mode\r\n");
		}
		else if(!ap_strnicmp(param[0], "rxmode", 6))
		{
			//HAL_GPIO_WritePin(GPIOA, RF_SWITCH_Pin, GPIO_PIN_RESET);
			printf("Set Rx Mode\r\n");
		}

		else if(!ap_strnicmp(param[0], "bb", 2))
		{
			data = ap_atoi(param[1]);	

			if(RFM_Set_BBGain(data) == false)
			{
				NVIC_SystemReset();	
			}

			ulp_flash_param.bb_gain = data;

			printf("\nRF BB_GAIN is saved : 0x%02x\r\n", data);
			
			ULP_Flash_Write();
		}
		else if(!ap_strnicmp(param[0], "dr", 2))
		{
			data = ap_atoi(param[1]);

			ulp_flash_param.data_rate = data;

			RFM_Set_DataRate(ulp_flash_param.data_rate);
			WMO_Set_DataRate(ulp_flash_param.data_rate);

			printf("\nData Rate is saved : %d\r\n", data);

			ULP_Flash_Write();
		}

#if defined(FEATURE_FIRENZE_RX_MODE)
		else if(!strncmp(param[0], "wk_th", 5))
		{
			uint16_t data = (uint16_t)ap_atoi(param[1]);	

			if(data > 4096)
			{
				printf("\nWakeup threashold is invalid : %d\r\n", data);
				return;
			}

			if(WMO_Set_WakeupTH(data) == false)
			{
				NVIC_SystemReset();	
			}

			ulp_flash_param.wk_th = data;
			printf("\nWakeup threashold is saved : %d\r\n", data);
			
			ULP_Flash_Write();
		}
#endif		
		else
		{
			addr = ap_atoi(param[0]);	
			data = ap_atoi(param[1]);	
			
			if(RFM_WriteRregister(addr, data) == false)
			{
				printf("[Fail] RFM Write Register\r\n");
				return;
			}
			
			printf("RFM Write : [0x%04X] 0x%02X\r\n", addr, data);
		}
	}
	else if(str[CMD_TYPE_POS] == 'c')
	{
		data = ap_atoi(param[0]);
		ulp_flash_param.duty_cycle = data;

		rfm_preamble_cycle = (ulp_flash_param.duty_cycle == 0) ? 1 : ulp_flash_param.duty_cycle;

		RFM_TX_Resource_Init();
		printf("Preamble Cycle is %d\r\n", rfm_preamble_cycle);

		ULP_Flash_Write();
	}
	else if(str[CMD_TYPE_POS] == 'p')
	{
		data = ap_atoi(param[0]);
		ulp_flash_param.display_count = data;

		printf("Count Display is %d\r\n", ulp_flash_param.display_count);

		ULP_Flash_Write();
	}
	else if(str[CMD_TYPE_POS] == 'n')
	{
		data = ap_atoi(param[0]);
		ulp_flash_param.pn_len = data;

		RFM_TX_Resource_Init();

		printf("PN Code Length is %d\r\n", ulp_flash_param.pn_len);

		ULP_Flash_Write();
	}
	else if(str[CMD_TYPE_POS] == 'v')
	{
		uint16_t data = (uint16_t)ap_atoi(param[0]);

		ulp_flash_param.decision_val = data;

		ULP_Flash_Write();

		printf("Decision Val is %d\r\n", ulp_flash_param.decision_val);

	}
	else if(str[CMD_TYPE_POS] == 'm')
	{
		WMO_ClearIntr();
		printf("Modem Clear\r\n");
	}
	else if(str[CMD_TYPE_POS] == 'd')
	{
#if defined(FEATURE_FIRENZE_RX_MODE)		
		if(RFM_Set_RxDefaultRegister() == false)
		{
			return;
		}
		
		printf("RX Default Register is set\r\n");
		
#elif defined(FEATURE_FIRENZE_TX_MODE)		
		if(RFM_Set_TxDefaultRegister() == false)
		{
			return;
		}

		printf("TX Default Register is set\r\n");
#else
		printf("Operation mode is invaild\r\n");
#endif
	}
	
	else if(str[CMD_TYPE_POS] == 't')
	{
#if defined(FEATURE_FIRENZE_TX_MODE)	
		if(!ap_strnicmp(param[0], "pr", 2))
		{
			// TX 프리엠블 전송
			printf("[TX] Send Preamble\n");
			while(RFM_TX_Busy_Check());
			printf("[TX] Not busy\n");

			if(RFM_TX_Buff_In(NULL, 0) > 0)
			{
				printf("[TX] TX Done\n");
			}
		}

		else if(!ap_strnicmp(param[0], "tfer", 4))
		{
			// TX 테스트 데이터 전송 			
			static bool fTh_is_run = FALSE;
			
			ferCounter = (uint16_t)ap_atoi(param[1]);	
			ferDuration = (uint8_t)ap_atoi(param[2]);	 
			
			ferTryCounter = 0;
			ferTestOn = true;			

			if(ferCounter < 1)		ferCounter = 1;
			if(ferDuration < 1)	ferDuration = 1;

			if(fTh_is_run == FALSE)
			{
				//process_exit(&fer_process);
				process_start(&fer_process, NULL);
				fTh_is_run = TRUE;
			}
		}
		else if(!ap_strnicmp(param[0], "time", 4))
		{

			//ulp_flash_param.app_test_time = (uint8_t)ap_atoi(param[1]);
			//printf("App Test TIme is %d\r\n", ulp_flash_param.app_test_time);

			ULP_Flash_Write();
		}

		else if(!ap_strnicmp(param[0], "stop", 4))
		{
			// TX 중지
			extern bool appProcessRun; 
			appProcessRun = FALSE;
			printf("Stop transfer\n");
		}

		else if(!ap_strnicmp(param[0], "start", 5))
		{
			// TX 시작 
			extern bool appProcessRun; 
			appProcessRun = TRUE;
			printf("Start transfer\n");
		}

		else if(!ap_strnicmp(param[0], "pfer", 4))
		{
			// TX 테스트 데이터 중단	
			process_exit(&fer_process);
		}
#elif defined(FEATURE_FIRENZE_RX_MODE)
		if(!strncmp(param[0], "sfer", 2))
		{
			// Start
			printf("[RX] FER Start\n");
			bFerModeOn = true;
			ferRecvCounter = 0;
			ferMissCounter = 0;
		}

		else if(!strncmp(param[0], "pfer", 2))
		{
			// Stop
			printf("[RX] FER Stop\n");
			bFerModeOn = false;
			ferRecvCounter = 0;
			ferMissCounter = 0;
		}
#endif

		else if(!ap_strnicmp(param[0], "led_on", 6))
		{
			printf("SET LED2 ON\n");
			LED_On(LED2);
		}

		else if(!ap_strnicmp(param[0], "led_off", 7))
		{
			printf("SET LED2 OFF\n");
			LED_Off(LED2);
		}

		else if(!ap_strnicmp(param[0], "sleep", 5))
		{
			data = ap_atoi(param[1]);
			ulp_flash_param.sleep = data;
			printf("SET Sleep Mode\n");

			ULP_Flash_Write();

			//printf("Clear Sleep Mode\n");
		}

	}
	
	else if(str[CMD_TYPE_POS] == 's')
	{
	}
	
	else
	{
		printf("Invalid Command : -r/-w/-d/-t/-s\r\r\n");
		return;
	}


	return;
}

