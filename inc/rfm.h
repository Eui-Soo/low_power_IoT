#ifndef _RFM_H_
#define _RFM_H_

#include "main.h"
#include "flash_if.h"

#include "common.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"


/**************************************
*
*			Defined structure / enum			
*
***************************************/

#define RFM_ADDRESS_MAX	49

#define WMODEM_START_ADDRESS		0x200
#define WMODEM_ADDRESS_MAX			512



/*		PN 		*/
#define PN_PATTERN_1		0xFF
#define PN_PATTERN_0		0x00
#define PN_LEN					128
//#define PN_LEN					64
//#define PN_LEN				8


enum
{
	READ = 1,
	WRITE,
	DEFAULT,				
	RX_DEFAULT,			
	TX_DEFAULT,			
	RX_MOD_OFF_MDOE,		
	READ_ALL,
	CMD_TEST
}RFM_CMD_TYPE;


/**************************************
*
*			functions			
*
***************************************/
void RFM_Cmd(char * str);
bool RFM_Set_RxDefaultRegister(void);
bool RFM_Set_TxDefaultRegister(void);
bool RFM_Set_TxModulation(uint8_t onoff);
void RFM_Write(uint16_t addr, uint8_t data);
uint8_t RFM_Read(uint16_t addr);
bool RFM_WriteRregister(uint16_t addr, uint8_t data);
void RFM_Wait(int32_t delay);
void RFM_Show_RegisterAll(void);
void RFM_Init(void);
bool RFM_Set_Offset(uint8_t data);
bool RFM_Set_Vco(uint8_t data);
bool RFM_Set_BBGain(uint8_t data);;
bool RFM_Set_DataRate(uint8_t dr);

void MODEM_Write(uint16_t addr, uint8_t data);
uint8_t MODEM_Read(uint16_t addr);
bool MODEM_WriteRregister(uint16_t addr, uint8_t data);
void WMO_Show_Register(void);
bool WMO_Set_DefaultRegister(void);
bool WMO_Set_Corr(uint8_t onoff);
bool WMO_Set_CLRIntr(uint8_t onoff);
bool WMO_ClearIntr(void);
bool WMO_Set_AlwaysOn(uint8_t onoff);
bool WMO_Set_WakeupTH(uint16_t th);
uint16_t WMO_Read_WakeupTH(void);
bool WMO_Set_DataRate(uint8_t dr);
uint8_t WMO_Read_DataRate(void);

#endif
