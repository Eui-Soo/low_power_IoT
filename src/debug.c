/**
  ******************************************************************************
  * File Name          : debug.c
  * Description        : This file provides code for the configuration
  *                      of the debuging
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 hrlee@airpoint.co.kr
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"
#include "gpio.h"
//#include "rfm.h"

#include "stdio.h"
#include "string.h"
#include "stdarg.h"

#include "flash_if.h"

/* Private defines -----------------------------------------------------------*/
#define DEBUG_BUFFER_SIZE	150


char debugbuffer[DEBUG_BUFFER_SIZE];					// debug input buffer
volatile unsigned char debug_ascii_cnt = 0;				// debug input ASCII index


/* in RW section */
ram1 uint8_t gSramTest1[10] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa};
ram2 uint8_t gSramTest2[10]  = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a};
ram3 uint8_t gSramTest3[10]  = {0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a};
ram3 uint8_t gSramTest4[10];
extern _ULP_FLASH_PARAM ulp_flash_param;


/* Private function prototypes -----------------------------------------------*/


void DEBUG_RecvAsciiData(uint8_t rx_data)
{
	static char buf[20];

	if(rx_data == 127 || rx_data == 8)
	{
		sprintf(buf, "\b\033[K");		// 문자 삭제

		putStr(buf, strlen(buf));

		if(debug_ascii_cnt > 0)
		{
			debugbuffer[--debug_ascii_cnt] = 0;		// 실제 ASCII 버퍼에 적용
		}
		return;
	}

	if(rx_data == 0x0D && debug_ascii_cnt == 0)
	{
		putCh('\r');	putCh('\n');	
		putCh('>');	putCh('>');	putCh(0x20);
		return;
	}

	putCh(rx_data);
	
	debugbuffer[debug_ascii_cnt++]  = (char)rx_data;

	if(debugbuffer[debug_ascii_cnt - 1] == 0x0D)
	{
		debugbuffer[debug_ascii_cnt] = 0;

		debug_ascii_cnt = 0;

		printf(">> %s\r\n", debugbuffer);
		DEBUG_Command(debugbuffer);
		putCh('>');	putCh('>');	putCh(0x20);
	}

	if(debug_ascii_cnt >= DEBUG_BUFFER_SIZE)	debug_ascii_cnt = 0;
}


void PrintDebugHelp(void)
{
	printf("\033[1;34m---------------------------------------------------\r\n");
	printf("\033[1;34m\t\tHELP MANUAL\r\n");
	printf("\033[1;34m---------------------------------------------------\r\n");	
	printf("\033[1;34m\r\n [RF]\r\n");
	printf("\033[1;34m * \"rfic -r [addr(HEX or DEC)]\" : RFM READ FROM ADDR\r\n");
	printf("\033[1;34m * \"rfic -w [addr(HEX or DEC)] [data(HEX or DEC)]\" : RFM WRITE FOR ADDR TO DATA\r\n");
	printf("\033[1;34m * \"rfic -r all\" : RFM shows Current Register\r\n");
	printf("\033[1;34m * \"rfic -d tx\" : RFM Write TX Default Register\r\n");
	printf("\033[1;34m * \"rfic -d rx\" : RFM Write RX Default Register\r\n");
	printf("\033[1;34m---------------------------------------------------\r\n");
	
}


/*
 * debug 명령어 분석하여 해당 명령 실행하는 함수
 */
void DEBUG_Command(char * cmd)
{
	if( !strncmp(cmd, "help", 4) )
	{
		PrintDebugHelp();
	}


	 /*
	 * RFM  명령어 수행
	 */
	else if( !strncmp(cmd, "rfic", 4) )
	{
		RFM_Cmd(cmd);
	}

	 /*
	 * system  명령어 수행
	 */
	else if( !strncmp(cmd, "sys", 3) )
	{
		SYS_Cmd(cmd);
	}
}


/**********************************************************************
*
*		SYSTEM Command Line
*
**********************************************************************/
void SYS_Cmd(char * str)
{
#define MAX_CMD_PARAM_ROW		4
#define MAX_CMD_PARAM_COLUMN	10

	const uint8_t MAX_CMD_LEN = 30;
	const uint8_t SUB_CMD_POS = 4;

	uint8_t param[MAX_CMD_PARAM_ROW][MAX_CMD_PARAM_COLUMN] = { 0, };
	uint8_t paramRowIdx = 0;
	uint8_t paramColumnIdx = 0;
	uint8_t strIdx = 0;
	uint16_t i = 0;


#if 0
	for(i = 0 ; i < MAX_CMD_LEN; i++)
	{
		printf("%d] %c, %x\r\n", i, str[i], str[i]);
	}
#endif		

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

		if(paramRowIdx == MAX_CMD_PARAM_ROW)
		{
			break;
		}

		if(paramColumnIdx == MAX_CMD_PARAM_COLUMN)
		{
			break;
		}
	}


#if 0
	uint16_t k;
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

	if(!strncmp(param[0], "reset", 5))
	{
		NVIC_SystemReset();
	}

	else if(!strncmp(param[0], "sram", 4))
	{
		printf("SRAM1 = 0x%08x\n", gSramTest1);
		for(i = 0 ; i < 10; i++)
		{
			printf("SRAM1[%d] = 0x%x\n", i, gSramTest1[i]);
		}

		printf("SRAM2 = 0x%08x\n", gSramTest2);
		for(i = 0 ; i < 10; i++)
		{
			printf("SRAM2[%d] = 0x%x\n", i, gSramTest2[i]);
		}

		printf("SRAM3 = 0x%08x\n", gSramTest3);
		for(i = 0 ; i < 10; i++)
		{
			printf("SRAM3[%d] = 0x%x\n", i, gSramTest3[i]);
		}

		printf("SRAM1 = 0x%08x\n", gSramTest4);
		for(i = 0 ; i < 10; i++)
		{
			printf("SRAM1[%d] = 0x%x\n", i, gSramTest4[i]);
		}
	}

	else if(!strncmp(param[0], "flash", 5))
	{
		if(!strncmp(param[1], "erase", 5))
		{
			ULP_Flash_Erase(0x081FFF00);
		}
		else if(!strncmp(param[1], "show", 4))
		{
			ULP_Param_show(1);
		}
	}

	else if(!strncmp(param[0], "test", 4))
	{
		TIM1_Init(1000-1);
	}
	else
	{
		printf("Invalid command\n");
	}
		
	return;
}


