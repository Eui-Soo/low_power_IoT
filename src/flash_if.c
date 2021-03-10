/**
  ******************************************************************************
  * @file    IAP_Main/Src/flash_if.c 
  * @author  MCD Application Team
  * @brief   This file provides all the memory related operation functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/** @addtogroup STM32H7xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"
#include "common.h"
#include "stm32h7xx_hal_flash.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static uint32_t GetSector(uint32_t Address);

_ULP_FLASH_PARAM ulp_flash_param;


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{
  HAL_FLASH_Unlock(); 

  /* Clear pending flags (if any) */  
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                         FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  StartSector: start of user flash area
  * @retval 0: user flash area successfully erased
  *         1: error occurred
  */
uint32_t FLASH_If_Erase(uint32_t StartSector)
{
  uint32_t UserStartSector;
  uint32_t SectorError;
  FLASH_EraseInitTypeDef pEraseInit;

  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_If_Init();
  
  /* Get the sector where start the user flash area */
  UserStartSector = GetSector(APPLICATION_ADDRESS);
  
  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  pEraseInit.Sector = UserStartSector;
  pEraseInit.NbSectors = 8 - UserStartSector;
  pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  
  if (APPLICATION_ADDRESS < ADDR_FLASH_SECTOR_0_BANK2)
  {
    pEraseInit.Banks = FLASH_BANK_1;
    if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
    {
      /* Error occurred while sector erase */
      return (1);
    }
    
    /* Mass erase of second bank */
    pEraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE;
    pEraseInit.Banks = FLASH_BANK_2;
    if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
    {
      /* Error occurred while sector erase */
      return (1);
    }
  }
  else
  {
    pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    pEraseInit.Banks = FLASH_BANK_2;
    if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
    {
      /* Error occurred while sector erase */
      return (1);
    }
  }
  
  return (0);
}

/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  FlashAddress: start address for writing data buffer
  * @param  Data: pointer on data buffer
  * @param  DataLength: length of data buffer (unit is 32-bit word)   
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */
uint32_t FLASH_If_Write(uint32_t FlashAddress, uint32_t* Data ,uint32_t DataLength)
{
  uint32_t i = 0;

  for (i = 0; (i < DataLength) && (FlashAddress <= (USER_FLASH_END_ADDRESS-32)); i+=8)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, FlashAddress, (uint64_t)((uint32_t)(Data+i))) == HAL_OK)
    {
     /* Check the written value */
      if (*(uint32_t*)FlashAddress != *(uint32_t*)(Data+i))
      {
        /* Flash content doesn't match SRAM content */
        return(FLASHIF_WRITINGCTRL_ERROR);
      }
      /* Increment FLASH destination address */
      FlashAddress += 32;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return (FLASHIF_WRITING_ERROR);
    }
  }

  return (FLASHIF_OK);
}

/**
  * @brief  Returns the write protection status of user flash area.
  * @param  None
  * @retval 0: No write protected sectors inside the user flash area
  *         1: Some sectors inside the user flash area are write protected
  */
uint16_t FLASH_If_GetWriteProtectionStatus(void)
{
  uint32_t ProtectedSECTOR = 0x0;
  FLASH_OBProgramInitTypeDef OptionsBytesStruct;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  if (APPLICATION_ADDRESS < ADDR_FLASH_SECTOR_0_BANK2)
  {
    /* Select Bank1 */
    OptionsBytesStruct.Banks = FLASH_BANK_1;
    
    /* Check if there are write protected sectors inside the user flash area ****/
    HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
  
    /* Get pages already write protected ****************************************/
    ProtectedSECTOR = OptionsBytesStruct.WRPSector & FLASH_SECTOR_TO_BE_PROTECTED;
  }

  /* Select Bank2*/
  OptionsBytesStruct.Banks = FLASH_BANK_2;
  
  /* Check if there are write protected sectors inside the user flash area ****/
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);

  /* Get pages already write protected ****************************************/
  ProtectedSECTOR |= OptionsBytesStruct.WRPSector & FLASH_SECTOR_TO_BE_PROTECTED;

  /* Check if desired pages are already write protected ***********************/
  if(ProtectedSECTOR != 0)
  {
    /* Some sectors inside the user flash area are write protected */
    return FLASHIF_PROTECTION_WRPENABLED;
  }
  else
  { 
    /* No write protected sectors inside the user flash area */
    return FLASHIF_PROTECTION_NONE;
  }
}

/**
  * @brief  Gets the sector of a given address
  * @param  Address: Flash address
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1_BANK1) && (Address >= ADDR_FLASH_SECTOR_0_BANK1))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2_BANK1) && (Address >= ADDR_FLASH_SECTOR_1_BANK1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3_BANK1) && (Address >= ADDR_FLASH_SECTOR_2_BANK1))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4_BANK1) && (Address >= ADDR_FLASH_SECTOR_3_BANK1))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5_BANK1) && (Address >= ADDR_FLASH_SECTOR_4_BANK1))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6_BANK1) && (Address >= ADDR_FLASH_SECTOR_5_BANK1))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7_BANK1) && (Address >= ADDR_FLASH_SECTOR_6_BANK1))
  {
    sector = FLASH_SECTOR_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_0_BANK2) && (Address >= ADDR_FLASH_SECTOR_7_BANK1))
  {
    sector = FLASH_SECTOR_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_1_BANK2) && (Address >= ADDR_FLASH_SECTOR_0_BANK2))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2_BANK2) && (Address >= ADDR_FLASH_SECTOR_1_BANK2))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3_BANK2) && (Address >= ADDR_FLASH_SECTOR_2_BANK2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4_BANK2) && (Address >= ADDR_FLASH_SECTOR_3_BANK2))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5_BANK2) && (Address >= ADDR_FLASH_SECTOR_4_BANK2))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6_BANK2) && (Address >= ADDR_FLASH_SECTOR_5_BANK2))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7_BANK2) && (Address >= ADDR_FLASH_SECTOR_6_BANK2))
  {
    sector = FLASH_SECTOR_6;  
  }
  else /*if((Address < USER_FLASH_END_ADDRESS) && (Address >= ADDR_FLASH_SECTOR_7_BANK2))*/
  {
    sector = FLASH_SECTOR_7;  
  }

  return sector;
}

/**
  * @brief  Configure the write protection status of user flash area.
  * @param  modifier DISABLE or ENABLE the protection
  * @retval HAL_StatusTypeDef HAL_OK if change is applied.
  */
HAL_StatusTypeDef FLASH_If_WriteProtectionConfig(uint32_t modifier)
{
  uint32_t ProtectedSECTOR = 0xFFF;
  FLASH_OBProgramInitTypeDef config_new, config_old;
  HAL_StatusTypeDef result = HAL_OK;
  
  if (APPLICATION_ADDRESS < ADDR_FLASH_SECTOR_0_BANK2)
  {
    /* Select Bank1 */
    config_old.Banks = FLASH_BANK_1;
    config_new.Banks = FLASH_BANK_1;
    
    /* Get pages write protection status ****************************************/
    HAL_FLASHEx_OBGetConfig(&config_old);
  
    /* The parameter says whether we turn the protection on or off */
    config_new.WRPState = modifier;
  
    /* We want to modify only the Write protection */
    config_new.OptionType = OPTIONBYTE_WRP;
    
    /* No read protection, keep BOR and reset settings */
    config_new.RDPLevel = OB_RDP_LEVEL_0;
    config_new.USERConfig = config_old.USERConfig;  
    /* Get pages already write protected ****************************************/
    ProtectedSECTOR = config_old.WRPSector | FLASH_SECTOR_TO_BE_PROTECTED;
  
    /* Unlock the Flash to enable the flash control register access *************/ 
    HAL_FLASH_Unlock();
  
    /* Unlock the Options Bytes *************************************************/
    HAL_FLASH_OB_Unlock();
    
    config_new.WRPSector    = ProtectedSECTOR;
    result = HAL_FLASHEx_OBProgram(&config_new);
  }

  /* Select Bank2 */
  config_old.Banks = FLASH_BANK_2;
  config_new.Banks = FLASH_BANK_2;
  
  /* Get pages write protection status ****************************************/
  HAL_FLASHEx_OBGetConfig(&config_old);

  /* The parameter says whether we turn the protection on or off */
  config_new.WRPState = modifier;

  /* We want to modify only the Write protection */
  config_new.OptionType = OPTIONBYTE_WRP;
  
  /* No read protection, keep BOR and reset settings */
  config_new.RDPLevel = OB_RDP_LEVEL_0;
  config_new.USERConfig = config_old.USERConfig;  
  /* Get pages already write protected ****************************************/
  ProtectedSECTOR = config_old.WRPSector | FLASH_SECTOR_TO_BE_PROTECTED;

  /* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  HAL_FLASH_OB_Unlock();
  
  config_new.WRPSector    = ProtectedSECTOR;
  result = HAL_FLASHEx_OBProgram(&config_new);
  
  return result;
}



void ULP_Flash_Init(void)
{	
	FLASH_If_Init();

	if(ULP_Flash_Read() > 0)
	{
		ULP_DefualtFlashParam();
	}
}


uint32_t ULP_Flash_Erase(uint32_t StartSector)
{
	uint32_t SectorError;

	FLASH_EraseInitTypeDef pEraseInit;

	/* Unlock the Flash to enable the flash control register access *************/ 
	FLASH_If_Init();

	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	pEraseInit.Sector = GetSector(StartSector);
	pEraseInit.NbSectors = 1;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
	pEraseInit.Banks = FLASH_BANK_2;
	if (HAL_FLASHEx_Erase(&pEraseInit, &SectorError) != HAL_OK)
	{
		/* Error occurred while sector erase */
		printf("erase error\n");
		return (1);
	}

	printf("[FLASH] Erase ok\n");
	return (0);
	
}



void ULP_Flash_Write(void)
{		
	uint64_t buffer[4];

	/* copy flash buffer */
	ulp_flash_param.flash_identier = FLASH_IDENTIFIER;
	memcpy((uint8_t *)buffer, (uint8_t *)&ulp_flash_param, sizeof(_ULP_FLASH_PARAM));
	
	if(ULP_Flash_Erase(ULP_PARAM_ADDRESS) > 0)
	{
		NVIC_SystemReset();
	}

//	HAL_Delay(1);
	

	if (FLASH_If_Write(ULP_PARAM_ADDRESS, (uint64_t)((uint32_t)(buffer)), sizeof(buffer)/4) > 0)   
	{
		printf("ULP Param Write Error\n");
		NVIC_SystemReset();
	}			

	printf("[FLASH] Write ok\n");
}


uint32_t ULP_Flash_Read(void)
{
	memcpy((uint8_t *)(&ulp_flash_param), (uint8_t *)ULP_PARAM_ADDRESS, sizeof(_ULP_FLASH_PARAM));

	if(ulp_flash_param.flash_identier != FLASH_IDENTIFIER)
	{
		printf("\n[Flash] Read error\n");
		return (FLASHIF_WRITING_ERROR);
	}

	ULP_Param_show(0);

	return (FLASHIF_OK);
}


void ULP_DefualtFlashParam(void)
{
	memset(&(ulp_flash_param), 0, sizeof(_ULP_FLASH_PARAM));

	ulp_flash_param.flash_identier = FLASH_IDENTIFIER;
	ulp_flash_param.rf_offset = 0x03;
	ulp_flash_param.wk_th = 3000;
	ulp_flash_param.rf_vco = 0x34;
	ulp_flash_param.bb_gain = 0x3C;
	ulp_flash_param.data_rate = 3;
	ulp_flash_param.duty_cycle = 1;
	ulp_flash_param.decision_val = 96;
	ulp_flash_param.display_count = 1;
	ulp_flash_param.sleep = 0;
	ulp_flash_param.pn_len = 64;
	printf("\n[ULP Flash] Default Set\n");
}

void ULP_Param_show(uint8_t type)
{
	if(type == 1)	ULP_Flash_Read();
	
	printf("================= FLASH PARAMETER =================\n");
	printf("IDENTIFIER = %x\n", ulp_flash_param.flash_identier);
	printf("RX_OFFSET = 0x%02x\n", ulp_flash_param.rf_offset);
	printf("WK_TH = %d\n", ulp_flash_param.wk_th);
	printf("VCO = 0x%02x\n", ulp_flash_param.rf_vco);
	//printf("BB_GAIN = 0x%02x\n", ulp_flash_param.bb_gain);
	printf("DATA_RATE = %d.%dKhz\n", (250 >> ulp_flash_param.data_rate), ((25000/(1<<ulp_flash_param.data_rate))%100));
	printf("DUTY CYCLE = %d\n", ulp_flash_param.duty_cycle);
	printf("DECISION_VALUE = %d \n", ulp_flash_param.decision_val);
	printf("COUNT DISPLAY = %d \n", ulp_flash_param.display_count);
	printf("SLEEP_MODE = %d \n", ulp_flash_param.sleep);
	printf("PN CODE LEN = %d \n", ulp_flash_param.pn_len);
	printf("===================================================\n\n");
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
