/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "flash_if.h"



extern int _si_sram_1, _s_sram_1, _e_sram_1;
extern int _si_sram_2, _s_sram_2, _e_sram_2;
extern int _si_sram_3, _s_sram_3, _e_sram_3;

/* USER CODE BEGIN Includes */

#include "ipc.h"
#include "debug.h"
#include "app.h"

#if (CONTIKI==1)
#include "contiki.h"
/*#include "cc2420.h" */
#include "dev/leds.h"
//#include "dev/serial-line.h"
//#include "dev/slip.h"
//#include "dev/uart0.h"
#include "dev/watchdog.h"
#include "dev/xmem.h"
#include "lib/random.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"
/*
 * #include "dev/button-sensor.h"
 * #include "dev/adxl345.h"
 */
#include "sys/clock.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

#include "net/rime/rime.h"

/*
 * #include "sys/node-id.h"
 * #include "cfs-coffee-arch.h"
 * #include "cfs/cfs-coffee.h"
 */
#include "sys/autostart.h"

#include "ipc.h"

/*
 * #include "dev/battery-sensor.h"
 * #include "dev/button-sensor.h"
 * #include "dev/sht11/sht11-sensor.h"
 */

/*
 * SENSORS(&button_sensor);
 */

unsigned short node_id = 0;
unsigned char node_mac[8];

unsigned int idle_count = 0;

#if DCOSYNCH_CONF_ENABLED
static struct timer mgt_timer;
#endif

#ifndef NETSTACK_CONF_WITH_IPV4
#define NETSTACK_CONF_WITH_IPV4 0
#endif

#define UIP_OVER_MESH_CHANNEL 8

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif


extern struct process tx_modem_process;
extern struct process rx_modem_process;
extern struct process mac_process;
extern struct process ULP_process;
extern struct process tx_app_process;
extern struct process rx_app_process;

extern _ULP_FLASH_PARAM ulp_flash_param;

#endif

void init_platform(void);

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t milliscount = 0;				// millis counter
bool IS_STANDBY_MODE = false;		// standby mode check
bool IS_WAKEUP_MODE = false;			// wake up mode check

/* from lora.c */
extern int BSNID;						// BS NID
extern int BroadcastSEQ;				// Broadcast START MSG sequence number (OLD)
extern int BroadcastSEQNEW;				// Broadcast START MSG sequence number (NEW)
/* from lora.c END */


volatile uint16_t	ADC_buff = 0;
char oledbuf[31] = { 0, };

uint8_t RESP_COUNT = 0;				// »£»Ì»Ωºˆ
uint16_t RESP_MPEAK = 0;				// »£»Ìºº±‚
uint8_t RESP_CYCLE = 0;				// »£»Ì¡÷±‚

#define RSEP_MEASURE_COUNT 150			// »£»Ì √¯¡§ µ•¿Ã≈Õ ƒ´øÓ≈Õ

uint8_t buttonL = 0;					// button L pushed check

GPIO_TypeDef* GPIO_PORT[LEDn] = {LD1_GPIO_Port, LD2_GPIO_Port, LD3_GPIO_Port};
const uint16_t GPIO_PIN[LEDn] = {LD1_Pin, LD2_Pin, LD3_Pin};

extern volatile int rcvWaitCnt;			// Lora MSG receive wait counter


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */


/*********************************************************
*
* Function proto type
*
*********************************************************/
void SystemClock_Config(void);
void ApplicationBootingMsg(void);
void SRAM_Init(void);


extern void Debug_RxReady(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* Value defined for WUT */
#define RTC_WUT_TIME               ((uint32_t)30)     /* 25sec */

/* Oscillator time-out values */
#define LSI_TIMEOUT_VALUE          ((uint32_t)100)   /* 100 ms */
#define LSE_TIMEOUT_VALUE          ((uint32_t)5000)  /* 5 s */
#define RTC_TIMEOUT_VALUE          ((uint32_t)1000)  /* 1 s */

/* Defines related to Clock configuration */
/* Uncomment to enable the adequate Clock Source */
#define RTC_CLOCK_SOURCE_LSI
/*#define RTC_CLOCK_SOURCE_LSE*/

#ifdef RTC_CLOCK_SOURCE_LSI
/* ck_apre=LSIFreq/(ASYNC prediv + 1) = 256Hz with LSIFreq=37 kHz RC */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
/* ck_spre=ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV           ((uint32_t)0x120)
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
/* ck_apre=LSEFreq/(ASYNC prediv + 1) = 256Hz with LSEFreq=32768Hz */
#define RTC_ASYNCH_PREDIV          ((uint32_t)0x7F)
/* ck_spre=ck_apre/(SYNC prediv + 1) = 1 Hz */
#define RTC_SYNCH_PREDIV           ((uint32_t)0x00FF)
#endif

/* USER CODE END 0 */


void LED_On(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

void LED_Off(Led_TypeDef Led)
{
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

void LED_Toggle(Led_TypeDef Led)
{
	HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}



void uip_log(char *msg)
{
  printf("%s\n", msg);
}


static void set_rime_addr(void)
{
  linkaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(linkaddr_t));
#if NETSTACK_CONF_WITH_IPV6
  memcpy(addr.u8, node_mac, sizeof(addr.u8));
#else
  if(node_id == 0) {
    for(i = 0; i < sizeof(linkaddr_t); ++i) {
      addr.u8[i] = node_mac[7 - i];
    }
  } else {
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }
#endif
  linkaddr_set_node_addr(&addr);

	if(IS_WAKEUP_MODE == false)
	{
	  printf("Rime started with address ");
	  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
	    printf("%d.", addr.u8[i]);
	  }
	  printf("%d\n", addr.u8[i]);
	}
}



static void print_processes(struct process *const processes[])
{
  /*  const struct process * const * p = processes;*/
  printf("Starting");
  while(*processes != NULL) {
    printf(" '%s'", (*processes)->name);
    processes++;
  }
  putchar('\n');
}


void MAC_Init(void)
{
	rtimer_init();

  /*
   * Hardware initialization done!
   */

  /* Restore node id if such has been stored in external mem */
  //node_id_restore();

  /* If no MAC address was burned, we use the node id or the Z1 product ID */
  if(!(node_mac[0] | node_mac[1] | node_mac[2] | node_mac[3] |
       node_mac[4] | node_mac[5] | node_mac[6] | node_mac[7])) {

#ifdef SERIALNUM
    if(!node_id) {
      PRINTF("Node id is not set, using Z1 product ID\n");
      node_id = SERIALNUM;
    }
#endif
    node_mac[0] = 0xc1;  /* Hardcoded for Z1 */
    node_mac[1] = 0x0c;  /* Hardcoded for Revision C */
    node_mac[2] = 0x00;  /* Hardcoded to arbitrary even number so that
                            the 802.15.4 MAC address is compatible with
                            an Ethernet MAC address - byte 0 (byte 2 in
                            the DS ID) */
    node_mac[3] = 0x00;  /* Hardcoded */
    node_mac[4] = 0x00;  /* Hardcoded */
    node_mac[5] = 0x00;  /* Hardcoded */
    node_mac[6] = node_id >> 8;
    node_mac[7] = node_id & 0xff;
  }

  /* Overwrite node MAC if desired at compile time */
#ifdef MACID
#warning "***** CHANGING DEFAULT MAC *****"
  node_mac[0] = 0xc1;  /* Hardcoded for Z1 */
  node_mac[1] = 0x0c;  /* Hardcoded for Revision C */
  node_mac[2] = 0x00;  /* Hardcoded to arbitrary even number so that
                          the 802.15.4 MAC address is compatible with
                          an Ethernet MAC address - byte 0 (byte 2 in
                          the DS ID) */
  node_mac[3] = 0x00;  /* Hardcoded */
  node_mac[4] = 0x00;  /* Hardcoded */
  node_mac[5] = 0x00;  /* Hardcoded */
  node_mac[6] = MACID >> 8;
  node_mac[7] = MACID & 0xff;
#endif

#ifdef IEEE_802154_MAC_ADDRESS
  /* for setting "hardcoded" IEEE 802.15.4 MAC addresses */
  {
    uint8_t ieee[] = IEEE_802154_MAC_ADDRESS;
    memcpy(node_mac, ieee, sizeof(uip_lladdr.addr));
    node_mac[7] = node_id & 0xff;
  }
#endif /* IEEE_802154_MAC_ADDRESS */

  /*
   * Initialize Contiki and our processes.
   */
  process_init();

  process_start(&etimer_process, NULL);

  //process_start(&test1_process, NULL);

  ctimer_init();

  //init_platform();

  set_rime_addr();

  uint8_t longaddr[8];
  uint16_t shortaddr;

  shortaddr = (linkaddr_node_addr.u8[0] << 8) +
    linkaddr_node_addr.u8[1];
  memset(longaddr, 0, sizeof(longaddr));
  linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);

	if(IS_WAKEUP_MODE == false)
	{
		printf("MAC %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x ",
			longaddr[0], longaddr[1], longaddr[2], longaddr[3],
			longaddr[4], longaddr[5], longaddr[6], longaddr[7]);

		PRINTF(CONTIKI_VERSION_STRING " started. ");

		if(node_id) 	PRINTF("Node id is set to %u.\n", node_id);
		else				PRINTF("Node id not set\n");
	}


#if NETSTACK_CONF_WITH_IPV6
  memcpy(&uip_lladdr.addr, node_mac, sizeof(uip_lladdr.addr));
  /* Setup nullmac-like MAC for 802.15.4 */
/*   sicslowpan_init(sicslowmac_init(&cc2420_driver)); */
/*   printf(" %s channel %u\n", sicslowmac_driver.name, CC2420_CONF_CHANNEL); */

  /* Setup X-MAC for 802.15.4 */
  queuebuf_init();

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();


	if(IS_WAKEUP_MODE == false)
	{
	  printf("%s %s, channel check rate %lu Hz, radio channel is null\n",
	         NETSTACK_MAC.name, NETSTACK_RDC.name,
	         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1 :
	                         NETSTACK_RDC.channel_check_interval()) );
	}

  process_start(&tcpip_process, NULL);

	if(IS_WAKEUP_MODE == false)	printf("Tentative link-local IPv6 address ");

  uip_ds6_addr_t *lladdr;
  int i;
  lladdr = uip_ds6_get_link_local(-1);

	if(IS_WAKEUP_MODE == false)
	{
	  for(i = 0; i < 7; ++i) 
		{
	    printf("%02x%02x:", lladdr->ipaddr.u8[i * 2],
	           lladdr->ipaddr.u8[i * 2 + 1]);
	  }
	  printf("%02x%02x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
	}

  if(!UIP_CONF_IPV6_RPL) {
    uip_ipaddr_t ipaddr;
    int i;
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    printf("Tentative global IPv6 address ");
    for(i = 0; i < 7; ++i) {
      printf("%02x%02x:",
             ipaddr.u8[i * 2], ipaddr.u8[i * 2 + 1]);
    }
    printf("%02x%02x\n",
           ipaddr.u8[7 * 2], ipaddr.u8[7 * 2 + 1]);
  }

#else /* NETSTACK_CONF_WITH_IPV6 */

  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  printf("%s %s, channel check rate %lu Hz, radio channel null\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1 :
                         NETSTACK_RDC.channel_check_interval()) );
#endif /* NETSTACK_CONF_WITH_IPV6 */



  //leds_off(LEDS_GREEN);

#if TIMESYNCH_CONF_ENABLED
  timesynch_init();
  timesynch_set_authority_level(linkaddr_node_addr.u8[0]);
#endif /* TIMESYNCH_CONF_ENABLED */

  energest_init();
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  /*
   * This is the scheduler loop.
   */
#if DCOSYNCH_CONF_ENABLED
  timer_set(&mgt_timer, DCOSYNCH_PERIOD * CLOCK_SECOND);
#endif
}

static void EXTI15_10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI lines 15 to 10 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

int app_main(void)
{

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

	/* MCU Configuration--------------------------------------------------------*/

	/* Initialize UART2 for debugging*/
	MX_USART3_UART_Init();
	Debug_RxReady();

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	__HAL_RCC_D2SRAM3_CLK_ENABLE();

	SystemCoreClockUpdate();

	SRAM_Init();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();


	/* Initialize UART2 for debugging*/
	MX_USART3_UART_Init();
	Debug_RxReady();

	MX_RFM_SPI_Init();

	/* print booting message */
	ApplicationBootingMsg();

	/* Initialize FLASH */
	ULP_Flash_Init();

	/*
	* MAC Initialized
	*/
	MAC_Init();

	/*
	* RFM Initialized
	*/
	RFM_Init();

	EXTI15_10_IRQHandler_Config();

#if defined(FEATURE_FIRENZE_TX_MODE)
	/*
	* Modem Initialized
	*/
	process_start(&tx_modem_process, NULL);

	/*
	* MAC Initialized
	*/
	process_start(&mac_process, NULL);

	/*
	* Application Initialized
	*/
	process_start(&tx_app_process, NULL);


#elif defined(FEATURE_FIRENZE_RX_MODE)

	/*
	* Application Initialized
	*/
	process_start(&rx_app_process, NULL);

	/*
	* ULP Initialized
	*/
	process_start(&ULP_process, NULL);

	/*
	* Modem Initialized
	*/
	process_start(&rx_modem_process, NULL);

#endif



	while(1)
	{

	  do 
		{

	  } while(process_run() > 0);

	  idle_count++;
	  /* Idle! */
	  /* Stop processor clock */
	  /* asm("wfi"::); */

	}

}




/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /**Supply configuration update enable
  */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);
  /**Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY)
  {

  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}


void SRAM_Init(void)
{
	/* Copy global symbol to sram */
	memcpy(&_s_sram_1, &_si_sram_1, (int)&_e_sram_1 - (int)&_s_sram_1);
	memcpy(&_s_sram_2, &_si_sram_2, (int)&_e_sram_2 - (int)&_s_sram_2);
	memcpy(&_s_sram_3, &_si_sram_3, (int)&_e_sram_3 - (int)&_s_sram_3);
}




void ApplicationBootingMsg(void)
{
	const char *build_time = __DATE__ ", " __TIME__;

	const char *rfm_hw_version = "HW v2";


	printf("\n*****************************************************\n");
	printf("  Build Date : %s\n", build_time);
	printf("  Core Type : STM32H743ZI\n");
	printf("  Application section : 0x%x\n", (unsigned int)SCB->VTOR);
	printf("  System Core Clock : %lu\n", SystemCoreClock);
	printf("  RF B`D Version : %s\n", rfm_hw_version);
	printf("*****************************************************\n\n");
	//putCh('>');		putCh('>');		putCh(0x20);			// print prompt
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	printf("error\n");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
