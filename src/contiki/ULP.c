/* RF radio driver */
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* ULP radio driver */
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "dev/watchdog.h"

#include "dev/leds.h"

#include "modem.h"
#include "contiki.h"


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ULP.h"
#include "ipc.h"
#include "process.h"
#include "rfm_def.h"
#include "rfm.h"


#define APPENDIX_LEN 2
#define RF_ON 0x02
#define RF_TX_ACTIVE 0x08
#define RF_UPDATE_CHANNEL 0x10
#define RF_POLL_RX_INTERRUPT 0x20
#define RF_RX_PROCESSING_PKT 0x40
#define ULP_RXFIFO 0x3F
#define PHR_LEN 1
#define ACK_LEN 3
#define CC_APPENDIX_LEN                 2
#define ULP_NUM_RXBYTES 0x2FD7
#define CC1200_LQI_VAL 0x2F74
#define CC1200_RSSI1 0x2F71
#define ADDR_CHECK_OK 2
#define ADDR_CHECK_OK_ACK_SEND 3
#define ULP_USE_RX_WATCHDOG 0

//#define TEST_MODEM

#ifdef CC1200_CONF_MAX_PAYLOAD_LEN
#define CC1200_MAX_PAYLOAD_LEN           CC1200_CONF_MAX_PAYLOAD_LEN
#else
#define CC1200_MAX_PAYLOAD_LEN           500
#endif

#define LOCK_SPI() 0
#define SPI_IS_LOCKED() 0
#define RELEASE_SPI() 0

/*---------------------------------------------------------------------------*/
/* Variables */
/*---------------------------------------------------------------------------*/
/* Flag indicating whether non-interrupt routines are using SPI */
static volatile uint8_t spi_locked = 0;
/* Packet buffer for transmission, filled within prepare() */
static uint8_t tx_pkt[CC1200_MAX_PAYLOAD_LEN];
/* The number of bytes waiting in tx_pkt */
static uint16_t tx_pkt_len;
/* Packet buffer for reception */
static uint8_t rx_pkt[CC1200_MAX_PAYLOAD_LEN + APPENDIX_LEN];
/* The number of bytes placed in rx_pkt */
static volatile uint16_t rx_pkt_len = 0;
/*
 * The current channel in the range CC1200_RF_CHANNEL_MIN
 * to CC1200_RF_CHANNEL_MAX
 */
static uint8_t rf_channel;
/* The next channel requested */
static uint8_t new_rf_channel;
/* RADIO_PARAM_RX_MODE. Initialized in init() */
static radio_value_t rx_mode_value;
/* RADIO_PARAM_RX_MODE. Initialized in init() */
static radio_value_t tx_mode_value;
/* RADIO_PARAM_TXPOWER in dBm. Initialized in init() */
static int8_t txpower;
static int8_t new_txpower;
/* RADIO_PARAM_CCA_THRESHOLD. Initialized in init() */
static int8_t cca_threshold;
static int8_t new_cca_threshold;
/* The radio drivers state */
static uint8_t rf_flags = 0;
#if !CC1200_AUTOCAL && CC1200_CAL_TIMEOUT_SECONDS
/* Use a timeout to decide when to calibrate */
static unsigned long cal_timer;
#endif
#if CC1200_USE_RX_WATCHDOG
/* Timer used for RX watchdog */
static struct etimer et;
#endif /* #if CC1200_USE_RX_WATCHDOG */


/*---------------------------------------------------------------------------*/
#if defined(FEATURE_FIRENZE_RX_MODE)	
extern uint8_t ferData[RFM_DATA_LEN];
extern uint8_t ferData2[8+24];
extern bool bFerModeOn;
extern uint16_t ferRecvCounter;
extern uint16_t ferMissCounter;
extern uint8_t rfm_preamble_cycle;


extern void RFM_RX_Start(void);
#endif
extern _ULP_FLASH_PARAM ulp_flash_param;

/*---------------------------------------------------------------------------*/
/* Restart RX from within RX interrupt. */
static void
rx_rx(void)
{

}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static int
addr_check_auto_ack(uint8_t *frame, uint16_t frame_len)
{
  return ADDR_CHECK_OK;
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Read a single byte from the specified address. */
static uint8_t
single_read(uint16_t addr)
{
  return 1;
  
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* Update rf channel if possible, else postpone it (->pollhandler) */
static int
set_channel(uint8_t channel)
{

}
/*---------------------------------------------------------------------------*/

int len;
//extern unsigned char size;
/*---------------------------------------------------------------------------*/
/* Prototypes for Netstack API radio driver functions */
/*---------------------------------------------------------------------------*/
/* Init the radio. */
static int
init(void);
/* Prepare the radio with a packet to be sent. */
static int
prepare(const void *payload, unsigned short payload_len);
/* Send the packet that has previously been prepared. */
static int
transmit(unsigned short payload_len);
/* Prepare & transmit a packet. */
static int
send(const unsigned char *payload, unsigned short payload_len);
/* Read a received packet into a buffer. */
static int
read(unsigned char *buf, unsigned short bufsize);

static int
gunho_read(uint8_t *data, uint16_t *buf, unsigned short buf_len);

/*
 * Perform a Clear-Channel Assessment (CCA) to find out if there is
 * a packet in the air or not.
 */
static int
channel_clear(void);
/* Check if the radio driver is currently receiving a packet. */
static int
receiving_packet(void);
/* Check if the radio driver has just received a packet. */
static int
pending_packet(void);
/* Turn the radio on. */
static int
on(void);
/* Turn the radio off. */
static int
off(void);
/* Get a radio parameter value. */
static radio_result_t
get_value(radio_param_t param, radio_value_t *value);
/* Set a radio parameter value. */
static radio_result_t
set_value(radio_param_t param, radio_value_t value);
/* Get a radio parameter object. */
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size);
/* Set a radio parameter object. */
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size);
static int
getAbit(unsigned char payload, int n);
void 
ArraygetAbit(unsigned char* x, int cnt, unsigned char *y);
void
ArraygetAHex(unsigned char* x, int cnt, unsigned char *y);


/*---------------------------------------------------------------------------*/
/* The radio driver exported to contiki */
/*---------------------------------------------------------------------------*/
const struct radio_driver ULP_driver = {
  init,
  prepare,
  transmit,
  send,
  read,
  gunho_read,
  channel_clear,
  receiving_packet,
  pending_packet,
  on,
  off,
  get_value,
  set_value,
  get_object,

  set_object
};


/*---------------------------------------------------------------------------*/
/* Handle tasks left over from rx interrupt or because SPI was locked */
static void pollhandler(void);
/*---------------------------------------------------------------------------*/
PROCESS(ULP_process, "ULP driver");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ULP_process, ev, data)
{
#if 0
  ev = PROCESS_EVENT_POLL;
	PROCESS_POLLHANDLER(pollhandler());

	PROCESS_BEGIN();
  printf("ULP process!!!!!!\n");

#if ULP_USE_RX_WATCHDOG
	while(1) {
		if ((rf_flags & (RF_ON | RF_TX_ACTIVE)) == RF_ON) {

			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

			if(ULP_arch_gpio0_read_pin() == 0) {
				LOCK_SPI();
				if(state() != STATE_RX) {
					printf("RF: RX watchdog triggered!\n");
					rx_rx();
				}
				RELEASE_SPI();
			}
		} else {
			PROCESS_YIELD();
		}
	}
#endif

	PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_EXIT);

	PROCESS_END();
#else
	extern bool ferTestOn;

	PROCESS_BEGIN();

	while(1)
	{
		
		/* Wait untill recieved event from MAC Layer */
		PROCESS_WAIT_EVENT();

		/* Get payload length from IPC length field */
		//payload_len = IPC_GET_LEN(ipc);

		/* Copy payload  from IPC databuffer */
		//memcpy(payload, IPC_GET_DATA_PTR(ipc), IPC_GET_LEN(ipc));

		printf("[From Modem to MAC] payload_len = %d bytes\n", rx_pkt_len);

		if(rx_pkt_len > 0) 
		{
			int len;
			packetbuf_clear();
			len = read(packetbuf_dataptr(), PACKETBUF_SIZE);

			if(len>0) 
			{
				packetbuf_set_datalen(len);
				NETSTACK_RDC.input();		
			}
			else
			{
				rx_pkt_len = 0;
			}

#if defined(FEATURE_FIRENZE_RX_MODE)
			RFM_RX_Start();
#endif

			if(ulp_flash_param.sleep)
			{
				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
				SystemClock_Config();
			}
		}
	}

	PROCESS_END();

#endif
}

/*---------------------------------------------------------------------------*/
static void
pollhandler(void)
{
  printf("pollhandler!!!!\n");
	if((rf_flags & (RF_ON + RF_POLL_RX_INTERRUPT)) ==
			(RF_ON + RF_POLL_RX_INTERRUPT)) {
		ULP_rx_interrupt();
	}
	if(rf_flags & RF_UPDATE_CHANNEL) {
		set_channel(new_rf_channel);
	}
	if(rx_pkt_len > 0) {
		int len;
		packetbuf_clear();
		len = read(packetbuf_dataptr(), PACKETBUF_SIZE);

		if(len>0) {
			packetbuf_set_datalen(len);
			NETSTACK_RDC.input();
		}
	}
}


/* Initialize radio. */
static int
init(void)
{
  printf("init\n");
  process_start(&ULP_process, NULL);
  return 0;
}

/*---------------------------------------------------------------------------*/
/* Prepare the radio with a packet to be sent. */
static int
prepare(const void *payload, unsigned short payload_len)
{
  printf("prepare\n");
  return 0;
}

/*---------------------------------------------------------------------------*/
/* Send the packet that has previously been prepared. */
static int
transmit(unsigned short transmit_len)
{
  printf("transmit\n");
  return 0;
}
/*---------------------------------------------------------------------------*/


int getAbit(unsigned char x, int n)
{
   return (x&(1 << n)) >> n;
}

/*bites ?�위�?bytes�?변�? */
void ArraygetAbit(unsigned char* x, int cnt, unsigned char *y)
{
   
   int index = 0;

   for (int i = 0; i < cnt; i++)
   {
      for (int r = 7; r >= 0; r--)
      {
         y[index++] = getAbit(x[i], r);
      }
   }

}
/*bytes ?�위�?bits ?�위�?변�?ex) data = 1101 0111(HEX = 0xdb) > 0000 0001 0000 0001 0000 0000 0000 0001 0000 0000 0000 00001 0000 0001 0000 00001 */
void ArraygetAHex(unsigned char* x, int cnt, unsigned char *y) // cnt = payload_len*8
{
	int i;
	unsigned char j;
	unsigned char bit=7;
	char temp = 0x00;


	for (i=0; i<cnt/8; i++){
		for (j=0; j<8; j++){
			temp = temp|(x[i*8+j]<<bit-j);
		}
		y[i] = temp;
		temp=0;
	}



}

/* Prepare & transmit a packet. */
/******************************************** Modem ?�신부 *************************************************************/
static int
send(const unsigned char *payload, unsigned short payload_len)
{
	extern struct process tx_modem_process;
	IPC_PKT ipc;

	/* Set payload length to IPC length field*/
	IPC_SET_LEN(&ipc, payload_len);

	/* Copy payload to IPC data buffer */
	memcpy(ipc.data, payload, payload_len);

	printf("\n[MAC] data length = %d bytes \n", ipc.hdr.msg_len);
	
	for(uint8_t i = 0; i < payload_len; i++)
	{
		printf("0x%02x", payload[i]);

		if((i + 1) % 8 == 0)	printf("\n");
		else						printf("  ");
	}
	printf("\n");

	/* Send to modem by IPC */	
	process_post_synch(&tx_modem_process, PROCESS_EVENT_MSG, &ipc);

  return 0;
}


/*---------------------------------------------------------------------------*/
/* Read a received packet into a buffer. */
static int
read(unsigned char *buf, unsigned short buf_len)
{

  int len = 0;
	int i;

  if(rx_pkt_len > 0) {

    int8_t rssi = rx_pkt[rx_pkt_len - 2];
    /* CRC is already checked */
    uint8_t crc_lqi = rx_pkt[rx_pkt_len - 1];

    //len = rx_pkt_len - APPENDIX_LEN;
    len = rx_pkt_len;

    if(len > buf_len) {

      printf("RF: Failed to read packet (too big)!\n");

    } else {

      //printf("[MAC] RF: Read (%d bytes, %d dBm)\n", len, rssi);
     // printf("[MAC] RF: Read (%d bytes) = \n", len);

      memcpy((void *)buf, (const void *)rx_pkt, len);

			for(i = 0 ; i < len; i++)
			{	
				printf("0x%02x", buf[i]);

				if((i+1)%8 == 0)
					printf("\n");
				else
					printf("  ");
			}
			printf("\n\n");

#if defined(FEATURE_FIRENZE_RX_MODE)	
			if(bFerModeOn == true)
			{	
				if(memcmp(ferData, buf, len))	ferMissCounter++;
				else												ferRecvCounter++;
				
				printf("OK : %03d / Error : %03d\n", ferRecvCounter, ferMissCounter); 
				return 0;
			}
#endif

      /* Release rx_pkt */
      rx_pkt_len = 0;

      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);
      /* Mask out CRC bit */
      packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY,
                         crc_lqi & ~(1 << 7));

      RIMESTATS_ADD(llrx);
    }

  }

  return len;

}



/*---------------------------------------------------------------------------*/
/*
 * Perform a Clear-Channel Assessment (CCA) to find out if there is a
 * packet in the air or not.
 */
static int
channel_clear(void)
{
  printf("channel_clear\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/*
 * Check if the radio driver is currently receiving a packet.
 *
 * nullrdc uses this function
 * - to detect a collision before transmit()
 * - to detect an incoming ACK
 */
static int
receiving_packet(void)
{
  printf("receiving_packet\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Check if the radio driver has just received a packet. */
static int
pending_packet(void)
{
  printf("pending_packet\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Turn the radio on. */
static int
on(void)
{
  //printf("on\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Turn the radio off. */
static int
off(void)
{
  //printf("off\n");
  return 0;
}

/*---------------------------------------------------------------------------*/
/* Get a radio parameter value. */
static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
  printf("get_value\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Set a radio parameter value. */
static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
  printf("set_value\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Get a radio parameter object. */
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
  printf("get_object\n");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Set a radio parameter object. */
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
  printf("set_object\n");
  return 0;
}

/*---------------------------------------------------------------------------*/
/* Write a burst of bytes starting at the specified address. */
static void
burst_write(uint16_t addr, const uint8_t *data, uint8_t data_len)
{

  /*cc1200_arch_spi_select();
  if(CC1200_IS_EXTENDED_ADDR(addr)) {
    cc1200_arch_spi_rw_byte(CC1200_EXTENDED_BURST_WRITE_CMD);
    cc1200_arch_spi_rw_byte((uint8_t)addr);
  } else {
    cc1200_arch_spi_rw_byte(addr | CC1200_WRITE_BIT | CC1200_BURST_BIT);
  }
  cc1200_arch_spi_rw(NULL, data, data_len);
  cc1200_arch_spi_deselect();
   */
}
/*---------------------------------------------------------------------------*/
/* Read a burst of bytes starting at the specified address. */
static void
burst_read(uint16_t addr, uint8_t *data, uint8_t data_len)
{

  /* cc1200_arch_spi_select();
  if(CC1200_IS_EXTENDED_ADDR(addr)) {
    cc1200_arch_spi_rw_byte(CC1200_EXTENDED_BURST_READ_CMD);
    cc1200_arch_spi_rw_byte((uint8_t)addr);
  } else {
    cc1200_arch_spi_rw_byte(addr | CC1200_READ_BIT | CC1200_BURST_BIT);
  }
  cc1200_arch_spi_rw(data, NULL, data_len);
  cc1200_arch_spi_deselect();
  */
}
/*---------------------------------------------------------------------------*/







/*---------------------------------------------------------------------------*/
/*
 * The CC1200 interrupt handler: called by the hardware interrupt
 * handler, which is defined as part of the cc1200-arch interface.
 */
int
ULP_rx_interrupt(void)
{

  printf("ULP_rx_interrupt!!!!!!!!!!!\n");
  /* The radio's state */
  uint8_t s;
  /* The number of bytes in the RX FIFO waiting for read-out */
  uint8_t num_rxbytes;
  /* The payload length read as the first byte from the RX FIFO */
  static uint16_t payload_len;
  /*
   * The number of bytes already read out and placed in the
   * intermediate buffer
   */
  static uint16_t bytes_read;
  /*
   * We use an intermediate buffer for the packet before
   * we pass it to the next upper layer. We also place RSSI +
   * LQI in this buffer
   */
  static uint8_t buf[CC1200_MAX_PAYLOAD_LEN + APPENDIX_LEN];

  if(SPI_IS_LOCKED()) {

    /*
     * SPI is in use. Exit and make sure this
     * function is called from the poll handler as soon
     * as SPI is available again
     */

    rf_flags |= RF_POLL_RX_INTERRUPT;
    process_poll(&ULP_process);
    return 1;

  }
  rf_flags &= ~RF_POLL_RX_INTERRUPT;

  LOCK_SPI();

  /*
   * If CC1200_USE_GPIO2 is enabled, we come here either once RX FIFO
   * threshold is reached (GPIO2 rising edge)
   * or at the end of the packet (GPIO0 falling edge).
   */

  /* Make sure we are in a sane state. Sane means: either RX or IDLE */
#if 0
  /* In initial ULP chip, the information about states may not exist. */
  s = state();
  if((s == STATE_RX_FIFO_ERR) || (s == STATE_TX_FIFO_ERR)) {

    rx_rx();
    RELEASE_SPI();
    return 0;

  }
#endif

#if 0
  /* There is no RX buffer in ULP chip */
  num_rxbytes = single_read(ULP_NUM_RXBYTES);

  if(num_rxbytes == 0) {

    /*
     * This might happen from time to time because
     * this function is also called by the pollhandler and / or
     * from TWO interrupts which can occur at the same time.
     */

    printf("RF: RX FIFO empty!\n");
    RELEASE_SPI();
    return 0;

  }
#else
  num_rxbytes = single_read(ULP_NUM_RXBYTES);
#endif

#if 0
  /* Checking validity of received packet.*/
  if(!(rf_flags & RF_RX_PROCESSING_PKT)) {

    /* Read first byte in RX FIFO (payload length) */
    burst_read(ULP_RXFIFO,
               (uint8_t *)&payload_len,
               PHR_LEN);

    if(payload_len < ACK_LEN) {
      /* Packet to short. Discard it */
      printf("RF: Packet too short!\n");
      RIMESTATS_ADD(tooshort);
      rx_rx();
      RELEASE_SPI();
      return 0;
    }

    if(payload_len > CC1200_MAX_PAYLOAD_LEN) {
      /* Packet to long. Discard it */
      printf("RF: Packet to long!\n");
      RIMESTATS_ADD(toolong);
      rx_rx();
      RELEASE_SPI();
      return 0;
    }

    RX_LEDS_ON();
    bytes_read = 0;
    num_rxbytes -= PHR_LEN;

    rf_flags |= RF_RX_PROCESSING_PKT;

    /* Fall through... */

  }
#else
  bytes_read = 0;
  num_rxbytes -= PHR_LEN;
  rf_flags |= RF_RX_PROCESSING_PKT;
#endif

  if(rf_flags & RF_RX_PROCESSING_PKT) {

#if 0
	  /* Check packet length matches with RX buffer length
	   * NOTE implement if ULP chip support this function
	   */

    /*
     * Read out remaining bytes unless FIFO is empty.
     * We have at least num_rxbytes in the FIFO to be read out.
     */

    if((num_rxbytes + bytes_read) > (payload_len + CC_APPENDIX_LEN)) {

      /*
       * We have a mismatch between the number of bytes in the RX FIFO
       * and the payload_len. This would lead to an buffer overflow,
       * so we catch this error here.
       */

      printf("RF: RX length mismatch %d %d %d!\n", num_rxbytes,
              bytes_read,
              payload_len);
      rx_rx();
      RELEASE_SPI();
      return 0;

    }
#endif

    /* NOTE implement packet read for ULP */
/*     burst_read(ULP_RXFIFO,
               &buf[bytes_read],
               num_rxbytes); */


 /* RF read add*/

#if defined(FEATURE_FIRENZE_RX_MODE)
//static unsigned char readBuffer[RFM_DATA_BIT_LEN]={0};
    //static unsigned char readBuffer[RFM_RX_DATA_BIT_LEN_MAX]={0};
    static uint16_t readBuffer[RFM_RX_DATA_BIT_LEN_MAX]={0};
    int receive_len = 0;

	while(RFM_RX_GetDataBufState()==false);
	receive_len = RFM_RX_GetData(readBuffer);
	//num_rxbytes = gunho_read(&buf[bytes_read], readBuffer, RFM_DATA_BIT_LEN); //len
	num_rxbytes = gunho_read(&buf[bytes_read], readBuffer, receive_len); //len

	//printf("num_rxbyte is %d\n", num_rxbytes);

	bytes_read = num_rxbytes;
	num_rxbytes = 0;
#endif

  /***********************************/
#if 0
    if(bytes_read == (payload_len + CC_APPENDIX_LEN))
#else
    if (1)
#endif
    {
      /*
       * End of packet. Read appendix (if available), check CRC
       * and copy the data from temporary buffer to rx_pkt
       * RSSI offset already set using AGC_GAIN_ADJUST.GAIN_ADJUSTMENT
       */

#if APPEND_STATUS
      uint8_t crc_lqi = buf[bytes_read - 1];
#else
      int8_t rssi = single_read(CC1200_RSSI1);
      uint8_t crc_lqi = single_read(CC1200_LQI_VAL);
#endif

#if 0 
      if(!(crc_lqi & (1 << 7))) {
        /* CRC error. Drop the packet */
        printf("RF: CRC error!\n");
        RIMESTATS_ADD(badcrc);
      } else if(rx_pkt_len != 0) {
        /* An old packet is pending. Drop the packet */
        printf("RF: Packet pending!\n");
      } else {
#endif

      int ret = addr_check_auto_ack(buf, bytes_read);

  		//printf("4\n");
      if ((ret == ADDR_CHECK_OK) ||
          (ret == ADDR_CHECK_OK_ACK_SEND))
      {
#if APPEND_STATUS
        /* RSSI + LQI already read out and placed into buf */
#else
        //buf[bytes_read++] = (uint8_t)rssi;
        //buf[bytes_read++] = crc_lqi;
#endif

        rx_pkt_len = bytes_read;
        memcpy((void *)rx_pkt, buf, rx_pkt_len);
        rx_rx();
        //process_start(&ULP_process, NULL);
        // process_poll(&ULP_process); 
        RELEASE_SPI();
        return 1;
      }
      else
      {
        /* Invalid address. Drop the packet */
      }
#if 0
      }
#endif

      /* Buffer full, address or CRC check failed */
      rx_rx();
      RELEASE_SPI();
      return 0;

    } /* if (bytes_read == payload_len) */
  }

  RELEASE_SPI();
  return 0;

}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* Read a received packet into a buffer. */
/******************************************** Modem ?�신부 *************************************************************/
static int
gunho_read(uint8_t *data, uint16_t *buf, unsigned short buf_len) // netstack_radio.read()
//gunho_read(uint8_t *data, unsigned char *buf, unsigned short buf_len) // netstack_radio.read()
{

  unsigned char buf_tmp[SIZE*2]={0};
  unsigned char buf_desp[SIZE*2]={0};
  unsigned char buf_viter[SIZE]={0};
  unsigned char sig2[SIZE*2]={0};
  //unsigned char count[1984]={0};
  unsigned char count[3968]={0};
  unsigned short peak_pos = 0;

  //unsigned char desp[SIZE*2]={0};
  printf("\n[MODEM] Recieved Data from PHY = %d byte\n", buf_len);
#if(0)
  for (int i = 0; i < buf_len; i++)
   {
     printf("%03d ", buf[i]);
     if ((i + 1) % 8 == 0)
     {
       if ((i + 1) % 64 == 0)
         printf("\n");
       else
         printf("  ");
     }
   }
 	printf("\n");

#endif
/*
  	if(ulp_flash_param.duty_cycle)
  	{
  		peak_pos=corr(buf, buf_len);
		printf("packet detection = %d\n", peak_pos);
		peak_pos += RFM_PRM_BIT_LEN;
	}
	*/
  	peak_pos += RFM_PRM_BIT_LEN;

	printf("\n");

	printf("Decision value = %d\n", ulp_flash_param.decision_val);
	printf("\n");


	if(ulp_flash_param.pn_len != 1)
	{
	  for (int i = 0; i < SIZE*2*SG; i++)
	  {
		count[buf[peak_pos+i]]++;
		/*
		    if(buf[peak_pos+i]>=ulp_flash_param.decision_val)		buf_tmp[i]=7;
		    else						buf_tmp[i]=0;

		*/

		if(buf[peak_pos+i]>=(ulp_flash_param.decision_val+54))		buf_tmp[i]=7;
		else if(buf[peak_pos+i]<(ulp_flash_param.decision_val+54) && buf[peak_pos+i]>=(ulp_flash_param.decision_val+36))		buf_tmp[i]=6;
		else if(buf[peak_pos+i]<(ulp_flash_param.decision_val+36) && buf[peak_pos+i]>=(ulp_flash_param.decision_val+18))		buf_tmp[i]=5;
		else if(buf[peak_pos+i]<(ulp_flash_param.decision_val+18) && buf[peak_pos+i]>=(ulp_flash_param.decision_val))		buf_tmp[i]=4;
		else if(buf[peak_pos+i]<(ulp_flash_param.decision_val) && buf[peak_pos+i]>=(ulp_flash_param.decision_val-18))		buf_tmp[i]=3;
		else if(buf[peak_pos+i]<(ulp_flash_param.decision_val-18) && buf[peak_pos+i]>=(ulp_flash_param.decision_val-36))		buf_tmp[i]=2;
		else if(buf[peak_pos+i]<(ulp_flash_param.decision_val-36) && buf[peak_pos+i]>=(ulp_flash_param.decision_val-54))		buf_tmp[i]=1;
		else						buf_tmp[i]=0;

		/*
	    if(buf[i]>=55)
	    	buf[i]=7;
	    else if(buf[i]>55||buf[i]<=47)
	    	buf[i]=6;
	    else if(buf[i]>47||buf[i]<=39)
	    	buf[i]=5;
	    else if(buf[i]>39||buf[i]<=31)
	    	buf[i]=4;
	    else if(buf[i]>31||buf[i]<=23)
	    	buf[i]=3;
	    else if(buf[i]>23||buf[i]<=15)
	        buf[i]=2;
	    else if(buf[i]>15||buf[i]<=7)
	        buf[i]=1;
	    else
	    	buf[i]=0;
	    	*/

	  }
	  //De_spreading(buf,desp);
	  if(ulp_flash_param.display_count)
	  {
		  for ( int i = 800; i<1000; i++)
		  {
			  printf("count[%d] : %d\n", i+1, count[i]);
		  }

	  }
#if defined(TEST_MODEM)
  char sig1[SIZE*2]=
  {
  	0,1,1,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,
		0,1,0,1,0,0,0,0,0,0,0,0,1,0,0,1,0,0,1,0,0,
		1,0,0,0,0,0,0,0,0,0,1,0,1,1,1,0,0,0,1,1,0,0,
		0,0,0,0,0,0,0,1,0,0,1,0,1,0,1,1,0,0,0,0,0,0,
		1,1,0,0,1,1,1,0,0,1,1,0,0,0,0,0,0,0,1,0,0,0,
		1,1,0,1,0,0,1,0,0,0,0,0,0,0,1,1,0,1,1,1,0,1,
		1,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,0,1,1,1,0,
		0,0,0,0,0,1,1,0,0,0,1,1,0,0,0,1,1,1,0,0,0,0,
		0,1,1,1,0,1,0,1,1,0,1,1,1,1,0,0,0,0,0,1,1,1,
		1,1,0,1,1,0,1,1,1,1,0,0,0,0,0,0,0,1,1,1,1,0,
		0,1,1,1,1,0,0,0,0,0,0,1,0,1,0,0,0,1,1,1,1,1,
		1,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,1,1,0,0,0,
		0,0,0,0,0,0,0,0,0,0,1,0,1,1,1,0,0,0,0,0,0,0,
		0,0,1,0};
	  char sig2[SIZE*2]={0};
	  int count2=0;
	  printf("\n");
	  for (int i = 0; i < SIZE*2; i++)
		{

		  if(buf[i]>=45)
			sig2[i]=1;
		  else
			sig2[i]=0;

		  if(sig2[i]!=sig1[i])
			  count2=count2+1;
			printf("%d ", sig2[i]);
			 if ((i + 1) % 8 == 0)
			 {
			   if ((i + 1) % 64 == 0)
				 printf("\n");
			   else
				 printf("  ");
			 }
		}
	  printf("err count : %d",count2);
	  printf("\n");
	  for (int i = 0; i < SIZE*2; i++)
	  {
		  count[buf[i]]=count[buf[i]]+1;
	  }
	  for (int i = 0; i < 64; i++)
	  {
		  printf("%d : %d\n",i,count[i]);
	  }
	  printf("\n");
	  printf("%d %d",sig2[0],sig1[0]);
	  printf("\n");
	  for (int i = 0; i < SIZE*2; i++)
		  {
			  if(i != 0 && (i % 8 == 0)) printf(" ");
			  if(i != 0 && (i % 64 == 0)) printf("\n");
			  printf("%02d ", abs(sig2[i]-sig1[i]));
		  }
#endif


	#if 1
	  /* ??��??과정 �??�인 */
	  //printf("\ndeinterleaver_data = \n");
	  deinterleaver(buf_tmp, buf_desp);
	/* for (int i = 0; i < SIZE*2; i++)
	  {
		printf("%d", buf_desp[i]);
		if ((i + 1) % 8 == 0)
		{
		  if ((i + 1) % 64 == 0)
			printf("\n");
		  else
			printf("  ");
		}
	  }
	  printf("\n");
	*/
	/*
	#if !defined(MULTIBIT_DECISON_IN_MODEM)
	  printf("\nQuantization = \n");
	  Quantization(buf_desp, sig2);
	  for (int i = 0; i <SIZE*2/SG; i++)
	  {
		printf("%d", sig2[i]);
		if ((i + 1) % 8 == 0)
		{
		  if ((i + 1) % 64 == 0)
			printf("\n");
		  else
			printf("  ");
		}
	  }

	  printf("\n");
	#endif
	*/

	  /* viterbi 과정 (복호??과정)�??�인 */
	  //printf("\nbuf_viter= \n");
	  Viterbi_decoder(buf_desp, buf_viter);
	  /*for (int i = 0; i < ( SIZE * SG); i++)
	  {
		printf("%d", buf_viter[i]);
		if ((i + 1) % 8 == 0)
		{
		  if ((i + 1) % 64 == 0)
			printf("\n");
		  else
			printf("  ");
		}
	  }
	*/

	  printf("\n\n");

/*
	printf("Binary to Hex = \n");
	for (int i = 0; i < (SIZE / (8 * SG)); i++)
  {
    printf("0x%02x", data[i]);

		if((i + 1) % 8 == 0)	printf("\n");
		else								printf(" ");
  }
	printf("\n\n");
*/
#endif
	}
	else
	{
		for (int i = 0; i < SIZE*2*SG; i++)
	    {
			count[buf[peak_pos+i]]++;
			 if(buf[peak_pos+i]>=ulp_flash_param.decision_val)		buf_viter[i]=1;
			 else						buf_viter[i]=0;
	    }

		if(ulp_flash_param.display_count)
		{
			for ( int i = 0; i<=31; i++)
			{
			  printf("count[%d] : %d\n", i+1, count[i]);
			}
		}
		printf("\n\n");
	}

	  /* bites ?�위 ?�이?��? bytes ?�위 ?�이?�로 변�?�??�인 */
	  ArraygetAHex(buf_viter, SIZE * SG,data);

  //printf("\n\nModem success\n\n");

  return (SIZE / (8 * SG));
}
