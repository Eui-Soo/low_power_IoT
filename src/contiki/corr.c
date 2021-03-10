/*
 * corr.c
 *
 *  Created on: 2020. 8. 30.
 *      Author: euisoo
 */


#include <stdlib.h>
#include "modem.h"
#include "rfm_def.h"
extern _ULP_FLASH_PARAM ulp_flash_param;
unsigned short corr(unsigned char *rx_data, uint32_t len) {

	//char corr[4096*8-64] = { 0 };
	char corr[RFM_RX_DATA_BIT_LEN_MAX] = { 0 };
	unsigned char preamble[8] = { 0x2C,0x1F,0x5C,0x20,0xAB,0x1A,0x2B,0x90 };
	unsigned char rx_buf_tmp[64] = { 0x00 };
	unsigned char rx_buf[8] = { 0x00 };
	unsigned char tmp,tmp2;
	unsigned int max_tmp = 0;
	unsigned short sync_pos, min_sp = 0xFFFF, max_sp;

	for (int i = 0; i < len; i++)
	{
		//printf("[%d] ", i);

		for(int j = 0; j<8; j++)
		{	rx_buf[j] = (((rx_data[i + (j*8)] >=ulp_flash_param.decision_val) & 0x01) << 7) & 0x80;
			rx_buf[j] += (((rx_data[i + (j*8)+1] >=ulp_flash_param.decision_val) & 0x01) << 6) & 0x40;
			rx_buf[j] += (((rx_data[i + (j*8)+2] >=ulp_flash_param.decision_val) & 0x01) << 5) & 0x20;
			rx_buf[j] += (((rx_data[i + (j*8)+3] >=ulp_flash_param.decision_val) & 0x01) << 4) & 0x10;
			rx_buf[j] += (((rx_data[i + (j*8)+4] >=ulp_flash_param.decision_val) & 0x01) << 3) & 0x08;
			rx_buf[j] += (((rx_data[i + (j*8)+5] >=ulp_flash_param.decision_val) & 0x01) << 2) & 0x04;
			rx_buf[j] += (((rx_data[i + (j*8)+6] >=ulp_flash_param.decision_val) & 0x01) << 1) & 0x02;
			rx_buf[j] += (rx_data[i + (j*8)+7] >=ulp_flash_param.decision_val) & 0x01;
#if(0)
			//printf("%d [%d: %2x] ",i,  j, rx_buf[j]);

			printf("%d ",rx_buf_tmp[(j*8)]);
			printf("%d ",rx_buf_tmp[(j*8)+1]);
			printf("%d ",rx_buf_tmp[(j*8)+2]);
			printf("%d ",rx_buf_tmp[(j*8)+3]);
			printf("%d ",rx_buf_tmp[(j*8)+4]);
			printf("%d ",rx_buf_tmp[(j*8)+5]);
			printf("%d ",rx_buf_tmp[(j*8)+6]);
			printf("%d ",rx_buf_tmp[(j*8)+7]);
#endif
			//printf("%02x",rx_buf[j]);

            tmp = ~(preamble[j]^rx_buf[j]);
            tmp2 = (tmp >> 7) & 0x01;
            tmp2 += (tmp >> 6) & 0x01;
            tmp2 += (tmp >> 5) & 0x01;
            tmp2 += (tmp >> 4) & 0x01;
            tmp2 += (tmp >> 3) & 0x01;
            tmp2 += (tmp >> 2) & 0x01;
            tmp2 += (tmp >> 1) & 0x01;
            tmp2 += (tmp) & 0x01;
            corr[i] = corr[i] + tmp2;
		}

		//printf("\t");
#if(1)
		if(corr[i] == 64)
		{
			printf("\n[%d] ",i);
			for(int a = 0; a < 8; a++)
				printf(" %2x", rx_buf[a]);
			printf("\n");
			sync_pos = i;
			break;
		}

#else
		 corr[i] = corr[i] + corr[i] - 64;
		 if (corr[i] < 0)
			corr[i] = corr[i] * (-1);
		 if (corr[i] >= max_tmp)
		 {
			max_tmp = corr[i];
			sync_pos = i;
		 }
#endif

      }

      return sync_pos;
}
