#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include "modem.h"

//#if defined(MULTIBIT_DECISON_IN_MODEM)
#if(1)
#define tblen 30
//#define DYNAMIC_ALLOC

void Viterbi_decoder(char *data, char *y_out){
	char t, next_state, States = 64;
	char input_bit=0, before_state = 1, path_metric = 2, flag = 3, interv = 4;
	char two_bit[2];
	char branch_metric;
	int total_path, max_metric;

	char loop =  tblen + (2*num);	
	int l,m,n,nn;
	char data_tail[(2*SIZE)+(4*num)];
	float block;

	char nextState[64][2] = {{ 0, 32 },{ 0, 32 },{ 1, 33 },{ 1, 33 },{ 2, 34 },{ 2, 34 },{ 3, 35 },{ 3, 35 },{ 4, 36 },{ 4, 36 },{ 5, 37 },{ 5, 37 },{ 6, 38 },{ 6, 38 },{ 7, 39 },{ 7, 39 },{ 8, 40 },
	{ 8, 40 },{ 9, 41 },{ 9, 41 },{ 10, 42 },{ 10, 42 },{ 11, 43 },{ 11, 43 },{ 12, 44 },{ 12, 44 },{ 13, 45 },{ 13, 45 },{ 14, 46 },{ 14, 46 },{ 15, 47 },{ 15, 47 },
	{ 16, 48 },{ 16, 48 },{ 17, 49 },{ 17, 49 },{ 18, 50 },{ 18, 50 },{ 19, 51 },{ 19, 51 },{ 20, 52 },{ 20, 52 },{ 21, 53 },{ 21, 53 },{ 22, 54 },{ 22, 54 },{ 23, 55 },{ 23, 55 },
	{ 24, 56 },{ 24, 56 },{ 25, 57 },{ 25, 57 },{ 26, 58 },{ 26, 58 },{ 27, 59 },{ 27, 59 },{ 28, 60 },{ 28, 60 },{ 29, 61 },{ 29, 61 },{ 30, 62 },{ 30, 62 },{ 31, 63 },{ 31, 63 } };

#if 0
	char outputs[64][4] = {{ 0, 0, 31, 31 },{ 31, 31, 0, 0 },{ 31, 0, 0, 31 },{ 0, 31, 31, 0 },{ 0, 0, 31, 31 },{ 31, 31, 0, 0 },{ 31, 0, 0, 31 },{ 0, 31, 31, 0 },{ 31, 31, 0, 0 },{ 0, 0, 31, 31 },{ 0, 31, 31, 0 },
	{ 31, 0, 0, 31 },{ 31, 31, 0, 0 },{ 0, 0, 31, 31 },{ 0, 31, 31, 0 },{ 31, 0, 0, 31 },{ 31, 31, 0, 0 },{ 0, 0, 31, 31 },{ 0, 31, 31, 0 },{ 31, 0, 0, 31 },{ 31, 31, 0, 0 },{ 0, 0, 31, 31 },{ 0, 31, 31, 0 },
	{ 31, 0, 0, 31 },{ 0, 0, 31, 31 },{ 31, 31, 0, 0 },{ 31, 0, 0, 31 },{ 0, 31, 31, 0 },{ 0, 0, 31, 31 },{ 31, 31, 0, 0 },{ 31, 0, 0, 31 },{ 0, 31, 31, 0 },{ 0, 31, 31, 0 },{ 31, 0, 0, 31 },{ 31, 31, 0, 0 },
	{ 0, 0, 31, 31 },{ 0, 31, 31, 0 },{ 31, 0, 0, 31 },{ 31, 31, 0, 0 },{ 0, 0, 31, 31 },{ 31, 0, 0, 31 },{ 0, 31, 31, 0 },{ 0, 0, 31, 31 },{ 31, 31, 0, 0 },{ 31, 0, 0, 31 },{ 0, 31, 31, 0 },{ 0, 0, 31, 31 },
	{ 31, 31, 0, 0 },{ 31, 0, 0, 31 },{ 0, 31, 31, 0 },{ 0, 0, 31, 31 },{ 31, 31, 0, 0 },{ 31, 0, 0, 31 },{ 0, 31, 31, 0 },{ 0, 0, 31, 31 },{ 31, 31, 0, 0 },{ 0, 31, 31, 0 },{ 31, 0, 0, 31 },{ 31, 31, 0, 0 },
	{ 0, 0, 31, 31 },{ 0, 31, 31, 0 },{ 31, 0, 0, 31 },{ 31, 31, 0, 0 },{ 0, 0, 31, 31 } };
#else
	char outputs[64][4] = {{ 0, 0, 7, 7 },{ 7, 7, 0, 0 },{ 7, 0, 0, 7 },{ 0, 7, 7, 0 },{ 0, 0, 7, 7 },{ 7, 7, 0, 0 },{ 7, 0, 0, 7 },{ 0, 7, 7, 0 },{ 7, 7, 0, 0 },{ 0, 0, 7, 7 },{ 0, 7, 7, 0 },{ 7, 0, 0, 7 },{ 7, 7, 0, 0 },{ 0, 0, 7, 7 },
	{ 0, 7, 7, 0 },{ 7, 0, 0, 7 },{ 7, 7, 0, 0 },{ 0, 0, 7, 7 },{ 0, 7, 7, 0 },{ 7, 0, 0, 7 },{ 7, 7, 0, 0 },{ 0, 0, 7, 7 },{ 0, 7, 7, 0 },{ 7, 0, 0, 7 },{ 0, 0, 7, 7 },{ 7, 7, 0, 0 },{ 7, 0, 0, 7 },{ 0, 7, 7, 0 },
	{ 0, 0, 7, 7 },{ 7, 7, 0, 0 },{ 7, 0, 0, 7 },{ 0, 7, 7, 0 },{ 0, 7, 7, 0 },{ 7, 0, 0, 7 },{ 7, 7, 0, 0 },{ 0, 0, 7, 7 },{ 0, 7, 7, 0 },{ 7, 0, 0, 7 },{ 7, 7, 0, 0 },{ 0, 0, 7, 7 },{ 7, 0, 0, 7 },{ 0, 7, 7, 0 },
	{ 0, 0, 7, 7 },{ 7, 7, 0, 0 },{ 7, 0, 0, 7 },{ 0, 7, 7, 0 },{ 0, 0, 7, 7 },{ 7, 7, 0, 0 },{ 7, 0, 0, 7 },{ 0, 7, 7, 0 },{ 0, 0, 7, 7 },{ 7, 7, 0, 0 },{ 7, 0, 0, 7 },{ 0, 7, 7, 0 },{ 0, 0, 7, 7 },{ 7, 7, 0, 0 },
	{ 0, 7, 7, 0 },{ 7, 0, 0, 7 },{ 7, 7, 0, 0 },{ 0, 0, 7, 7 },{ 0, 7, 7, 0 },{ 7, 0, 0, 7 },{ 7, 7, 0, 0 },{ 0, 0, 7, 7 } };
#endif	

#if defined(DYNAMIC_ALLOC)
	/*
	* dynamic allocation to memory        
	*
	* It have to modify STM32L151VET_RX_FLASH.ld 
	*     _Min_Heap_Size = 0xC400;      
	*	_Min_Stack_Size = 0x400;     
	*/
	
	unsigned short **trellis_struct;
	int i;

	trellis_struct = (unsigned short **)malloc(sizeof(unsigned short*) * States * interv);

	for(i = 0 ; i < States * interv; i++)
	{
		trellis_struct[i] = (unsigned short *)malloc(sizeof(unsigned short) * (tblen + (2*num) + 1) );
		memset(trellis_struct[i], 0, sizeof(unsigned short) * (tblen + (2*num) + 1) );
	}

#else
	/*
	* static allocation to memory
	*
	* It have to modify STM32L151VET_RX_FLASH.ld 
	*     _Min_Heap_Size = 0x200;      
	*	_Min_Stack_Size = 0x10000;     
	*/
	unsigned short trellis_struct[64 * 4][tblen + (2*num)+1];
	
	for (l = 0; l < States * interv; l++) 
	{
		memset(trellis_struct[l], 0, sizeof(unsigned short) * (tblen + (2*num) + 1) );
	}
#endif

	trellis_struct[flag][0] = 1;

	for (n=0; n<(2*SIZE)+(4*num); n++)
	{
		if (n<2*num)		data_tail[n] = data[2*SIZE-(2*num-n)];
		else if	(n>=(2*num)+(2*SIZE))	data_tail[n] = data[n-((2*num)+(2*SIZE))];
		else	data_tail[n] = data[n-2*num];
	}
	

	for (block=0; block<ceil((float)SIZE/(float)tblen); block++)
	{
		if ( (SIZE%tblen)!=0 && block == ceil((float)SIZE/(float)tblen)-1 )	
		{
			loop = (2*SIZE - 2*tblen*floor((float)SIZE/(float)tblen))/2 + 2*num;
		}

		for (n=0; n<loop; n++)
		{
			two_bit[0] = data_tail[(short)block*2*tblen + 2*n];
			two_bit[1] = data_tail[(short)block*2*tblen + (2*n)+1];

			l=0;

			for (m=0; m<States*interv; m+=interv)
			{
				if ( trellis_struct[m + flag][n] != 0 ) 
				{
					for (t=0; t<2; t++)
					{

						next_state = nextState[l][t];
						branch_metric = abs ( two_bit[0] - outputs[l][2*t] ) + abs(two_bit[1] - outputs[l][2*t+1]);
						total_path = branch_metric + trellis_struct[m + path_metric][n];

						if (trellis_struct[next_state * interv + path_metric][n + 1] == 0 && trellis_struct[next_state * interv + flag][n + 1] == 0) 
						{
							trellis_struct[next_state * interv + path_metric][n + 1] = total_path;		// 해당 path_metric 자리가 비어있을 경우 채워 넣음
							trellis_struct[next_state * interv + before_state][n + 1] = m;
							trellis_struct[next_state * interv + flag][n + 1] = 1;

							if ( t==1 )
								trellis_struct[(next_state)*interv+input_bit][n+1] = 1;
						}

						else if ( total_path < trellis_struct[next_state * interv + path_metric][n+1] ) 
						{
							trellis_struct[next_state * interv + path_metric][n+1] = total_path;
							trellis_struct[next_state * interv + before_state][n+1] = m;
							trellis_struct[next_state * interv + flag][n+1] = 1;


							if (t==1)
								trellis_struct[next_state * interv + input_bit][n+1] = 1;

						}
					}
				}
				l += 1;

			}
		}

		/*for (i=0; i<256; i++){
			printf("%03d : ", i);
			for (j=60; j<71; j++){
				printf("%d   ", trellis_struct[i][j]);
			}
			printf("\n");
		}*/

		max_metric = 0;
		
		for(nn=0; nn<States-1; nn++)
		{
			if(trellis_struct[max_metric*interv+path_metric][loop] <= trellis_struct[nn*interv+path_metric][loop])
			{
				max_metric = nn;
			}
		}
		
		max_metric = max_metric*interv;

		for (nn=loop-1; nn>=0; nn--)
		{
			if(nn>=num && nn<loop-num)
			{
				y_out[(short)block*tblen+nn-num] = trellis_struct[max_metric+input_bit][nn+1];
			}

			max_metric = trellis_struct[max_metric+before_state][nn+1];
		}

		for (nn = 0; nn < States * interv; nn++) 
		{
			memset(trellis_struct[nn], 0, sizeof(unsigned short) * (tblen + (2*num) + 1));
		}

		trellis_struct[flag][0] = 1;
		
	}


#if defined(DYNAMIC_ALLOC)
	/* dynamic allocation */
	for(i = 0 ; i < States * interv; i++)
	{
		free(trellis_struct[i]);
	}

	free(trellis_struct); 
#endif	

}
#else
#define tblen 24

void Viterbi_decoder(char *data, char *y_out) {
	char t, next_state, States = 64;
	char input_bit = 0, before_state = 1, path_metric = 2, flag = 3, interv = 4;
	char two_bit[2];
	char branch_metric;
	int total_path, max_metric;

	int l, m, n;
	int L = (SIZE + 2 * num) * 2;
	//char tblen = 80;
	int tblen_len = tblen + 4 * num;
	char block;
	int k;

	//char *data_ = (char*)malloc(sizeof(char)*(2*(SIZE+num*2)));
	char data_[2 * (SIZE + num * 2)];
	//char *data_temp = (char*)malloc(sizeof(char)*(4*num+tblen));
	char data_temp[4 * num + tblen];
	//char *y = (char*)malloc(sizeof(char)*(tblen_len/2));
	char y[(tblen + 4 * num) / 2];

	char nextState[64][2] = { { 0, 32 }, { 0, 32 }, { 1, 33 }, { 1, 33 }, { 2,
			34 }, { 2, 34 }, { 3, 35 }, { 3, 35 }, { 4, 36 }, { 4, 36 },
			{ 5, 37 }, { 5, 37 }, { 6, 38 }, { 6, 38 }, { 7, 39 }, { 7, 39 }, {
					8, 40 }, { 8, 40 }, { 9, 41 }, { 9, 41 }, { 10, 42 }, { 10,
					42 }, { 11, 43 }, { 11, 43 }, { 12, 44 }, { 12, 44 }, { 13,
					45 }, { 13, 45 }, { 14, 46 }, { 14, 46 }, { 15, 47 }, { 15,
					47 }, { 16, 48 }, { 16, 48 }, { 17, 49 }, { 17, 49 }, { 18,
					50 }, { 18, 50 }, { 19, 51 }, { 19, 51 }, { 20, 52 }, { 20,
					52 }, { 21, 53 }, { 21, 53 }, { 22, 54 }, { 22, 54 }, { 23,
					55 }, { 23, 55 }, { 24, 56 }, { 24, 56 }, { 25, 57 }, { 25,
					57 }, { 26, 58 }, { 26, 58 }, { 27, 59 }, { 27, 59 }, { 28,
					60 }, { 28, 60 }, { 29, 61 }, { 29, 61 }, { 30, 62 }, { 30,
					62 }, { 31, 63 }, { 31, 63 } };

	char outputs[64][4] = { { 0, 0, 7, 7 }, { 7, 7, 0, 0 }, { 7, 0, 0, 7 }, { 0,
			7, 7, 0 }, { 0, 0, 7, 7 }, { 7, 7, 0, 0 }, { 7, 0, 0, 7 }, { 0, 7,
			7, 0 }, { 7, 7, 0, 0 }, { 0, 0, 7, 7 }, { 0, 7, 7, 0 },
			{ 7, 0, 0, 7 }, { 7, 7, 0, 0 }, { 0, 0, 7, 7 }, { 0, 7, 7, 0 }, { 7,
					0, 0, 7 }, { 7, 7, 0, 0 }, { 0, 0, 7, 7 }, { 0, 7, 7, 0 }, {
					7, 0, 0, 7 }, { 7, 7, 0, 0 }, { 0, 0, 7, 7 },
			{ 0, 7, 7, 0 }, { 7, 0, 0, 7 }, { 0, 0, 7, 7 }, { 7, 7, 0, 0 }, { 7,
					0, 0, 7 }, { 0, 7, 7, 0 }, { 0, 0, 7, 7 }, { 7, 7, 0, 0 }, {
					7, 0, 0, 7 }, { 0, 7, 7, 0 }, { 0, 7, 7, 0 },
			{ 7, 0, 0, 7 }, { 7, 7, 0, 0 }, { 0, 0, 7, 7 }, { 0, 7, 7, 0 }, { 7,
					0, 0, 7 }, { 7, 7, 0, 0 }, { 0, 0, 7, 7 }, { 7, 0, 0, 7 }, {
					0, 7, 7, 0 }, { 0, 0, 7, 7 }, { 7, 7, 0, 0 },
			{ 7, 0, 0, 7 }, { 0, 7, 7, 0 }, { 0, 0, 7, 7 }, { 7, 7, 0, 0 }, { 7,
					0, 0, 7 }, { 0, 7, 7, 0 }, { 0, 0, 7, 7 }, { 7, 7, 0, 0 }, {
					7, 0, 0, 7 }, { 0, 7, 7, 0 }, { 0, 0, 7, 7 },
			{ 7, 7, 0, 0 }, { 0, 7, 7, 0 }, { 7, 0, 0, 7 }, { 7, 7, 0, 0 }, { 0,
					0, 7, 7 }, { 0, 7, 7, 0 }, { 7, 0, 0, 7 }, { 7, 7, 0, 0 }, {
					0, 0, 7, 7 } };

	unsigned char trellis_struct[States * interv][(tblen_len / 2) + 1];
	for (k = 0; k < States * interv; k++) {
		memset(trellis_struct[k], 0,
				sizeof(unsigned char) * (tblen_len / 2 + 1));
	}
	/*for (k = 0; k < States * interv; k++) {
	 for (l = 0; l < tblen_len / 2 + 1; l++) {

	 trellis_struct[k][l] = 0;
	 }
	 }*/
	trellis_struct[flag][0] = 1;



	for (n = 0; n < 2 * num; n++) {
		data_[n] = data[2 * SIZE - (2 * num - n)];
		data_[2 * (num + SIZE) + n] = data[n];
	}

	for (n = 2 * num; n < 2 * (SIZE + num); n++) {
		data_[n] = data[n - 2 * num];
	}

	for (block = 0; block < ceil((float) 2 * SIZE / (float) tblen); block++) {

		for (k = 0; k < tblen_len; k++) {
			if (2 * SIZE % tblen != 0
					&& block == ceil((float) 2 * SIZE / (float) tblen) - 1) { //(2*SIZE%tblen!=0 && block==ceil((float)(2*SIZE/tblen))-1){ // (float)??
				if (k >= 2 * SIZE - block * tblen + 4 * num) {
					break;
				} else
					data_temp[k] = data_[block * tblen + k];
			} else {
				data_temp[k] = data_[block * tblen + k];

			}
		}

		for (n = 0; n < tblen_len / 2; n++) {      //for (n=0; n<SIZE; n++)

			two_bit[0] = data_temp[n * 2];
			two_bit[1] = data_temp[n * 2 + 1];

			l = 0;

			for (m = 0; m < States * interv - 1; m += interv) { // for(m=0; m<States*interv -1; m += interv)

				for (t = 0; t < 2; t++) {

					if (trellis_struct[m + flag][n] != 0) {

						next_state = nextState[l][t];

						branch_metric = abs(two_bit[0] - outputs[l][2 * t])
								+ abs(two_bit[1] - outputs[l][2 * t + 1]);

						total_path = branch_metric
								+ trellis_struct[m + path_metric][n];

						if (trellis_struct[next_state * interv + path_metric][n
								+ 1] == 0
								&& trellis_struct[next_state * interv + flag][n
										+ 1] == 0) {
							trellis_struct[next_state * interv + path_metric][n
									+ 1] = total_path;
							trellis_struct[next_state * interv + before_state][n
									+ 1] = m;
							trellis_struct[next_state * interv + flag][n + 1] =
									1;

							if (t == 1)
								trellis_struct[(next_state) * interv + input_bit][n
										+ 1] = 1;
						}

						else if (total_path
								< trellis_struct[next_state * interv
										+ path_metric][n + 1]) {
							trellis_struct[next_state * interv + path_metric][n
									+ 1] = total_path;
							trellis_struct[next_state * interv + before_state][n
									+ 1] = m;
							trellis_struct[next_state * interv + flag][n + 1] =
									1;

							if (t == 1)
								trellis_struct[next_state * interv + input_bit][n
										+ 1] = 1;

						}
					}
				}
				l += 1;
			}

			if (2 * SIZE % tblen != 0
					&& block == ceil((float) 2 * SIZE / (float) tblen) - 1
					&& n >= (4 * num + (2 * SIZE - block * tblen)) / 2)
				break;

		}

		max_metric = 0;

		for (l = 0; l < States - 1; l++) {
			if (2 * SIZE % tblen != 0
					&& block == ceil((float) 2 * SIZE / (float) tblen) - 1) {
				if (trellis_struct[max_metric * interv + path_metric][(4 * num
						+ (2 * SIZE - block * tblen)) / 2]
						<= trellis_struct[l * interv + path_metric][(4 * num
								+ (2 * SIZE - block * tblen)) / 2]) {
					max_metric = l;

				} else
					continue;
			} else {
				if (trellis_struct[max_metric * interv + path_metric][tblen_len
						/ 2]
						<= trellis_struct[l * interv + path_metric][tblen_len
								/ 2]) {
					max_metric = l;

				} else
					continue;
			}
		}

		max_metric = max_metric * interv;

		if (2 * SIZE % tblen != 0
				&& block == ceil((float) 2 * SIZE / (float) tblen) - 1) {
			k = (4 * num + (2 * SIZE - block * tblen)) / 2 - 1;
		} else
			k = tblen_len / 2 - 1;
		for (k; k >= 0; k--) {
			y[k] = trellis_struct[max_metric + input_bit][k + 1];
			max_metric = trellis_struct[max_metric + before_state][k + 1];

		}

		for (k = 0; k < tblen / 2; k++) {

			if (2 * SIZE % tblen != 0
					&& block == ceil((float) 2 * SIZE / (float) tblen) - 1
					&& k >= (2 * SIZE - block * tblen) / 2)
				break;

			y_out[(char) block * (tblen / 2) + k] = y[num + k];
		}

		for (k = 0; k < States * interv; k++) {
			memset(trellis_struct[k], 0,
					sizeof(unsigned char) * (tblen_len / 2 + 1));
		}
		trellis_struct[flag][0] = 1;

	}



}


#endif


