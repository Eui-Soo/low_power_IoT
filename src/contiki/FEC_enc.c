#include <stdlib.h>
#include <stdio.h>
#include "modem.h"


void FEC_enc(char *data, char *output_tail) {
	char reg[7] = { 0 };
	char output[2 * (SIZE+2*num)];
	char data_tail[SIZE+num*2];
	int i;
	char j;

	for (i=0; i<num; i++){
		data_tail[i] = data[(SIZE)-(num-i)]; 
		data_tail[num+SIZE+i] = data[i];
	}
	for (i=num; i<SIZE+num; i++){
		data_tail[i] = data[i-num];
	}

	for (i = 0; i<SIZE+num*2; i++) {

		for (j = 6; j>0; j--) {
			reg[j] = reg[j - 1];
		}
		reg[0] = data_tail[i];
		output[2 * i] = ((reg[0] + reg[2] + reg[3] + reg[5] + reg[6]) % 2);
		output[2 * i + 1] = ((reg[0] + reg[1] + reg[2] + reg[3] + reg[6]) % 2);
	}

	for (i=0; i<2*SIZE; i++){
		output_tail[i] = output[2*num+i];
	}

}
