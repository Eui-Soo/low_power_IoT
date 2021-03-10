#include <stdlib.h>
#include "modem.h"



void De_spreading(char *data, char *output) {

	int i;

	for (i = 0; i<(2 * SIZE); i++) {
		for(int j=0; j<64;j++){
			//printf("%d",data[64*i+j]);
			output[i] += data[64*i+j];
		}

		//printf("[%d: %d] \n", i, output[i]);

		}
	//output[i]=output[i]/64;
}


