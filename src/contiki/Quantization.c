#include<stdio.h>
#include<stdlib.h>
#include "modem.h"

void Quantization(char *data, char *quan){
	int i;

	for (i = 0; i<2 * SIZE; i++) {
		if (data[i] >= 0.9)
			quan[i] = 7;
		else if (data[i] >= 0.775)
			quan[i] = 6;
		else if (data[i] >= 0.65)
			quan[i] = 5;
		else if (data[i] >= 0.525)
			quan[i] = 4;
		else if (data[i] >= 0.4)
			quan[i] = 3;
		else if (data[i] >= 0.275)
			quan[i] = 2;
		else if (data[i] >= 0.15)
			quan[i] = 1;
		else
			quan[i] = 0;
	}

}

