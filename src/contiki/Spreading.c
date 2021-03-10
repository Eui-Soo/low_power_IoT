#include <stdlib.h>
#include "modem.h"

//extern char SG;
void Spreading(char *data, char *output){
	//char *output = (char*)malloc(sizeof(char)*(2 * SIZE*SG));
	int i;
	char j;

	for (i = 0; i<(2 * SIZE)+64; i++) {
		for (j = 0; j<SG; j++) {
			output[SG*i + j] = data[i];
		}
	}

	//return output;
}


