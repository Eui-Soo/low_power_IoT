#ifndef _MODEM_H_
#define _MODEM_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define SIZE 192    // 288->144->280(MAC data 35byte), MAC_PAYLOAD * 8 
#define num 40		// 20
#define SG 1



void FEC_enc(char *data, char *output_tail);
void interleaver(char *data, char *output);
void Spreading(char *data, char *output);
void De_spreading(char *data, char *output);
void Quantization(char *data, char *quan);
void deinterleaver(char *data, char *output);
void Viterbi_decoder(char *data, char *y_out);
void Modem_EncPayload(uint8_t* payload, uint8_t payload_len, uint8_t* RF_data);


#endif
