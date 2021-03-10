
#include <stdlib.h>
#include <stdio.h>
#include "modem.h"

void deinterleaver(char *data, char *output)
{
   int col_len = 16; //18(mac 18byte ���� )
   int raw_len = 24; //16(mac 18byte ���� )
   for (int col = 0; col < col_len; col++)
   {
      for (int raw = 0; raw < raw_len; raw++)
      {
         output[raw_len*col + raw] = data[col_len*raw + col];
      }
   }
}
