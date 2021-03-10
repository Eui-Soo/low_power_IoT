#include <stdlib.h>
#include <stdio.h>
#include "modem.h"

void interleaver(char *data, char *output)
{
   int col_len = 24; // 16(mac 18byte 기준 )
   int raw_len = 16; // 18(mac 18byte 기준 )
   for (int col = 0; col < col_len; col++)
   {
      for (int raw = 0; raw < raw_len; raw++)
      {
         output[raw + raw_len*col] = data[col_len*raw + col];
      }
   }
}
