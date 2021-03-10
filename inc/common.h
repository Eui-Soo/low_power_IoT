
#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {false = 0, true} bool;
typedef enum {FALSE = 0, TRUE} BOOL;

#define BOOL bool


/*******************************************************************************
//  				Memory Structure								  
//
//		DTCMRAM (xrw)		: ORIGIN = 0x20000000, LENGTH = 128K
//		RAM_D1 (xrw)			: ORIGIN = 0x24000000, LENGTH = 512K
//		RAM_D2 (xrw)			: ORIGIN = 0x30000000, LENGTH = 288K
//		RAM_D3 (xrw)      	: ORIGIN = 0x38000000, LENGTH = 64K
//
*******************************************************************************/
#define ram1 __attribute__ ((section (".sram_1")))
#define ram2 __attribute__ ((section (".sram_2")))
#define ram3 __attribute__ ((section (".sram_3")))

#define ramfunc __attribute__((section(".RamFunc")))



#endif

