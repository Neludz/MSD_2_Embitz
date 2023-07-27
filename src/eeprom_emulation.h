#ifndef _EEPROM_EMULATION_H
#define _EEPROM_EMULATION_H

/*-----------------------------------------------------------------------*/
// includes
/*-----------------------------------------------------------------------*/
#include <stdint.h>

/*-----------------------------------------------------------------------*/
// define
/*-----------------------------------------------------------------------*/
/* Define the STM32F10Xxx Flash page size depending on the used STM32 device */
//#define PAGE_SIZE  (uint16_t)0x400  /* Page size = 1KByte */
#define PAGE_SIZE    data_eeprom_len
/* EEPROM start address in Flash */
//#define EEPROM_START_ADDRESS    (uint32_t)0x08010000
#define EEPROM_START_ADDRESS    data_eeprom_p


/* Results of Function*/
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_NOTFND,		/* 2: Not Found */
	RES_PAGEFULL,
} EEPRESULT;


/*-----------------------------------------------------------------------*/
// function prototype
/*-----------------------------------------------------------------------*/
EEPRESULT EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data);
EEPRESULT EE_WriteVariable(uint16_t VirtAddress, uint16_t Data);
EEPRESULT EE_UpdateVariable(uint16_t VirtAddress, uint16_t Data);
EEPRESULT GetNextVirtAddrData(uint16_t VirtAddressLast, uint16_t *VirtAddressNext, uint16_t *NextData);

#endif //_EEPROM_EMULATION_H
