#ifndef _EE_AT25_H
#define _EE_AT25_H

#include <stdint.h>

#define  AT25_READ_COM              0x3
#define  AT25_WRITE_COM             0x2
#define  AT25_READ_STATUS_COM       0x5
#define  AT25_SET_WRITE_COM         0x6
#define  AT25_RESET_WRITE_COM       0x4
#define  AT25_WRITE_STATUS_COM   	0x1

#define  AT_25_RDY_BIT              0x1
#define  AT_25_WEN_BIT              0x2


void AT25_Init (void);
void AT25_write_byte(uint16_t adr, uint8_t *data_in, uint16_t len);
void AT25_read_byte(uint16_t adr, uint8_t *data_out, uint16_t len);
void AT25_mutex_update_byte(uint16_t adr, uint8_t *data_out, uint16_t len);
void AT25_update_byte(uint16_t adr, uint8_t *data_in, uint16_t len);
#endif
