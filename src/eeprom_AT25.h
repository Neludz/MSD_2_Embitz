#ifndef _EE_AT25_H
#define _EE_AT25_H

#include <stm32f1xx.h>
#include <stdint.h>
#include <stdbool.h>
#include <IO.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


#define  AT25_read_com              0x3
#define  AT25_write_com             0x2
#define  AT25_read_status_com       0x5
#define  AT25_set_write_com         0x6
#define  AT25_reset_write_com       0x4
#define  AT25_write_status_com   	0x1

#define  AT_25_RDY_bit              0x1
#define  AT_25_WEN_bit              0x2


void AT_25_Init (void);
void AT25_wait_ready_bit(void);
void AT25_write_byte(uint16_t adr, uint8_t *data_in, uint16_t len);
void AT25_read_byte(uint16_t adr, uint8_t *data_out, uint16_t len);
uint8_t AT25_get_status_bit(void);
uint16_t AT25_rxtx(uint16_t data);
void AT25_update_byte(uint16_t adr, uint8_t *data_in, uint16_t len);
void AT25_update_test(uint16_t adr, uint8_t *data_in, uint16_t len);
#endif
