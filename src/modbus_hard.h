#ifndef MODBUS_HARD_H_INCLUDED
#define MODBUS_HARD_H_INCLUDED

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#define	UART_FRIQ	72000000LL

#define MODBUS_TASK_PRIORITY               M_MODBUS_TASK_PRIORITY
#define MODBUS_TASK_STACK_SIZE             M_MODBUS_TASK_STACK_SIZE

#define	 BAUD_9600		    0x1D4C
#define	 BAUD_19200		    0xEA6
#define	 BAUD_57600		    0x4E2
#define	 BAUD_115200	    0x271
#define  BAUD_NUMBER        4
#define	 RS_485_BAUD_LIST   {BAUD_9600, BAUD_19200, BAUD_57600, BAUD_115200}

typedef enum
{
    NO_PARITY_1_STOP	= 0x00,
    NO_PARITY_2_STOP	= 0x01,
    EVEN_PARITY_1_STOP	= 0x02,
    ODD_PARITY_1_STOP	= 0x03,

} Parity_Stop_Bits_t;

void mh_Write_Eeprom (void *mbb);
void mh_Modbus_Init(void);
void mh_USB_Init(void);
void mh_USB_Transmit_Start (void *mbb);
void mh_USB_Recieve(uint8_t *USB_buf, uint16_t len);
void mh_RS485_Init(void);
void mh_Rs485_Transmit_Start (void *mbb);
void mh_Rs485_Recieve_Start (void *mbb);
void rs485_timer_callback (xTimerHandle xTimer);
void IO_Uart1_Init(void);
void mh_task_Modbus (void *pvParameters);
void mh_Factory (void);
void mh_Buf_Init (void);

#endif /* MODBUS_HARD_H_INCLUDED */
