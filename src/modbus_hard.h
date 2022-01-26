#ifndef MODBUS_HARD_H_INCLUDED
#define MODBUS_HARD_H_INCLUDED

#include "modbus.h"
#include "modbus_reg.h"
#include "IO.h"
#include "main.h"
#include "system_stm32f1xx.h"
#include "stm32f1xx.h"

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

#define	 MIN_TIME_TO_START_TRANSMIT_MS    4


#define	BYTE_RECEIVED	USART1->DR
#define	BYTE_TO_SEND	USART1->DR


#define START_RECEIVE do{USART1->CR1 |= USART_CR1_RXNEIE;} while(0)

#define	TO_TRANSMT	do { IO_SetLine(io_RS485_Switch, ON); \
            USART1->CR1  |= (USART_CR1_TCIE|USART_CR1_TXEIE);   } 	while(0)
//USART1->CR1  &= ~(USART_CR1_RE|USART_CR1_RXNEIE)
//USART1->CR1  |= (USART_CR1_TCIE)

#define	TO_RECEIVE	do { IO_SetLine(io_RS485_Switch, OFF); \
            USART1->CR1  &= ~(USART_CR1_TCIE|USART_CR1_TXEIE);	} 	while(0)
//USART1->CR1  &= ~(USART_CR1_TCIE)
//USART1->CR1  |= (USART_CR1_RE|USART_CR1_RXNEIE)

#define	TXEIE_OFF	do { USART1->CR1  &= ~(USART_CR1_TXEIE); 	} 	while(0)

typedef enum {
    NO_PARITY_1_STOP	= 0x00,
    NO_PARITY_2_STOP	= 0x01,
    EVEN_PARITY_1_STOP	= 0x02,
    ODD_PARITY_1_STOP	= 0x03,

} Parity_Stop_Bits_t;

void mh_Write_Eeprom (void *mbb);
void mh_MB_Init(void);
void mh_USB_Init(void);
void mh_USB_Transmit_Start (void *mbb);
void mh_USB_Recieve(uint8_t *USB_buf, uint16_t len);
void mh_RS485_Init(void);
void mh_Rs485_Transmit_Start (void *mbb);
void rs485_timer_callback (xTimerHandle xTimer);
void IO_Uart1_Init(void);
void mh_task_Modbus (void *pvParameters);
void mh_Factory (void);
void mh_Buf_Init (void);

#endif /* MODBUS_HARD_H_INCLUDED */
