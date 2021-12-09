#ifndef MODBUS_HARD_H_INCLUDED
#define MODBUS_HARD_H_INCLUDED

#include "modbus.h"
#include "modbus_reg.h"
#include "IO.h"
#include "main.h"
#include "system_stm32f1xx.h"
#include "stm32f1xx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#define	UART_FRIQ	72000000LL

#define	TIMER_FULL		200	//макс счет , смотреть чтобы хватило диапазона
#define TIMER_15_SYMBOL 20	//кол-во символов тишины для определения конца сообщения

#define	 BAUD_9600		0x1D4C
#define	 BAUD_19200		0xEA6
#define	 BAUD_57600		0x4E2
#define	 BAUD_115200	0x271


#define	 TIMER_9600		9600LL		// Bod value for timer divider calculation (Fcpu*simvol_count)/baudrate/count_timer
#define	 TIMER_19200	19200LL
#define	 TIMER_57600	57600LL
#define	 TIMER_115200	115200LL


#define	TIMER_COUNT		TIM2->CNT
#define	BYTE_RECEIVED	USART1->DR
#define	BYTE_TO_SEND	USART1->DR

#define	STOP_TIMER	do { TIM2->CR1  &=  ~(TIM_CR1_CEN); } 	while(0)
#define	RESET_TIMER	do { TIM2->EGR  |= TIM_EGR_UG; \
                         TIM2->CR1  |=  TIM_CR1_CEN;     }	while(0)

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

void Write_Eeprom (void *mbb);
void Start_Trans_RS485 (void *mbb);
void Start_Trans_USB (void *mbb);
void vRS485 (void *pvParameters);
void MB_RS_Init(void);
void vUSB_MB (void *pvParameters);
void MB_USB_Init(void);
void USB_Recieve(uint8_t *USB_buf, uint16_t len);
void IO_Timer2_Init(void);
void IO_Uart1_Init(void);

#endif /* MODBUS_HARD_H_INCLUDED */
