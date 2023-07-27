#ifndef _IO_H
#define _IO_H

#include <stdint.h>
#include <stdbool.h>
#include <stm32f1xx.h>

#define IN	  	  (0x00)    //MODE_1
#define OUT_10MHz (0x01)    //MODE_1
#define OUT_2MHz  (0x02)    //MODE_1
#define OUT_50MHz (0x03)    //MODE_1


#define OUT_PP   (0x00) // MODE_2 - General purpose output push-pull
#define OUT_OD   (0x04) // MODE_2 - General purpose output Open-drain
#define OUT_APP  (0x08) // MODE_2 - Alternate function output Push-pull
#define OUT_AOD  (0x0C) // MODE_2 - Alternate function output Open-drain

#define IN_ADC   (0x00)     //MODE_2
#define IN_HIZ   (0x04)     //MODE_2
#define IN_PULL  (0x08)     //MODE_2

typedef struct
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    uint8_t MODE;
    uint8_t DefState;
    uint8_t ActiveState;
} tGPIO_Line;

/*
#define ADC_DMA_Temperature_ON() 		do {DMA1_Channel1->CCR |=  DMA_CCR1_EN;\
										ADC1->CR2 |= ADC_CR2_ADON;  \
                                		ADC1->CR2 |= ADC_CR2_SWSTART; } while (0)

#define ADC_DMA_Temperature_OFF()		do {ADC1->CR2 &= (~(ADC_CR2_ADON)); \
                                 		DMA1_Channel1->CCR &= (~(DMA_CCR1_EN)); } while (0)

#define DMA_Int_Temperature_ON() 		do {DMA1_Channel1->CCR |= DMA_CCR1_TCIE;} while (0)
#define DMA_Int_Temperature_OFF()		do {DMA1_Channel1->CCR &= (~(DMA_CCR1_TCIE));}	while (0)
#define ADC_Temperature_Start()			do {ADC1->CR2 |= ADC_CR2_SWSTART; } while (0)
*/
#define ADC_Current_Start()				do {ADC2->CR2  |= (ADC_CR2_JSWSTART);}	while (0)
#define ADC_Current_Off()				do {ADC2->CR2  &= (~(ADC_CR2_ADON));}	while (0)
#define ADC_Current_On()				do {ADC2->CR2  |= ADC_CR2_ADON;}	while (0)

#define ADC_CHANNEL_ALL					10
#define ADC_CHANNEL_T					9
#define ADC_N_CHANNEL_T_MCU				9

#define mV_ADC  				3300
#define ADC_COUNTS  			(1<<12)
#define OFFSET_I  				(ADC_COUNTS>>1)

//------========IO_Start_Table========------
//	 NAME  					GPIOx   GPIO_Pin    MODE_1 		MODE_2	 DefState  ActiveState
#define IO_TABLE\
	X_IO(io_LED,			GPIOA,	15,			OUT_2MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(io_RS485_Switch,	GPIOA,  8, 			OUT_2MHz,	OUT_PP,  	LOW,  	HIGH)	\
	X_IO(io_RX,				GPIOA,  10, 		IN,			IN_HIZ,  	HIGH,  	HIGH)	\
	X_IO(io_TX,				GPIOA,  9, 			OUT_50MHz,	OUT_APP, 	HIGH,  	HIGH)	\
	X_IO(io_DI_1,			GPIOB,  4, 			IN,			IN_PULL,	HIGH,  	LOW)	\
	X_IO(io_DI_2,			GPIOB,  5,  		IN,			IN_PULL,	HIGH,  	LOW)	\
	X_IO(io_DI_3,			GPIOB,  6,  		IN,			IN_PULL,   	HIGH,  	LOW)	\
	X_IO(io_DI_4,			GPIOB,  7,  		IN,			IN_PULL,    HIGH,  	LOW)	\
	X_IO(io_DOut_1,			GPIOB,  8,  		OUT_2MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(io_DOut_2,			GPIOB,  9,  		OUT_2MHz,	OUT_PP,		LOW,  	HIGH)	\
	X_IO(io_ADC_1,	 		GPIOB,  0, 			IN,			IN_ADC, 	LOW,  	HIGH)	\
	X_IO(io_ADC_2,	 		GPIOB,  1, 			IN,			IN_ADC, 	LOW,  	HIGH)	\
	X_IO(io_ADC_3,	 		GPIOA,  0, 			IN,			IN_ADC, 	LOW,  	HIGH)	\
	X_IO(io_ADC_4,	 		GPIOA,  1, 			IN,			IN_ADC, 	LOW,  	HIGH)	\
	X_IO(io_ADC_5,	 		GPIOA,  2,			IN,			IN_ADC, 	LOW,  	HIGH)	\
	X_IO(io_ADC_6,	  		GPIOA,  3, 			IN,			IN_ADC,		LOW,  	HIGH)	\
	X_IO(io_ADC_7,	 		GPIOA,  4, 			IN,			IN_ADC,		LOW,  	HIGH)	\
	X_IO(io_ADC_8,	 		GPIOA,  5, 			IN,			IN_ADC,		LOW,  	HIGH)	\
	X_IO(io_ADC_9,	 		GPIOA,  6, 			IN,			IN_ADC, 	LOW,  	HIGH)	\
	X_IO(io_ADC_10,			GPIOA,  7,			IN,			IN_ADC,		LOW,  	HIGH)	\

//  X_IO(io_Eeprom_CS,		GPIOB,  12, 		OUT_50MHz,	OUT_PP,		HIGH,  	HIGH)	\
//	X_IO(io_Eeprom_SCK,		GPIOB,  13, 		OUT_50MHz,	OUT_APP,	LOW,  	HIGH)	\
//	X_IO(io_Eeprom_MISO,	GPIOB,  14, 		IN,			IN_PULL, 	HIGH,  	HIGH)	\
//	X_IO(io_Eeprom_MOSI,	GPIOB,  15, 		OUT_50MHz,	OUT_APP,	HIGH,  	HIGH)	\

//USB pins init in default state
//------========IO_End_Table========------

typedef enum
{
#define X_IO(a,b,c,d,e,f,g)	a,
    IO_TABLE
#undef X_IO
    NUM_IO		//count
} tIOLine;

typedef enum
{
    OFF = 0,
    ON = 1,
    LOW = 0,
    HIGH =1,
} tIOState;

void IO_Init(void);
void IO_SetLine(tIOLine Line, bool State);
bool IO_GetLine(uint8_t Input);
void IO_SetLineActive(tIOLine Line, bool State);
bool IO_GetLineActive(uint8_t Input);
void IO_ConfigLine(tIOLine Line, uint8_t Mode, uint8_t State);
void IO_SPI_Init(void);
void IO_ADC_Init(void);
void IO_delay_ms(uint32_t ms);
uint16_t IO_getADCval(int nch);
int32_t IO_getMCUtemp(void);
//example
//     	i = IO_GetLine(io_LED3);
//		IO_ConfigLine(io_LED2, OUT_10MHz+OUT_PP, LOW);
//		IO_ConfigLine(io_ADC, IN+IN_ADC, LOW);


#endif /* _IO_H */

