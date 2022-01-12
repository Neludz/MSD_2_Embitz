/*
 * main.h
 *
 *  Created on: 21 мая 2019 г.
 *
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "clock.h"

#include <eeprom_AT25.h>
#include <measure_NTC.h>
#include <modbus_hard.h>

#include <stdbool.h>
#include "inttypes.h"
#include "system_stm32f1xx.h"
#include "stm32f1xx.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "IO.h"
#include "dma_103.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_istr.h"

#include "math.h"
#include <string.h>

#include "emfat.h"
#include "mass_mal.h"


#define Version_MSD_2			214


#define WATCH_DOG_TIME_MS		500
#define MAX_DI  				4
#define MAX_DI_TRIP_COUNTER 	2	//количество DI со счетчиком включения, max 32 (notification use number bit, see eeprom write task)
#define COUNTER_EE_UPD_MS		600000
#define MAX_DO  				2
#define	TIMER_ID_DO_1	        111
#define	TIMER_ID_DO_2        	222
#define	TIMER_ID_DO_LIST   		{TIMER_ID_DO_1, TIMER_ID_DO_2}

#define OFFSET_I_Plus  			(+15)
#define OFFSET_I_Minus  		(-15)

#define ADC_CURRENT_SAMPLE		0x01	//mask_for_notify
#define ADC_CURRENT_FIN			0x02	//mask_for_notify

#define SIZE_NTC_TABLE              166

#define RESET_VALUE					0xA01	//2561
#define FACTORY_SET_VALUE 			0xB01	//2817

// blink time
#define START_BLINK_TIME_MS			3000
#define START_BLINK_PERIOD_MS		150
#define TIME_ON_USB					950
#define TIME_OFF_USB 				50
#define TIME_ON_NO_USB				500
#define TIME_OFF_NO_USB 			500

#define M_MODBUS_TASK_PRIORITY         (tskIDLE_PRIORITY + 3)
#define M_MODBUS_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)

void vMeasure_Temperature (void *pvParameters);
void vMeasure_Current (void *pvParameters);
void vRead_DI (void *pvParameters);
void vBlinker (void *pvParameters);
void vUpdate_DO (void *pvParameters);
void vCurrent_Timers_Function (xTimerHandle xTimer);
void vOne_Shot_Timers_Function (xTimerHandle xTimer);
void Init_IWDG(uint16_t tw);
void IWDG_res(void);
void Set_Time_For_Blink (uint32_t On_Timer, uint32_t Off_Timer);
void sensor_param_init(void);
int16_t max_temperature(void);
//uint32_t Main_Timer_Set(const uint32_t AddTimeMs);
//bool Timer_Is_Expired (const uint32_t Timer);

//=========================================================================

// Отдадочная затычка. Сюда можно вписать код обработки ошибок.
#define	ERROR_ACTION(CODE,POS)		do{}while(1)

#endif /* MAIN_H_ */
