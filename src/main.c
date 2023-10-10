/*
******************************************************************************
File:
Info:

******************************************************************************
*/

//======================Includes========================================
#include "main.h"
#include <modbus_reg.h>
#include <modbus_hard.h>

#include "clock.h"

#include <eeprom_AT25.h>
#include <measure_NTC.h>

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
#include "math.h"
#include <string.h>
#include "stm32f1xx_hal.h"

#include "emfat.h"

#include "usb_device.h"

//======================Variables========================================
//#define DEBU_USER

extern uint16_t MBbuf_main[];

extern const RegParameters_t MBRegParam[MB_NUM_BUF] ;

const uint32_t Dot_count [4]= {1,10,100,1000};

volatile uint32_t Error=0;
volatile uint32_t Number_Of_Samples=0;
volatile int32_t Inow=0;
uint32_t Itotal = 0;
bool X_DI[MAX_DI];
int32_t Time_Blink_On = TIME_ON_NO_USB;
int32_t Time_Blink_Off = TIME_OFF_NO_USB;

xTimerHandle xCurrent_Timer;
xTimerHandle xOne_Shot_Timers [MAX_DO];
TaskHandle_t Current_Task;
TaskHandle_t Temperature_Task;

extern emfat_t emfat;
extern  emfat_entry_t entries[];
//======================Interrupt========================================

void ADC1_2_IRQHandler (void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if (ADC2->SR & ADC_SR_JEOC)
    {
        ADC2->SR = ~(ADC_SR_JEOC);
        Number_Of_Samples++;
        Inow = (ADC2->JDR1);
        xTaskNotifyFromISR(Current_Task, ADC_CURRENT_SAMPLE, eSetBits, &xHigherPriorityTaskWoken);
        return;
    }
    Error|=0x1;
}

//-------------------------------------------------------------------------
//======================Task========================================

void vMeasure_Temperature (void *pvParameters)
{
    int32_t i=0;
    uint32_t Adc_Filter_Value[ADC_CHANNEL_T];
    uint32_t Adc_Filter_T_MCU=0;
    uint32_t ADC_Val;
    for (i =0; i<ADC_CHANNEL_T; i++)
    {
        Adc_Filter_Value[i]=0;
        MBbuf_main[(i+Reg_T_0_Channel)] = TEMPERATURE_NO_MEASURE;
    }
    MBbuf_main[(Reg_T_Alarm_bit)]=0;
    MBbuf_main[(Reg_T_Warning_bit)]=0;
    sensor_param_init();
    while(1)
    {
        for(i =0; i<ADC_CHANNEL_T; i++)
        {
            ADC_Val=(uint16_t)IO_getADCval(i);
            Adc_Filter_Value[i]=(Adc_Filter_Value[i]*7+ADC_Val)>>3;
            MBbuf_main[(i+Reg_T_0_Channel)] = calc_temperature(Adc_Filter_Value[i]);
            if ((int16_t)MBbuf_main[(i+Reg_T_0_Channel)]>=(int16_t)MBbuf_main[(Reg_T_level_Warning)]) MBbuf_main[Reg_T_Warning_bit]|=1<<i;
            else MBbuf_main[Reg_T_Warning_bit] &=~(1<<i);

            if ((int16_t)MBbuf_main[(i+Reg_T_0_Channel)]>=(int16_t)MBbuf_main[(Reg_T_level_Alarm)]) MBbuf_main[Reg_T_Alarm_bit]|=1<<i;
            else MBbuf_main[Reg_T_Alarm_bit] &=~(1<<i);
        }
        Adc_Filter_T_MCU=((Adc_Filter_T_MCU*3 + IO_getMCUtemp())>>2);
        MBbuf_main[Reg_T_MSD]=Adc_Filter_T_MCU;
        vTaskDelay(29/portTICK_RATE_MS);
    }
}
//-------------------------------------------------------------------------

void vMeasure_Current (void *pvParameters)
{
    int64_t Isum=0;
    uint32_t Filter_Ratio = 0;
    uint32_t Cross_Count=0;
    uint32_t ulNotifiedValue;
    bool lastVCross=0;
    bool checkVCross=0;
    int32_t Intermediate_I=0;
    int32_t Filtered_I=0;
    uint32_t Irms=0;

    MBbuf_main[Reg_Cur_RMS_W1] = 0;
    MBbuf_main[Reg_Cur_RMS_W2] = 0;

    while(1)
    {
        if (!MBbuf_main[Reg_Mode_Cur])
        {
            MBbuf_main[Reg_Cur_RMS_W1] = 0;
            MBbuf_main[Reg_Cur_RMS_W2] = 0;
            ADC_Current_Off();
            vTaskSuspend(NULL);
            ADC_Current_On();
        }
        Isum=0;
        Number_Of_Samples=0;
        Cross_Count = 0;
        Inow=0;
        Intermediate_I=0;
        Filtered_I=0;
        //tick1 =  Main_Timer_Set (MBbuf_main[Reg_Cur_Time_Measure]);
        xTimerChangePeriod(xCurrent_Timer, (MBbuf_main[Reg_Cur_Time_Measure]/portTICK_RATE_MS), 0);
        xTimerStart(xCurrent_Timer, 0);
        ADC_Current_Start();

        do
        {
            xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
            if(ulNotifiedValue&ADC_CURRENT_SAMPLE)
            {
                Intermediate_I += ((Inow-OFFSET_I) - Filtered_I);	//фильтрация и смещение дискрет с АЦП (с датчика половина опорного->ноль)
                Filtered_I = (( Intermediate_I * MBbuf_main[Reg_Cur_Filter_Ratio]) >> 8);
                Isum+= (Filtered_I)*(Filtered_I);
                lastVCross = checkVCross;
                if (Filtered_I > OFFSET_I_Plus) checkVCross = true;
                if (Filtered_I < OFFSET_I_Minus) checkVCross = false;
                if (Number_Of_Samples==1) lastVCross = checkVCross;
                if (lastVCross != checkVCross) Cross_Count++;
            }
            if(!(ulNotifiedValue&ADC_CURRENT_FIN))
            {
                ADC_Current_Start();
            }
        }
        while(!(ulNotifiedValue&ADC_CURRENT_FIN));

        Filter_Ratio = ADC_COUNTS * MBbuf_main[Reg_Cur_Sens_Hall_Ratio]; //divider
        Irms = (((sqrt(Isum / Number_Of_Samples)) *MBbuf_main[Reg_Cur_Scale] * Dot_count[MBbuf_main[Reg_Cur_Dot]&0x03]*mV_ADC)+(Filter_Ratio>>1))/Filter_Ratio;

        Filter_Ratio = ((MBbuf_main[Reg_Cur_Zero_Level]*Dot_count[MBbuf_main[Reg_Cur_Dot]&0x03]*MBbuf_main[Reg_Cur_Scale])/1000);
        if (Irms < Filter_Ratio) Irms=0;                                    //zero level

        MBbuf_main[Reg_Cur_Cross_Count] = (uint16_t)Cross_Count;
        MBbuf_main[Reg_Cur_RMS_W1] = (uint16_t)(Irms & 0xFFFF);
        MBbuf_main[Reg_Cur_RMS_W2] = (uint16_t)((Irms >>16) & 0xFFFF);
        MBbuf_main[Reg_Cur_N_Measure_W1] =	(uint16_t) Number_Of_Samples & 0xFFFF;
        MBbuf_main[Reg_Cur_N_Measure_W2] = (uint16_t)((Number_Of_Samples >>16) & 0xFFFF);
        Itotal=Irms;
        vTaskDelay((199/portTICK_RATE_MS));
    }
}
//-------------------------------------------------------------------------

void vRead_DI (void *pvParameters)
{
    uint32_t i = 0;
    bool DI_Check[MAX_DI];
    bool Previous_State[MAX_DI];
    bool Last_State[MAX_DI];

    MBbuf_main[(Reg_Status_DI_Bit)]=0;
    for(i = 0; i<MAX_DI; i++)
    {
        X_DI[i] = IO_GetLineActive(io_DI_1+i);		//если вход включен при старте, то счетчик не работает
        Previous_State[i] = X_DI[i];
        Last_State[i] = X_DI[i];
        DI_Check[i] = X_DI[i];
        if (!X_DI[i])
        {
            MBbuf_main[Reg_Status_DI_Bit] &= ~(1<<i);
        }
        else
        {
            MBbuf_main[Reg_Status_DI_Bit] |=  X_DI[i]<<i;
        }
    }
    vTaskDelay(20/portTICK_RATE_MS);
    while(1)
    {
        IWDG_res();				//reset watch dog timer
        for(i = 0; i<MAX_DI; i++)
        {
            Previous_State[i] = Last_State[i];
            Last_State[i] = IO_GetLineActive(io_DI_1+i);
            if (Last_State[i] == Previous_State[i])
            {
                if(DI_Check[i] != Last_State[i])
                {
                    X_DI[i] = Last_State[i];
                    if (!X_DI[i])
                    {
                        MBbuf_main[Reg_Status_DI_Bit] &= ~(1<<i);
                    }
                    else
                    {
                        MBbuf_main[Reg_Status_DI_Bit] |=  X_DI[i]<<i;
                    }
                    DI_Check[i] = X_DI[i];
                }
            }
        }
        vTaskDelay(71/portTICK_RATE_MS);
    }
}
//-------------------------------------------------------------------------

void vBlinker (void *pvParameters)
{
    for (uint32_t i=0; i<(START_BLINK_TIME_MS/START_BLINK_PERIOD_MS); i++)
    {
        IO_SetLine(io_LED,ON);
        vTaskDelay((START_BLINK_PERIOD_MS>>1)/portTICK_RATE_MS);
        IO_SetLine(io_LED,OFF);
        vTaskDelay((START_BLINK_PERIOD_MS>>1)/portTICK_RATE_MS);
    }
    while(1)
    {
        IO_SetLine(io_LED,ON);
        vTaskDelay(Time_Blink_On/portTICK_RATE_MS);
        IO_SetLine(io_LED,OFF);
        vTaskDelay(Time_Blink_Off/portTICK_RATE_MS);

        if (eTaskGetState(Current_Task)==eSuspended)
        {
            if (MBbuf_main[Reg_Mode_Cur]) vTaskResume(Current_Task);
        }
        if (MBbuf_main[Reg_Set_Default_Reset])
        {
            if (MBbuf_main[Reg_Set_Default_Reset]==RESET_VALUE)
            {
                NVIC_SystemReset();
            }
            if (MBbuf_main[Reg_Set_Default_Reset]==FACTORY_SET_VALUE)
            {
                mh_Factory();
            }
            MBbuf_main[Reg_Set_Default_Reset]=0;
        }
#ifdef DEBU_USER
        printf ( "1 sec \n" );
#endif
        MBbuf_main[MB_ERRORor_Count] = Error;
        MBbuf_main[Reg_T_Max] = max_temperature();
    }
}
//-------------------------------------------------------------------------

void vUpdate_DO (void *pvParameters)
{
    uint32_t Mode_DO = 0;
    uint32_t i=0;
    uint16_t Temp_Mask=0;
    uint32_t x_d=0;
    uint32_t I_Comparison_Max = 0;
    uint32_t I_Comparison_Min = 0;

    MBbuf_main[(Reg_Status_DO_Bit)]=0;
    MBbuf_main[Reg_DO_On_Bit]=0;
    MBbuf_main[Reg_DO_Off_Bit]=0;
    vTaskDelay(3000/portTICK_RATE_MS);
    while(1)
    {
        if(Mode_DO!= MBbuf_main[Reg_Mode_DO])
        {
            Mode_DO = MBbuf_main[Reg_Mode_DO];
            for(i = 0; i<MAX_DO; i++)
            {
                IO_SetLine((io_DOut_1 + i), OFF);
                xTimerStop(xOne_Shot_Timers[i], 0);
            }
            MBbuf_main[Reg_DO_On_Bit]=0;
            MBbuf_main[Reg_DO_Off_Bit]=0;
        }
        switch (Mode_DO)
        {
        case 0:
        {
            break;
        }

        case 1:	//DO следит за температурой
        {
            for(i=0; i<MAX_DO; i++)
            {
                if (MBbuf_main[Reg_T_Number_Sensor_1+i] & (MBbuf_main[Reg_T_Alarm_bit] & MBbuf_main[Reg_T_Warning_bit]))
                {
                    //DO_Instruct = ON;
                    IO_SetLine((io_DOut_1 + i), ON);
                }
                else if ((!(MBbuf_main[Reg_T_Number_Sensor_1+i] & (MBbuf_main[Reg_T_Warning_bit] | MBbuf_main[Reg_T_Alarm_bit])))||((MBbuf_main[Reg_DO_Off_Bit]>>i)&1))
                {
                    //DO_Instruct = OFF;
                    IO_SetLine((io_DOut_1 + i), OFF);
                }
            }
            if (MBbuf_main[Reg_DO_Off_Bit])	MBbuf_main[Reg_DO_Off_Bit] = 0;
            break;
        }
        case 2:	//DO_1 следит за током
        {
            I_Comparison_Max = MBbuf_main[Reg_Cur_Level_Alarm_W1]|(MBbuf_main[Reg_Cur_Level_Alarm_W2]<<16);
            I_Comparison_Min = MBbuf_main[Reg_Cur_Lev_Warn_W1]|(MBbuf_main[Reg_Cur_Lev_Warn_W1]<<16);
            if ((Itotal >= I_Comparison_Min) && (Itotal >= I_Comparison_Max))
            {
                IO_SetLine((io_DOut_1 ), ON);
            }
            else if (((Itotal < I_Comparison_Min) && (Itotal < I_Comparison_Max))||(MBbuf_main[Reg_DO_Off_Bit]))
            {
                IO_SetLine((io_DOut_1 ), OFF);
            }
            if (MBbuf_main[Reg_DO_Off_Bit])	MBbuf_main[Reg_DO_Off_Bit]=0;
            break;
        }

        case 3:	//DO_1 следит за током, DO_2 следит за температурой
        {
            //current
            I_Comparison_Max = MBbuf_main[Reg_Cur_Level_Alarm_W1]|(MBbuf_main[Reg_Cur_Level_Alarm_W2]<<16);
            I_Comparison_Min = MBbuf_main[Reg_Cur_Lev_Warn_W1]|(MBbuf_main[Reg_Cur_Lev_Warn_W1]<<16);
            if ((Itotal >= I_Comparison_Min) && (Itotal >= I_Comparison_Max))
            {
                IO_SetLine((io_DOut_1 ), ON);
            }
            else if (((Itotal < I_Comparison_Min) && (Itotal < I_Comparison_Max))||(MBbuf_main[Reg_DO_Off_Bit] & 1))
            {
                IO_SetLine((io_DOut_1 ), OFF);
            }
            //temperature
            if (MBbuf_main[Reg_T_Number_Sensor_2] & (MBbuf_main[Reg_T_Alarm_bit] & MBbuf_main[Reg_T_Warning_bit]))
            {
                IO_SetLine((io_DOut_2), ON);
            }
            else if ((!(MBbuf_main[Reg_T_Number_Sensor_2] & (MBbuf_main[Reg_T_Warning_bit] | MBbuf_main[Reg_T_Alarm_bit])))||(MBbuf_main[Reg_DO_Off_Bit]&2))
            {
                IO_SetLine((io_DOut_2), OFF);
            }
            if (MBbuf_main[Reg_DO_Off_Bit])	MBbuf_main[Reg_DO_Off_Bit]=0;
            break;
        }

        case 4:		//modbus
        {
            if(MBbuf_main[Reg_DO_On_Bit]||MBbuf_main[Reg_DO_Off_Bit])
            {
                Temp_Mask = MBbuf_main[Reg_DO_On_Bit]&(~MBbuf_main[Reg_DO_Off_Bit]);
                for (i=0; i<MAX_DO; i++)
                {
                    if ((Temp_Mask>>i) & 1) IO_SetLine((io_DOut_1 + i), ON);
                    if ((MBbuf_main[Reg_DO_Off_Bit]>>i) & 1) IO_SetLine((io_DOut_1 + i), OFF);
                }
            }
            MBbuf_main[Reg_DO_Off_Bit]=0;
            MBbuf_main[Reg_DO_On_Bit]=0;
            break;
        }

        case 5:	//управление с задержкой без ОС
        {
            if(MBbuf_main[Reg_DO_On_Bit]||MBbuf_main[Reg_DO_Off_Bit])
            {
                Temp_Mask = MBbuf_main[Reg_DO_On_Bit]&(~MBbuf_main[Reg_DO_Off_Bit]);
                for (i=0; i<MAX_DO; i++)
                {
                    if ((Temp_Mask>>i) & 1)
                    {
                        xTimerChangePeriod(xOne_Shot_Timers[i],MBbuf_main[Reg_DO_1_Delay]*100/(portTICK_RATE_MS),0);
                        if (xTimerReset(xOne_Shot_Timers[i], 0)==pdTRUE)
                        {
                            IO_SetLine((io_DOut_1 + i), ON);
                        }
                    }
                    if ((MBbuf_main[Reg_DO_Off_Bit]>>i) & 1)
                    {
                        IO_SetLine((io_DOut_1 + i), OFF);
                        xTimerStop(xOne_Shot_Timers[i], 0);
                    }
                }
                MBbuf_main[Reg_DO_Off_Bit]=0;
                MBbuf_main[Reg_DO_On_Bit]=0;
            }
            break;
        }

        case 6:	//управление с задержкой c ОС
        {
            for (i=0; i<MAX_DO; i++)
            {
                x_d = i & 1;
                Temp_Mask = MBbuf_main[Reg_DO_On_Bit]&(~MBbuf_main[Reg_DO_Off_Bit]);
                if ((Temp_Mask>>i) & 1)
                {
                    if(x_d && X_DI[i>>1])
                    {
                        xTimerChangePeriod(xOne_Shot_Timers[i],MBbuf_main[Reg_DO_1_Delay]*100/(portTICK_RATE_MS),0);
                        if (xTimerReset(xOne_Shot_Timers[i], 0)==pdTRUE)
                        {
                            IO_SetLine((io_DOut_1 + i), ON);
                        }
                    }
                    if((!x_d) && (!X_DI[i>>1]))
                    {
                        xTimerChangePeriod(xOne_Shot_Timers[i],MBbuf_main[Reg_DO_1_Delay]*100/(portTICK_RATE_MS),0);
                        if (xTimerReset(xOne_Shot_Timers[i], 0)==pdTRUE)
                        {
                            IO_SetLine((io_DOut_1 + i), ON);
                        }
                    }
                }
                if(IO_GetLineActive(io_DOut_1 + i))
                {
                    if (((MBbuf_main[Reg_DO_Off_Bit]>>i) & 1)||(x_d && (!X_DI[i>>1]))||((!x_d) && (X_DI[i>>1])))
                    {
                        IO_SetLine((io_DOut_1 + i), OFF);
                        xTimerStop(xOne_Shot_Timers[i], 0);
                    }
                }
            }
            MBbuf_main[Reg_DO_Off_Bit]=0;
            MBbuf_main[Reg_DO_On_Bit]=0;
            break;
        }

        default:
        {
            break;
        }
        }
        for (i=0; i<MAX_DO; i++)
        {
            if(IO_GetLineActive(io_DOut_1 + i))	MBbuf_main[Reg_Status_DO_Bit]|=1<<i;
            else MBbuf_main[Reg_Status_DO_Bit]&= ~(1<<i);
        }
        vTaskDelay(311/portTICK_RATE_MS);
    }
}
//-------------------------------------------------------------------------

//======================Function========================================
/*
bool Timer_Is_Expired (const uint32_t Timer)
{
uint32_t TimeMs;
TimeMs = xTaskGetTickCount();
return ((TimeMs - Timer) < (1UL << 31));
}
//-------------------------------------------------------------------------
uint32_t Main_Timer_Set(const uint32_t AddTimeMs)
{
uint32_t TimeMs;
TimeMs = xTaskGetTickCount();
return TimeMs + AddTimeMs;
}*/
//-------------------------------------------------------------------------
int16_t max_temperature(void)
{
    int16_t temper_temp, temper_new;
    temper_temp = (-100);
    for (int32_t i=0; i<ADC_CHANNEL_T; i++)
    {
        temper_new = MBbuf_main[Reg_T_0_Channel+i];
        if ((temper_new>temper_temp)&&(MBbuf_main[Reg_T_Number_Sensor_1]&(1<<i)))
        {
            temper_temp = temper_new;
        }
    }
    if(temper_temp == (-100))
    {
        temper_temp = TEMPERATURE_NO_MEASURE;
    }
    return temper_temp;
}
//-------------------------------------------------------------------------
void sensor_param_init(void)
{
    NTC_Calculation_Data_t NTC;

    NTC.NTC_r2=(MBbuf_main[Reg_NTC_R2_Value_W1]&0xFFFF)|((MBbuf_main[Reg_NTC_R2_Value_W2]&0xFFFF)<<16);
    NTC.NTC_r_divider=(MBbuf_main[Reg_NTC_R_Divider_W1]&0xFFFF)|((MBbuf_main[Reg_NTC_R_Divider_W2]&0xFFFF)<<16);
    NTC.NTC_adc_multipler=1;
    NTC.NTC_adc_resolution=ADC_COUNTS;
    NTC.NTC_b=(int16_t)MBbuf_main[Reg_NTC_B_Value];
    NTC.NTC_t2=(int16_t)MBbuf_main[Reg_NTC_T2_Value];
    NTC.NTC_start_temperature=(int16_t)MBbuf_main[Reg_NTC_Start_Temper];
    NTC.NTC_step_temperature=(int16_t)MBbuf_main[Reg_NTC_Step_Temper];
    NTC.NTC_temper_number_step=MBbuf_main[Reg_NTC_Temper_N_Step];
    calculate_table_NTC(NTC);
}
//-------------------------------------------------------------------------

void Set_Time_For_Blink (uint32_t On_Timer, uint32_t Off_Timer)
{
    Time_Blink_On = On_Timer;
    Time_Blink_Off = Off_Timer;
}
//-------------------------------------------------------------------------

void vOne_Shot_Timers_Function (xTimerHandle xTimer)
{
    uint32_t pxTimerID;

// получаем идентификатор таймера
    pxTimerID = (uint32_t) pvTimerGetTimerID(xTimer);
    for (int32_t i=0; i<MAX_DO; i++)
    {
        if (pxTimerID == i)
        {
            IO_SetLine((io_DOut_1 + i), OFF);
            return;
        }
    }
}
//-------------------------------------------------------------------------

void vCurrent_Timers_Function (xTimerHandle xTimer)
{
    xTaskNotify(Current_Task, ADC_CURRENT_FIN, eSetBits);
    return;
}
//-------------------------------------------------------------------------

void flash_btock(void)
{
    if (!(FLASH->OBR & FLASH_OBR_RDPRT))
    {
        mh_Factory();	//when first start -> set MBbuf

        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;

        FLASH->OPTKEYR = FLASH_KEY1;
        FLASH->OPTKEYR = FLASH_KEY2;
        FLASH->CR |= FLASH_CR_OPTER;
        FLASH->CR|= FLASH_CR_STRT;
        while ((FLASH->SR & FLASH_SR_BSY) != 0 );

        FLASH->CR |= FLASH_CR_LOCK;
    }
    /*
    	if (FLASH_GetReadOutProtectionStatus() == RESET)
    	{
    	Factory();							//when first start -> set MBbuf

        FLASH_Unlock();                 	//
    	FLASH_ReadOutProtection(ENABLE); 	//
    	FLASH_Lock();                    	//
    	}
    	*/
}
//-------------------------------------------------------------------------

void Init_IWDG(uint16_t tw) // Параметр tw от 7мс до 26200мс
{
// Для IWDG_PR=7 Tmin=6,4мс RLR=Tмс*40/256
    IWDG->KR=0x5555; // Ключ для доступа к таймеру
    IWDG->PR=7; // Обновление IWDG_PR
    IWDG->RLR=tw*40/256; // Загрузить регистр перезагрузки
    IWDG->KR=0xAAAA; // Перезагрузка
    IWDG->KR=0xCCCC; // Пуск таймера
}
//-------------------------------------------------------------------------

// Функция перезагрузки сторожевого таймера IWDG

void IWDG_res(void)
{
    IWDG->KR=0xAAAA; // Перезагрузка
}
/*
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

int main(void)
{
    ClockInit();//SystemInit();  // Фукнция CMSIS которая установит тактовую частоту
    IO_Init();
#ifndef DEBU_USER
    flash_btock();
#endif
    emfat_init(&emfat, "MSD_2", entries);
    mh_Buf_Init();
    MX_USB_DEVICE_Init();
    Init_IWDG(WATCH_DOG_TIME_MS);

    /*	timers	*/
    for (uint32_t i = 0; i<MAX_DO; i++)
    {
        xOne_Shot_Timers[i]=xTimerCreate("One_Shot_Timer_n", 10000/portTICK_RATE_MS, pdFALSE, (void *)i, vOne_Shot_Timers_Function);
    }
    xCurrent_Timer=xTimerCreate("xCurrent_T", 1200/portTICK_RATE_MS, pdFALSE, NULL, vCurrent_Timers_Function);
    /*	Semaphore	*/

    /*	task	*/
    if(pdTRUE != xTaskCreate(vMeasure_Temperature,	"Temperature", 	configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY + 3, &Temperature_Task)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vMeasure_Current,		"Current", 		configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY + 2, &Current_Task)) ERROR_ACTION(TASK_NOT_CREATE,0);

    if(pdTRUE != xTaskCreate(vBlinker,	"Blinker", 	configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vRead_DI,	"DI", 		configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
    if(pdTRUE != xTaskCreate(vUpdate_DO,"DO", 		configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);

    mh_Modbus_Init();   //create task for modbus

    /*	start OS	*/
#ifdef DEBU_USER
    printf ( "[ INFO ] Program start now\n" );
#endif
    vTaskStartScheduler();
    return 0;
}
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

void vApplicationIdleHook( void )
{
}

void vApplicationMallocFailedHook( void )
{
    for( ;; );
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) xTask;
    for( ;; );
}

void vApplicationTickHook( void )
{
}

