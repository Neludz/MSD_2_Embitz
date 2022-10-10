
#include "modbus_hard.h"
#include "modbus_reg.h"
#include "modbus.h"
#include "modbus_x_macros.h"
#include <dma_103.h>
#include "main.h"
//-----------------------------------------------------------------------
// Variable
//-----------------------------------------------------------------------

uint16_t MBbuf_main[NUM_BUF]={Version_MSD_2};
extern uint32_t Error;

xQueueHandle xModbusQueue;
TimerHandle_t rs485_timer_handle;
TaskHandle_t m_modbus_task_handle;

volatile mb_struct MB_RS485;
volatile mb_struct MB_USB;

uint8_t RS485_MB_Buf[MB_FRAME_MAX];
uint8_t USB_MB_Buf[MB_FRAME_MAX];

const uint16_t Baud_rate[BAUD_NUMBER]={BAUD_9600, BAUD_19200, BAUD_57600, BAUD_115200};


extern const t_default_state default_state[];
//#define	limit_min(a)	default_state[a].Min_Level
//#define	limit_max(a)	default_state[a].Max_Level_Mask

//-----------------------------------------------------------------------
// Task function
//-----------------------------------------------------------------------

void mh_task_Modbus (void *pvParameters)
{
    mb_struct *st_mb;
    vTaskDelay(3000);

    //start recieve rs485 data
    DMA_Enable(DMA1_Channel5);

    while(1)
    {
        xQueueReceive(xModbusQueue,&st_mb,portMAX_DELAY);
        MBparsing((mb_struct*) st_mb);
    }
}

//-----------------------------------------------------------------------
// Function
//-----------------------------------------------------------------------

void USART1_IRQHandler (void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t cnt;
    (void)cnt;
    if (USART1->SR & USART_SR_IDLE)
	{
	    cnt = USART1->DR;
	    if (MB_RS485.mb_state==STATE_IDLE)
        {
          	MB_RS485.mb_index = MB_FRAME_MAX - DMA1_Channel5->CNDTR;
            DMA_Disable(DMA1_Channel5);
            DMA1_Channel5->CNDTR = MB_FRAME_MAX;
            MB_RS485.mb_state=STATE_RCVE;
            xTimerResetFromISR(rs485_timer_handle, &xHigherPriorityTaskWoken);
            DMA_Enable(DMA1_Channel5);
        }
        else
        {
            DMA_Disable(DMA1_Channel5);
            MB_RS485.mb_state=STATE_IDLE;
            DMA1_Channel5->CNDTR = MB_FRAME_MAX;
            DMA_Enable(DMA1_Channel5);
        }
	}
    if (USART1->SR & USART_SR_TC)
    {
        USART1->SR = ~USART_SR_TC;
        MB_RS485.mb_state=STATE_IDLE;
        IO_SetLine(io_RS485_Switch, OFF);   //RS485 to recieve
        DMA_Disable(DMA1_Channel4);
    }
}

void DMA1_Channel5_IRQHandler ()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    DMA1->IFCR |= DMA_IFCR_CGIF5;
    DMA_Disable(DMA1_Channel5);
    MB_RS485.mb_state=STATE_IDLE;
    DMA1_Channel5->CNDTR = MB_FRAME_MAX;
    xTimerStopFromISR(rs485_timer_handle, &xHigherPriorityTaskWoken);
    IO_SetLine(io_RS485_Switch, OFF);   //RS485 to recieve
    DMA_Enable(DMA1_Channel5);
}

void DMA1_Channel4_IRQHandler ()
{
    DMA_Disable(DMA1_Channel4);
    DMA1->IFCR |= DMA_IFCR_CGIF4;
    IO_SetLine(io_RS485_Switch, OFF);
    MB_RS485.mb_state=STATE_IDLE;
}

// Callback for usb com
void mh_USB_Recieve(uint8_t *USB_buf, uint16_t len)	//interrupt	function
{
    if (All_Idle_Check((mb_struct*)&MB_USB)==REG_OK)
    {
    	if(len>MB_FRAME_MAX)
    	{
    	len=MB_FRAME_MAX;
    	}
    MB_USB.mb_state=STATE_PARS;
    MB_USB.mb_index=(len);
    memcpy (MB_USB.p_mb_buff,USB_buf,len);
    mb_struct *st_mb=(mb_struct*)&MB_USB;
    xQueueSend(xModbusQueue, &st_mb, 0);
    }
}

// Init modbus
void mh_Modbus_Init(void)
{
    //create queue
    xModbusQueue=xQueueCreate(3,sizeof(mb_struct *));

    //create modbus task
    if(pdTRUE != xTaskCreate(mh_task_Modbus,	"RS485", 	MODBUS_TASK_STACK_SIZE, NULL, MODBUS_TASK_PRIORITY, &m_modbus_task_handle)) ERROR_ACTION(TASK_NOT_CREATE, 0);

    mh_USB_Init();
    mh_RS485_Init();
}

void mh_USB_Init(void)
{
    MB_USB.p_write = MBbuf_main;
    MB_USB.p_read = MBbuf_main;
    MB_USB.reg_read_last=NUM_BUF-1;
    MB_USB.reg_write_last=NUM_BUF-1;
    MB_USB.eep_state=EEP_FREE;
    MB_USB.er_frame_bad=EV_NOEVENT;
    MB_USB.slave_address=MB_ANY_ADDRESS;	//0==any address
    MB_USB.mb_state=STATE_IDLE;
    MB_USB.p_mb_buff=&USB_MB_Buf[0];
    MB_USB.f_save = mh_Write_Eeprom;
    MB_USB.f_start_trans = mh_USB_Transmit_Start;
}

void mh_RS485_Init(void)
{
    uint32_t Rs485_Time_ms;

    MB_RS485.p_write = MBbuf_main;
    MB_RS485.p_read = MBbuf_main;
    MB_RS485.reg_read_last=NUM_BUF-1;
    MB_RS485.reg_write_last=NUM_BUF-1;
    MB_RS485.eep_state=EEP_FREE;
    MB_RS485.er_frame_bad=EV_NOEVENT;
    MB_RS485.slave_address=MBbuf_main[Reg_RS485_Modbus_Address];
    MB_RS485.mb_state=STATE_IDLE;
    MB_RS485.p_mb_buff=&RS485_MB_Buf[0];
    MB_RS485.f_save = mh_Write_Eeprom;
    MB_RS485.f_start_trans=mh_Rs485_Transmit_Start;

    Rs485_Time_ms = (MBbuf_main[Reg_RS485_Ans_Delay]);
    rs485_timer_handle = xTimerCreate( "T_RS485", Rs485_Time_ms/portTICK_RATE_MS, pdFALSE, NULL, rs485_timer_callback);
    IO_Uart1_Init();
}

void rs485_timer_callback (xTimerHandle xTimer)
{
        if( STATE_RCVE == MB_RS485.mb_state)
        {                                               // If we are receiving, it's the end event: t3.5
        MB_RS485.mb_state=STATE_PARS;					// Begin parsing of a frame.
        mb_struct *st_mb=(mb_struct*)&MB_RS485;
        xQueueSend(xModbusQueue, &st_mb, 0);
        }

    return;
}

void IO_Uart1_Init(void)
{
RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;						//USART1 Clock ON
USART1->BRR = Baud_rate[MBbuf_main[Reg_RS485_Baud_Rate]&0x3];	// Bodrate
USART1->CR1  |= USART_CR1_UE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_RE | USART_CR1_TCIE;

      switch (MBbuf_main[Reg_Parity_Stop_Bits])
      {
            case NO_PARITY_1_STOP:
                 break; //default setting

            case NO_PARITY_2_STOP:
                 USART1->CR2  |=  USART_CR2_STOP_1;
                 break;

            case EVEN_PARITY_1_STOP:
                 USART1->CR1  |= USART_CR1_PCE | USART_CR1_M;
                 break;

            case ODD_PARITY_1_STOP:
                 USART1->CR1  |= USART_CR1_PCE | USART_CR1_M | USART_CR1_PS;
                 break;

            default:
            break;
      }

NVIC_SetPriority(USART1_IRQn,14);
NVIC_EnableIRQ (USART1_IRQn);

//DMA RX
DMA_Disable(DMA1_Channel5);		// Выключили канал.
DMA_DeInit_Di(DMA1_Channel5);		// Обнулили DMA канал

USART1->CR3 |=USART_CR3_DMAR;

DMA_Init_Di(  DMA1_Channel5,				// channel
              (uint32_t)&(USART1->DR),		// periphery address/mem_to_mem source
              (uint32_t)RS485_MB_Buf,	    // memory address/mem_to_mem destination
              sizeof(RS485_MB_Buf),		    // registers count
              TransCompl_Int_Enable       +	// interrupt complete
              HalfCompl_Int_Disable       +	// interrupt half complete
              TransError_Int_Enable       +	// interrupt error
              ReadPerif                   +	// read from
              CircularMode_Enable         +	// cyclic mode
              PeripheralInc_Disable       +	// increment periphery mode
              MemoryInc_Enable            +	// increment memory mode
              PDataSize_B                 +	// periphery data size
              MDataSize_B                 +	// memory data size
              DMA_Priority_Hi             +	// priority
              M2M_Disable                 );// memory to memory mode

NVIC_SetPriority(DMA1_Channel5_IRQn,14);
NVIC_EnableIRQ (DMA1_Channel5_IRQn);

//DMA TX
DMA_Disable(DMA1_Channel4);		// Выключили канал.
DMA_DeInit_Di(DMA1_Channel4);		// Обнулили DMA канал

USART1->SR &= ~(USART_SR_TC);
USART1->CR3 |=USART_CR3_DMAT;

DMA_Init_Di(  DMA1_Channel4,				// channel
              (uint32_t)&(USART1->DR),		// periphery address/mem_to_mem source
              (uint32_t)RS485_MB_Buf,	    // memory address/mem_to_mem destination
              100,		                    // registers count
              TransCompl_Int_Disable      +	// interrupt complete
              HalfCompl_Int_Disable       +	// interrupt half complete
              TransError_Int_Enable       +	// interrupt error
              ReadMemory                  +	// read from
              CircularMode_Disable        +	// cyclic mode
              PeripheralInc_Disable       +	// increment periphery mode
              MemoryInc_Enable            +	// increment memory mode
              PDataSize_B                 +	// periphery data size
              MDataSize_B                 +	// memory data size
              DMA_Priority_Hi            +	// priority
              M2M_Disable                 );// memory to memory mode
DMA1->IFCR = DMA_IFCR_CTCIF4;

NVIC_SetPriority(DMA1_Channel4_IRQn,14);
NVIC_EnableIRQ (DMA1_Channel4_IRQn);
}

void mh_Write_Eeprom (void *mbb)
{
    mb_struct *st_mb;
    st_mb = (void*) mbb;
    uint16_t len =  sizeof(default_state[0].Default_Value);

	for (int32_t i = 0; i < (st_mb->eep_indx); i++)
	{
		if(EESave_Check(i+(st_mb->eep_start_save))==REG_OK)
		{
            AT25_FRTOS_update_byte( ((st_mb->eep_start_save)+i)*len, (uint8_t*) &(st_mb->p_write[i+(st_mb->eep_start_save)]), len);
		}
	}
    st_mb->eep_state = EEP_FREE;
}

void mh_USB_Transmit_Start (void *mbb)
{
    mb_struct *st_mb;
    st_mb = (void*) mbb;
    st_mb->mb_index=0;
    while (st_mb->mb_index < st_mb->response_size)
    {
        USB_Send_Data((uint8_t) st_mb->p_mb_buff[st_mb->mb_index++]);
    }
    MB_USB.mb_state=STATE_IDLE;
}

void mh_Rs485_Transmit_Start (void *mbb)
{
    IO_SetLine(io_RS485_Switch, ON);   //RS485 to tx
    mb_struct *st_mb;
    st_mb = (void*) mbb;
    DMA1_Channel4->CNDTR = st_mb->response_size;

    DMA_Enable(DMA1_Channel4);
}

void mh_Factory (void)
{
    taskENTER_CRITICAL();
    uint16_t len =  sizeof(default_state[0].Default_Value);
	for (int32_t i=0; i< NUM_BUF; i++)
	{
		if (EESave_Check(i)==REG_OK)
		{
            MBbuf_main[i] = default_state[i].Default_Value;
            AT25_update_byte((i*len), (uint8_t *) &MBbuf_main[i],  len);
		}
	}
    taskEXIT_CRITICAL();
    MBbuf_main[Reg_Set_Default_Reset]=0;
}

void mh_Buf_Init (void)
{
    taskENTER_CRITICAL();
    AT25_Init();
    uint16_t len =  sizeof(default_state[0].Default_Value);
	for (int32_t i=0; i< NUM_BUF; i++)
	{
		if(EESave_Check(i)==REG_OK)
		{
            AT25_read_byte((i*len), (uint8_t *) &MBbuf_main[i],  len);
			if(Limit_Check(i, MBbuf_main[i])==REG_ERR)
			{
                MBbuf_main[i]=default_state[i].Default_Value;
                AT25_update_byte((i)*len, (uint8_t *) &MBbuf_main[i],  len);
			}
		}
	}
    MBbuf_main[Reg_Set_Default_Reset]=0;
    taskEXIT_CRITICAL();
}
