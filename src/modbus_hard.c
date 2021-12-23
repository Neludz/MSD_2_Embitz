
#include <modbus_hard.h>
#include <string.h>
//=========================================================================================

extern uint16_t MBbuf_main [];
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

//=========================================================================================
//--------------------------------------------------------------------------------------

void mh_Write_Eeprom (void *mbb)
{
mb_struct *st_mb;
st_mb = (void*) mbb;
uint16_t len =  sizeof(default_state[0].Default_Value);

	for (int32_t i = 0; i < (st_mb->eep_indx); i++)
	{
		if(default_state[i+(st_mb->eep_start_save)].Permission & EESAVE)
		{
		AT25_update_byte( ((st_mb->eep_start_save)+i)*len, (uint8_t*) &(st_mb->p_write[i+(st_mb->eep_start_save)]), len);
		}
	}
vTaskDelay((1500/portTICK_RATE_MS));
st_mb->eep_state = EEP_FREE;

}
//--------------------------------------------------------------------------------------

void mh_MB_Init(void)
{
    BaseType_t rtos_result;

    //create queue
    xModbusQueue=xQueueCreate(4,sizeof(mb_struct *));

    //create modbus task
    if(pdTRUE != xTaskCreate(mh_task_Modbus,	"RS485", 	MODBUS_TASK_STACK_SIZE, NULL, MODBUS_TASK_PRIORITY, &m_modbus_task_handle)) ERROR_ACTION(TASK_NOT_CREATE, 0);

    mh_USB_Init();
    mh_RS485_Init();
}
//--------------------------------------------------------------------------------------

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
//--------------------------------------------------------------------------------------

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
//--------------------------------------------------------------------------------------

void mh_USB_Recieve(uint8_t *USB_buf, uint16_t len)	//interrupt	function
{
    if(STATE_IDLE == MB_USB.mb_state)
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
//--------------------------------------------------------------------------------------

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
        if(Rs485_Time_ms < MIN_TIME_TO_START_TRANSMIT_MS)
        {
        Rs485_Time_ms=MIN_TIME_TO_START_TRANSMIT_MS;
        }
    rs485_timer_handle = xTimerCreate( "T_RS485", Rs485_Time_ms/portTICK_RATE_MS, pdFALSE, NULL, rs485_timer_callback);
    IO_Uart1_Init();
}
//--------------------------------------------------------------------------------------

void mh_Rs485_Transmit_Start (void *mbb)
{
TO_TRANSMT;
}
//--------------------------------------------------------------------------------------

static void rs485_timer_callback (xTimerHandle xTimer)
{
        if( STATE_RCVE == MB_RS485.mb_state)
        {                                               // If we are receiving, it's the end event: t3.5
        MB_RS485.mb_state=STATE_PARS;					// Begin parsing of a frame.
        mb_struct *st_mb=(mb_struct*)&MB_RS485;
        xQueueSend(xModbusQueue, &st_mb, 0);
        }
    return;
}
//--------------------------------------------------------------------------------------

void IO_Uart1_Init(void)
{
RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;						//USART1 Clock ON
USART1->BRR = Baud_rate[MBbuf_main[Reg_RS485_Baud_Rate]&0x3];	// Bodrate
USART1->CR1  |= USART_CR1_UE | USART_CR1_TE
				|USART_CR1_RE;									//  |USART_CR1_RXNEIE; // USART1 ON, TX ON, RX ON
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
}

//--------------------------------------------------------------------------------------

void USART1_IRQHandler (void)
{
 BaseType_t xHigherPriorityTaskWoken = pdFALSE;
 uint16_t cnt;
 if (USART1->SR & USART_SR_RXNE)
	{
    if ((USART1->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE)) == 0)
        {
        xTimerResetFromISR(rs485_timer_handle, &xHigherPriorityTaskWoken);	// Timer reset anyway: received symbol means NO SILENCE
        if( STATE_RCVE == MB_RS485.mb_state)
            {
            if(MB_RS485.mb_index > MB_FRAME_MAX-1)
                {
                 MB_RS485.er_frame_bad = EV_HAPPEND;	                 // This error will be processed later
				 USART1->SR = ~USART_SR_RXNE;                            // Nothing more to do in RECEIVE state
                 return;                                                 // Last acceptable position is  MB_FRAME_MAX-1
                }                                                        // Ignore big frames
            MB_RS485.p_mb_buff[MB_RS485.mb_index++] = BYTE_RECEIVED;	 // MAIN DOING: New byte to buffer
            }
        else if( STATE_IDLE == MB_RS485.mb_state)
            {											// 1-st symbol come!
        	MB_RS485.p_mb_buff[0] = BYTE_RECEIVED;		// Put it to buffer
        	MB_RS485.mb_index = 1;						// "Clear" the rest of buffer
        	MB_RS485.er_frame_bad = EV_NOEVENT;			// New buffer, no old events
        	MB_RS485.mb_state=STATE_RCVE;				// MBMachine: begin of receiving the request
            }
        else
        	{
        	Error|=0x80;
        	}
        }
    else
        {
    	Error|=0x4;
        cnt = BYTE_RECEIVED;
        MB_RS485.mb_state=STATE_IDLE;
        xTimerStopFromISR(rs485_timer_handle, &xHigherPriorityTaskWoken);
        TO_RECEIVE;
        }
	return;
	}

if(USART1->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE))
    {
    cnt = BYTE_RECEIVED;
    MB_RS485.mb_state=STATE_IDLE;
    xTimerStopFromISR(rs485_timer_handle, &xHigherPriorityTaskWoken);
    Error|=0x8;
    TO_RECEIVE;
    return;
    }

if ((USART1->SR & USART_SR_TXE)&&(USART1->CR1 & USART_CR1_TXEIE))
    {
    if( STATE_SEND == MB_RS485.mb_state)
        {
        if( MB_RS485.mb_index < MB_RS485.response_size)
            {
            BYTE_TO_SEND = MB_RS485.p_mb_buff[MB_RS485.mb_index++];//  sending of the next byte
			}
        else
            {
            MB_RS485.mb_state=STATE_SENT;
            TXEIE_OFF;				// Frame sent. Wait for the last bit to be sent
            }
		}
    else
        {
    	Error|=0x10;
    	xTimerStopFromISR(rs485_timer_handle, &xHigherPriorityTaskWoken);
        TO_RECEIVE;
        MB_RS485.mb_state=STATE_IDLE;
        }
    return;
    }

if (USART1->SR & USART_SR_TC)
    {
    USART1->SR = ~USART_SR_TC;
    TO_RECEIVE; 							// RS-485 driver - to receive mode
    MB_RS485.mb_state=STATE_IDLE;
    return;
    }
Error|=0x20;
}
//--------------------------------------------------------------------------------------

void mh_task_Modbus (void *pvParameters)
{
mb_struct *st_mb;
vTaskDelay(3000);

//start recieve rs485 data
START_RECEIVE;
    while(1)
    {
    xQueueReceive(xModbusQueue,&st_mb,portMAX_DELAY);
    MBparsing((mb_struct*) st_mb);
    }
}
//-------------------------------------------------------------------------

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

//-------------------------------------------------------------------------

void mh_Buf_Init (void)
{
taskENTER_CRITICAL();
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

//=========================================================================================
