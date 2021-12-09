
#include <modbus_hard.h>
#include <string.h>
//=========================================================================================

extern uint16_t MBbuf_main [];
extern uint32_t Error;
extern TaskHandle_t RS485_Task;
extern TaskHandle_t USB_CDC_Task;

volatile mb_struct MB_RS485;
volatile mb_struct MB_USB;

uint8_t RS485_MB_Buf[MB_FRAME_MAX];
uint8_t USB_MB_Buf[MB_FRAME_MAX];

const uint16_t Baud_rate[4]={BAUD_9600, BAUD_19200, BAUD_57600, BAUD_115200};
const uint32_t Timer_modbus[4]={TIMER_9600,TIMER_19200,TIMER_57600,TIMER_115200};		//Bod value for timer divider calculation (Fcpu*simvol_count)/baudrate/count_timer

uint16_t Timer_15;

extern const t_default_state default_state[];
//#define	limit_min(a)	default_state[a].Min_Level
//#define	limit_max(a)	default_state[a].Max_Level_Mask

//=========================================================================================
//--------------------------------------------------------------------------------------

void Write_Eeprom (void *mbb)
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

void Start_Trans_RS485 (void *mbb)
{
TO_TRANSMT;
}

//--------------------------------------------------------------------------------------

void Start_Trans_USB (void *mbb)
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

void vRS485 (void *pvParameters)
{
vTaskDelay(3000);
MB_RS_Init();
while(1)
    {
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    MBparsing((mb_struct*) &MB_RS485);
    }
}

//--------------------------------------------------------------------------------------

void MB_RS_Init(void)
{

    MB_RS485.p_write = MBbuf_main;
    MB_RS485.p_read = MBbuf_main;
    MB_RS485.reg_read_last=NUM_BUF;
    MB_RS485.reg_write_last=NUM_BUF;
    MB_RS485.eep_state=EEP_FREE;
    MB_RS485.er_frame_bad=EV_NOEVENT;
    MB_RS485.slave_address=MBbuf_main[Reg_RS485_Modbus_Address];
    MB_RS485.mb_state=STATE_IDLE;
    MB_RS485.p_mb_buff=&RS485_MB_Buf[0];
    MB_RS485.f_save = Write_Eeprom;
    MB_RS485.f_start_trans=Start_Trans_RS485;
    IO_Uart1_Init();
    IO_Timer2_Init();
    START_RECEIVE;
}

//--------------------------------------------------------------------------------------

void vUSB_MB (void *pvParameters)
{
MB_USB_Init();

while(1)
    {
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    MBparsing((mb_struct*) &MB_USB);
    }
}

//--------------------------------------------------------------------------------------

void MB_USB_Init(void)
{
    MB_USB.p_write = MBbuf_main;
    MB_USB.p_read = MBbuf_main;
    MB_USB.reg_read_last=NUM_BUF;
    MB_USB.reg_write_last=NUM_BUF;
    MB_USB.eep_state=EEP_FREE;
    MB_USB.er_frame_bad=EV_NOEVENT;
    MB_USB.slave_address=MB_ANY_ADDRESS;	//0==any address
    MB_USB.mb_state=STATE_IDLE;
    MB_USB.p_mb_buff=&USB_MB_Buf[0];
    MB_USB.f_save = Write_Eeprom;
    MB_USB.f_start_trans = Start_Trans_USB;
}

//--------------------------------------------------------------------------------------

void IO_Uart1_Init(void)
{
RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;						//USART1 Clock ON
USART1->BRR = Baud_rate[MBbuf_main[Reg_RS485_Baud_Rate]&0x3];	// Bodrate
USART1->CR1  |= USART_CR1_UE | USART_CR1_TE
				|USART_CR1_RE;									//  |USART_CR1_RXNEIE; // USART1 ON, TX ON, RX ON
NVIC_SetPriority(USART1_IRQn,14);
NVIC_EnableIRQ (USART1_IRQn);
}

//--------------------------------------------------------------------------------------

void IO_Timer2_Init(void)
{
RCC->APB1ENR    |= RCC_APB1ENR_TIM2EN;
TIM2->PSC = (MBbuf_main[Reg_RS485_Silence]*UART_FRIQ)/(Timer_modbus[(MBbuf_main[Reg_RS485_Baud_Rate]&0x3)]*TIMER_FULL); // -1 //(Fcpu*simvol_count)/baudrate/count_timer
TIM2->ARR = TIMER_FULL-1;				         				 // Потолком счета таймера укажем 200-1. Получим деление на 200 в частоте вызова прерываний.
TIM2->CR1   |=  TIM_CR1_URS;                                   	 // | TIM_CR1_CEN;  		//  ARPE=1 - буфферизируем регистр предзагрузки таймера опциональная вещь.
                                                                // URS=1	- разрешаем из событий таймера только события от переполнения
                                                                // CEN=1 - запускаем таймер
TIM2->DIER  |= TIM_DIER_UIE;					                // UIE=1 - Разрешаем прерывание от переполнения
NVIC_SetPriority(TIM2_IRQn,14);			                        // Очень важный момент!!! Надо правильно выставить приоритеты. Они должны быть в диапазоне между                                                                  // configKERNEL_INTERRUPT_PRIORITY  и configMAX_SYSCALL_INTERRUPT_PRIORITY
NVIC_EnableIRQ(TIM2_IRQn);					                    // Разрешаем прерывания от таймера 2 через NVIC

Timer_15 = ((TIMER_FULL*TIMER_15_SYMBOL)/ (MBbuf_main[Reg_RS485_Silence]));
}

//=========================================================================================

void USB_Recieve(uint8_t *USB_buf, uint16_t len)	//interrupt	function
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if(STATE_IDLE == MB_USB.mb_state)
    {
    	if(len>MB_FRAME_MAX)
    	{
    	len=MB_FRAME_MAX;
    	}
    MB_USB.mb_state=STATE_PARS;
    MB_USB.mb_index=(len);
    memcpy (MB_USB.p_mb_buff,USB_buf,len);
    vTaskNotifyGiveFromISR(USB_CDC_Task, &xHigherPriorityTaskWoken);
    }
    else; // MB_USB.mb_state = STATE_IDLE;
}

//--------------------------------------------------------------------------------------

void USART1_IRQHandler (void)
{
 uint16_t cnt;
 if (USART1->SR & USART_SR_RXNE)
	{
    if ((USART1->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE)) == 0)
        {
        cnt = TIMER_COUNT;
       // MBbuf_main[15]=cnt;
       // MBbuf_main[16]=Timer_15;
        RESET_TIMER;							// Timer reset anyway: received symbol means NO SILENCE
        if( STATE_RCVE == MB_RS485.mb_state)
            {
            if( (cnt > Timer_15) || (MB_RS485.mb_index > MB_FRAME_MAX-1))
                {				                         // t1.5
                 MB_RS485.er_frame_bad = EV_HAPPEND;		// This error will be processed later
				 USART1->SR = ~USART_SR_RXNE;            // Nothing more to do in RECEIVE state
                 return;                                       // Last acceptable position is  MB_FRAME_MAX-1
                }                                       // Ignore big frames
            MB_RS485.p_mb_buff[MB_RS485.mb_index++] = BYTE_RECEIVED;	// MAIN DOING: New byte to buffer
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
        STOP_TIMER;
        TO_RECEIVE;
        }
	return;
	}

if(USART1->SR & (USART_SR_NE|USART_SR_FE|USART_SR_PE|USART_SR_ORE))
    {
    cnt = BYTE_RECEIVED;
    MB_RS485.mb_state=STATE_IDLE;
    STOP_TIMER;
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
    	STOP_TIMER;
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
void TIM2_IRQHandler (void)
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
if(TIM2->SR & TIM_SR_UIF)
    {
    TIM2->SR = ~TIM_SR_UIF;
    STOP_TIMER;
    if( STATE_RCVE == MB_RS485.mb_state)
        {												// If we are receiving, it's the end event: t3.5
        MB_RS485.mb_state=STATE_PARS;					// Begin parsing of a frame.
        vTaskNotifyGiveFromISR(RS485_Task, &xHigherPriorityTaskWoken);
        }
    return;
    }
Error|=0x40;
}

//=========================================================================================
