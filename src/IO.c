
#include <stdint.h>
#include <stdbool.h>

#include <stm32f1xx.h>
#include <IO.h>
#include <dma_103.h>
#include <main.h>

volatile uint16_t ADC_DMA[ADC_CHANNEL_ALL * 9];

//--------------X macros---------------------------------------------------------
const tGPIO_Line IOs[NUM_IO] =
{
#define X_IO(a,b,c,d,e,f)	{b,c,d+e,f},
    IO_TABLE
#undef X_IO
};

//---------------------------------------------------------------------------------
void Delay_ms(uint32_t ms)
{
    volatile uint32_t nCount;
//RCC_ClocksTypeDef RCC_Clocks;
//RCC_GetClocksFreq (&RCC_Clocks);
//nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
    nCount=(SYSCLK_FREQ/10000)*ms;
    for (; nCount!=0; nCount--);
}
//---------------------------------------------------------------------------------
void IO_SetLine(tIOLine Line, bool State)
{
    if (State)
    {
        IOs[Line].GPIOx->BSRR = 1 << (IOs[Line].GPIO_Pin);
    }
    else
    {
        IOs[Line].GPIOx->BRR = 1 << (IOs[Line].GPIO_Pin);
    }
}

//---------------------------------------------------------------------------------
bool IO_GetLine(tIOLine Line)
{
    if (Line < NUM_IO)
        return (((IOs[Line].GPIOx->IDR) & (1<<(IOs[Line].GPIO_Pin))) == 0);
    else
        return false;
}
//---------------------------------------------------------------------------------
bool IO_GetLineDO(tIOLine Line)
{
    if (Line < NUM_IO)
        return (((IOs[Line].GPIOx->IDR) & (1<<(IOs[Line].GPIO_Pin))) != 0);
    else
        return false;
}

//---------------------------------------------------------------------------------
void IO_ConfigLine(tIOLine Line, uint8_t Mode, uint8_t State)
{
    if(IOs[Line].GPIO_Pin < 8)
    {
        IOs[Line].GPIOx->CRL &=   ~(0x0F << (IOs[Line].GPIO_Pin * 4));
        IOs[Line].GPIOx->CRL |=  Mode<<(IOs[Line].GPIO_Pin * 4);
    }
    else
    {
        IOs[Line].GPIOx->CRH &=   ~(0x0F << ((IOs[Line].GPIO_Pin - 8)* 4));
        IOs[Line].GPIOx->CRH |=    Mode<<((IOs[Line].GPIO_Pin - 8)* 4);
    }

    IOs[Line].GPIOx->ODR &= ~(1<<IOs[Line].GPIO_Pin);
    IOs[Line].GPIOx->ODR |= State<<IOs[Line].GPIO_Pin;
}
//---------------------------------------------------------------------------------
void IO_SPI_Init(void)
{
    SPI2->CR1 |= 	(SPI_CR1_SSM    //
                     |SPI_CR1_BR_1	//|SPI_CR1_BR_2|SPI_CR1_BR_0
                     |SPI_CR1_MSTR	//
                     |SPI_CR1_SSI);	//

    SPI2->CR1 |= SPI_CR1_SPE;       // ON
}
//---------------------------------------------------------------------------------
void IO_ADC_Init(void)
{
    //-=ADC1=-
    ADC1->SMPR2 |=  ADC_SMPR2_SMP0|ADC_SMPR2_SMP1|ADC_SMPR2_SMP2
                    |ADC_SMPR2_SMP3|ADC_SMPR2_SMP4|ADC_SMPR2_SMP5
                    |ADC_SMPR2_SMP6|ADC_SMPR2_SMP7|ADC_SMPR2_SMP8;		// количество циклов преобразования
    ADC1->SMPR1 |=ADC_SMPR1_SMP16;									// количество циклов преобразования

    /*	//обратный порядок каналов относительно клемм
       ADC1->SQR1 |= ADC_SQR1_L_3;
       ADC1->SQR2 |= ADC_SQR2_SQ7_1|ADC_SQR2_SQ7_2|ADC_SQR2_SQ8_0
       			 |ADC_SQR2_SQ8_1|ADC_SQR2_SQ8_2|ADC_SQR2_SQ9_3;
       ADC1->SQR3 |= ADC_SQR3_SQ2_0|ADC_SQR3_SQ3_1|ADC_SQR3_SQ4_0
       			 |ADC_SQR3_SQ4_1|ADC_SQR3_SQ5_2|ADC_SQR3_SQ6_0
    			 |ADC_SQR3_SQ6_2;
    */
    ADC1->SQR1 |= ADC_SQR1_L_3|ADC_SQR1_L_0;									// добавление в последовательность каналов
    ADC1->SQR2 |= ADC_SQR2_SQ7_1|ADC_SQR2_SQ8_0|ADC_SQR2_SQ10_4;
    ADC1->SQR3 |= ADC_SQR3_SQ1_3|ADC_SQR3_SQ2_0|ADC_SQR3_SQ2_1
                  |ADC_SQR3_SQ2_2|ADC_SQR3_SQ3_1|ADC_SQR3_SQ3_2
                  |ADC_SQR3_SQ4_0|ADC_SQR3_SQ4_2|ADC_SQR3_SQ5_2
                  |ADC_SQR3_SQ6_0|ADC_SQR3_SQ6_1;

    ADC1->CR1 |= ADC_CR1_SCAN;									// включаем автоматический перебор всех каналов в последовательности

    ADC1->CR2 |= ADC_CR2_DMA|ADC_CR2_EXTTRIG|ADC_CR2_EXTSEL|ADC_CR2_CONT|ADC_CR2_TSVREFE;

    //-=ADC2=-
    ADC2->CR1       |=  ADC_CR1_JEOCIE;
    ADC2->CR2       |=  ADC_CR2_JEXTTRIG|ADC_CR2_JEXTSEL;
    ADC2->SMPR2     |=  ADC_SMPR2_SMP9_2;//|ADC_SMPR2_SMP9_0;	// количество циклов преобразования
    ADC2->JSQR      |=  ADC_JSQR_JSQ4_0|ADC_JSQR_JSQ4_3;		// добавление в последовательность каналов

    //-=ON=-
    ADC1->CR2  |= (ADC_CR2_ADON);								//вкл
    ADC2->CR2  |= (ADC_CR2_ADON);								//вкл

    //-=calibration=-
    Delay_ms(1);
    ADC1->CR2  |= (ADC_CR2_RSTCAL);
    while ( ADC1->CR2  & (ADC_CR2_RSTCAL))
        ;

    ADC1->CR2  |= (ADC_CR2_CAL);
    while ( ADC1->CR2  & (ADC_CR2_CAL))
        ;

    ADC2->CR2  |= (ADC_CR2_RSTCAL);
    while ( ADC2->CR2  & (ADC_CR2_RSTCAL))
        ;

    ADC2->CR2  |= (ADC_CR2_CAL);
    while ( ADC2->CR2  & (ADC_CR2_CAL))
        ;
}

//---------------------------------------------------------------------------------
/**
 * @brief getADCval - calculate median value for `nch` channel
 * @param nch - number of channel
 * @return
 */
uint16_t getADCval(int nch)
{
    int i, addr = nch;
#define PIX_SORT(a,b) { if ((a)>(b)) PIX_SWAP((a),(b)); }
#define PIX_SWAP(a,b) {register uint16_t temp=(a);(a)=(b);(b)=temp; }
    uint16_t p[9];
    for(i = 0; i < 9; ++i, addr += ADC_CHANNEL_ALL) // first we should prepare array for optmed
        p[i] = ADC_DMA[addr];
    PIX_SORT(p[1], p[2]) ;
    PIX_SORT(p[4], p[5]) ;
    PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[1]) ;
    PIX_SORT(p[3], p[4]) ;
    PIX_SORT(p[6], p[7]) ;
    PIX_SORT(p[1], p[2]) ;
    PIX_SORT(p[4], p[5]) ;
    PIX_SORT(p[7], p[8]) ;
    PIX_SORT(p[0], p[3]) ;
    PIX_SORT(p[5], p[8]) ;
    PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[3], p[6]) ;
    PIX_SORT(p[1], p[4]) ;
    PIX_SORT(p[2], p[5]) ;
    PIX_SORT(p[4], p[7]) ;
    PIX_SORT(p[4], p[2]) ;
    PIX_SORT(p[6], p[4]) ;
    PIX_SORT(p[4], p[2]) ;
    return p[4];
#undef PIX_SORT
#undef PIX_SWAP
}

//---------------------------------------------------------------------------------
// return MCU temperature (degrees of celsius * 10)
int32_t getMCUtemp()
{
    int32_t ADval = getADCval(ADC_N_CHANNEL_T_MCU);
    int32_t temperature = (1430-((mV_ADC*ADval)/ADC_COUNTS));
    temperature *= (int32_t)(10);
    temperature /= (int32_t)(43);
    temperature += 25;
    return(temperature);
}
//---------------------------------------------------------------------------------
void IO_Init(void)
{
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    RCC->APB2ENR	|= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR 	|= RCC_APB2ENR_IOPBEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPCEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPDEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPEEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPFEN;
//RCC->APB2ENR	|= RCC_APB2ENR_IOPGEN;

    RCC->APB2ENR	|= RCC_APB2ENR_AFIOEN;

// for PA15
    AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

// Set all pins
    for (int Line = 0; Line < NUM_IO; Line++)
    {
        IO_ConfigLine(Line, IOs[Line].MODE, IOs[Line].DefState);
    }

// SPI
    RCC->APB1ENR    |= RCC_APB1ENR_SPI2EN;
    IO_SPI_Init();

// ADC
    RCC->CFGR 		&= ~RCC_CFGR_ADCPRE;
    RCC->CFGR		|= RCC_CFGR_ADCPRE_DIV8;
    RCC->APB2ENR	|= RCC_APB2ENR_ADC1EN|RCC_APB2ENR_ADC2EN;

    IO_ADC_Init();

    NVIC_SetPriority(ADC1_2_IRQn,14);
    NVIC_EnableIRQ(ADC1_2_IRQn);

// DMA
    RCC->AHBENR     |= RCC_AHBENR_DMA1EN;

    DMA_Disable(DMA1_Channel1);		    // Выключили канал.
    DMA_DeInit_Di(DMA1_Channel1);		// Обнулили DMA канал

    DMA_Init_Di(  DMA1_Channel1,				// 1 канал 1 контроллера.
                  (uint32_t)&ADC1->DR,		    // Адрес откуда брать -- адрес регистра DR  в USART1
                  (uint32_t)&ADC_DMA[0],	    // Адрес куда класть результат
                  (ADC_CHANNEL_ALL*9),    		// Сколько класть? Так как буфер у нас из char, то sizeof будет равен числу элементов. Но лучше так не делать ;)
                  TransCompl_Int_Disable    +	// Прерывание по окончанию выключено
                  HalfCompl_Int_Disable     +	// Прерывание по половине выключено
                  TransError_Int_Disable    +	// Прерывание по ошибке выключено
                  ReadPerif                 +	// Читаем из периферии
                  CircularMode_Enable       +	// Цикличный режим включен
                  PeripheralInc_Disable     +	// Адрес периферии не увеличиваем
                  MemoryInc_Enable          +	// А вот адрес примного буфера увеличиваем, перебирая байт за байтом его
                  PDataSize_W               +	// Размер данных из периферии - word16
                  MDataSize_W               +	// Размер данных в памяти - word16
                  DMA_Priority_Hi			+	// Низкий приоритет
                  M2M_Disable                 );	// Режим копирования память-память выключен.
//Включаем DMA - поехали!
    DMA_Enable(DMA1_Channel1);
    ADC1->CR2 |= ADC_CR2_SWSTART;
//NVIC_SetPriority(DMA1_Channel1_IRQn,14);
//NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}




