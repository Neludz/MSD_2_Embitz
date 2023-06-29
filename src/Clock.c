#include "stm32f1xx_ll_rcc.h"

// Config

#if SYSCLK_FREQ > HSE_VALUE
#define USE_PLL
#endif

// RCC Structure
// HSI8MHZ--/2-------,                   ,--USB
// HSE---------------+-PLL--SYSCLK---AHB-+--APB1---------
//        '----HSE/2-'                   '--APB2--+------
//                                                '--ADC-

void ClockInit(void)
{
#ifdef USE_HSE
    LL_RCC_HSE_Enable();


    while(!LL_RCC_HSE_IsReady ())
    {
        // Wait HSE
    }
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
#endif

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);                                     //MAX 72MHz
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);                                      // MAX 36 MHz
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);                                      // MAX 72MHz
    LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);                      // Must be 48Mhz
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_8);                         // 14MHz Max feeding from Source APB2


#ifdef USE_PLL
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1,LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();

    while(!LL_RCC_PLL_IsReady())
    {
        // Wait PLL
    }


#ifndef SYSCLK_FREQ
#define SYSCLK_FREQ 72000000U
#warning SYSCLK_FREQ set up to MAX automatically!!! Check this shit!
#endif

    /* Enable Prefetch Buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;

    /* Flash 2 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);

#if SYSCLK_FREQ<24000000U
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0; // Если SystemCoreClock <= 24 МГц
#endif

#if SYSCLK_FREQ>=24000000U && SYSCLK_FREQ<48000000U
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_1; // Если 24< SystemCoreClock <= 48, через такт.
#endif

#if SYSCLK_FREQ>=48000000U
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;  // Если больше 48, то через два такта.
#endif

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
#endif



}
