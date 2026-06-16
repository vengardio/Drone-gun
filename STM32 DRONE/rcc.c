#include "stm32f10x.h"
#include "rcc.h"

int RCC_Init(void)
{
    __IO uint32_t StartUpCounter;
    int result=0;

    /*======================
      HSE on
    ======================*/

    RCC->CR |= RCC_CR_HSEON;

    for(StartUpCounter=0;
        StartUpCounter<0x1000;
        StartUpCounter++)
    {
        if(RCC->CR & RCC_CR_HSERDY)
            break;
    }

    if(RCC->CR & RCC_CR_HSERDY)
    {
        /* HSE=8MHz */

        RCC->CFGR &= ~RCC_CFGR_PLLSRC;
        RCC->CFGR |= RCC_CFGR_PLLSRC;

        /* PLL×9 */

        RCC->CFGR &= ~RCC_CFGR_PLLMULL;
        RCC->CFGR |= RCC_CFGR_PLLMULL9;

        result=0;
    }

    else
    {
        /*======================
          HSI fallback
        ======================*/

        RCC->CR &= ~RCC_CR_HSEON;

        RCC->CR |= RCC_CR_HSION;

        while(!(RCC->CR & RCC_CR_HSIRDY));

        /*
        HSI/2=4MHz
        4×16=64MHz
        */

        RCC->CFGR &= ~RCC_CFGR_PLLSRC;

        RCC->CFGR &= ~RCC_CFGR_PLLMULL;
        RCC->CFGR |= RCC_CFGR_PLLMULL16;

        result=1;
    }

    /*======================
      FLASH
    ======================*/
		
    /*FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;*/
		

    /*======================
      prescalers
    ======================*/

    /* AHB=SYSCLK */

    RCC->CFGR &= ~RCC_CFGR_HPRE;

    /* APB2=SYSCLK */

    RCC->CFGR &= ~RCC_CFGR_PPRE2;

    /* APB1=SYSCLK/2 */

    RCC->CFGR &= ~RCC_CFGR_PPRE1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    /*======================
      PLL ON
    ======================*/

    RCC->CR |= RCC_CR_PLLON;

    while(!(RCC->CR & RCC_CR_PLLRDY));

    /*======================
      SYSCLK=PLL
    ======================*/

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while((RCC->CFGR &
          RCC_CFGR_SWS)
          != RCC_CFGR_SWS_PLL);

    return result;
}