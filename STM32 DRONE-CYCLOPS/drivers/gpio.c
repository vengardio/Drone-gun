#include "gpio.h"

void GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    /* PA0 LAUNCH */

    GPIOA->CRL &= ~(GPIO_CRL_MODE0|GPIO_CRL_CNF0);
		GPIOA->CRL |= GPIO_CRL_CNF0_1;

		GPIOA->ODR &= ~GPIO_ODR_ODR0;   // Pull-Down

    /* PA7 SAFETY */

    GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
		GPIOA->CRL |= GPIO_CRL_CNF7_1;

		GPIOA->ODR &= ~GPIO_ODR_ODR7;   // Pull-Down

    /* PC4 END */

		GPIOC->CRL &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
		GPIOC->CRL |= GPIO_CRL_CNF4_1;
		GPIOC->ODR &= ~GPIO_ODR_ODR4;

		/* PA6 Output Push-Pull 2 MHz */

		GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
		GPIOA->CRL |= GPIO_CRL_MODE6_1;
		
		/* PA5 Output Push-Pull 2 MHz */
		GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
		GPIOA->CRL |= GPIO_CRL_MODE5_1;
		
		//PB12 Output
		GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
		GPIOB->CRH |= GPIO_CRH_MODE12_1;
		
		//PB13 Output
		GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
		GPIOB->CRH |= GPIO_CRH_MODE13_1;
}

uint8_t Launch_Read(void)
{
    return (GPIOA->IDR & GPIO_IDR_IDR0)!=0;
}

uint8_t Safety_Read(void)
{
    return (GPIOA->IDR & GPIO_IDR_IDR7)!=0;
}

uint8_t End_Read(void)
{
    return (GPIOC->IDR & GPIO_IDR_IDR4)!=0;
}

void Optocouplers_Enable(void)
{
    GPIOB->BSRR = GPIO_BSRR_BS12;
    GPIOB->BSRR = GPIO_BSRR_BS13;
}

void StatusLed_Toggle(void)
{
    GPIOA->ODR ^= GPIO_ODR_ODR5;
}
