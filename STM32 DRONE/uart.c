#include "uart.h"

void UART_Init(uint32_t baud)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 TX */

    GPIOA->CRL &= ~(GPIO_CRL_MODE2 |
                    GPIO_CRL_CNF2);

    GPIOA->CRL |= GPIO_CRL_MODE2;
    GPIOA->CRL |= GPIO_CRL_MODE2_1;

    GPIOA->CRL |= GPIO_CRL_CNF2_1;


    /* PA3 RX */

    GPIOA->CRL &= ~(GPIO_CRL_MODE3 |
                    GPIO_CRL_CNF3);

    GPIOA->CRL |= GPIO_CRL_CNF3_0;


    USART2->BRR = 36000000 / baud;

    USART2->CR1 |= USART_CR1_TE;
    USART2->CR1 |= USART_CR1_RE;

    USART2->CR1 |= USART_CR1_UE;
		
}

void UART_SendByte(uint8_t data)
{
    while(!(USART2->SR & USART_SR_TXE));

    USART2->DR=data;
}

void UART_SendArray(uint8_t *data,uint16_t size)
{
    uint16_t i;

    for(i=0;i<size;i++)
    {
        UART_SendByte(data[i]);
    }
}

uint8_t UART_DataReady(void)
{
    return (USART2->SR & USART_SR_RXNE);
}

uint8_t UART_ReadByte(void)
{
    

    return USART2->DR;
}
