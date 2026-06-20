#include "usart.h"

void USART_Init(uint32_t baud)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;

    GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0;

    GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOA->CRL |= GPIO_CRL_MODE2 | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1;

    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
    GPIOA->CRL |= GPIO_CRL_CNF3_0;

    USART2->BRR = 36000000 / baud;
    USART1->BRR = 72000000 / baud;

    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RE;
    USART1->CR1 |= USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE;

    USART2->CR1 |= USART_CR1_TE;
    USART2->CR1 |= USART_CR1_RE;
    USART2->CR1 |= USART_CR1_RXNEIE;
    USART2->CR1 |= USART_CR1_UE;

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_SendByte(uint8_t data)
{
    while(!(USART2->SR & USART_SR_TXE));

    USART2->DR = data;
}

void USART2_SendArray(uint8_t *data, uint16_t size)
{
    uint16_t i;

    for(i=0;i<size;i++)
        USART2_SendByte(data[i]);
}

void USART1_SendByte(uint8_t data)
{
    while(!(USART1->SR & USART_SR_TXE));

    USART1->DR = data;
}

void USART1_SendArray(uint8_t *data, uint16_t size)
{
    uint16_t i;

    for(i=0;i<size;i++)
        USART1_SendByte(data[i]);
}

