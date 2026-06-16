#include "timer.h"
#include "gpio.h"

static volatile uint32_t Tick=0;

void Timer_Init(void)
{
    SysTick_Config(SystemCoreClock/1000);
}

void StatusLedTimer_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 7199;
    TIM3->ARR = 4999;
    TIM3->CNT = 0;
    TIM3->SR &= ~TIM_SR_UIF;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);
}

void SysTick_Handler(void)
{
    Tick++;
}

void TIM3_IRQHandler(void)
{
    if(TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;
        StatusLed_Toggle();
    }
}

uint32_t GetTick(void)
{
    return Tick;
}

void DelayMs(uint32_t ms)
{
    int range = ms * 72000;
		for (int i = 0; i < range; i++)
		{}
		
}

uint8_t Timeout(uint32_t startTime,uint32_t delay)
{
    return ((Tick-startTime)>=delay);
}
