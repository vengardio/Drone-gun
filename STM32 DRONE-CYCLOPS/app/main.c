#include "main.h"

#include "rcc.h"
#include "gpio.h"
#include "usart.h"
#include "timer.h"

#include "pwm.h"
#include "mavlink_tx.h"
#include "mavlink_rx.h"
#include "cyclops_rx.h"
#include "message_queue.h"
#include "logic_cyclops.h"
#include "logic_drone.h"

#include <stdint.h>



static uint8_t cyclops_connected = 0;

static void Main_ReadMessages(void)
{
    Message msg;

    while(MessageQueue_Pop(&msg))
    {
        if(msg.source == MESSAGE_SOURCE_CYCLOPS)
            LogicCyclops_ProcessMessage(&msg);

        if(msg.source == MESSAGE_SOURCE_DRONE)
        {
            LogicCyclops_ProcessDroneMessage(&msg);
            LogicDrone_ProcessMessage(&msg);
        }
    }
}

void App_Init(void)
{
    RCC_Init();
    GPIO_Init();
    Optocouplers_Enable();

    Timer_Init();
    StatusLedTimer_Init();
    USART_Init(115200);

    PWM_InitServo();
    MavlinkTx_Init();
    LogicDrone_Init();
}

void App_Process(void)
{
    if(cyclops_connected)
    {
        LogicCyclops_Process();
        Main_ReadMessages();
    }
    else
    {
        Main_ReadMessages();
        LogicDrone_Process();
    }
}



int main(void)
{
    App_Init();

    while(1)
    {
        App_Process();
    }
}



void USART1_IRQHandler(void)
{
    if(USART1->SR & USART_SR_RXNE)
    {
        cyclops_connected = 1;
        Cyclops_RxByte((uint8_t)USART1->DR);
    }
}

void USART2_IRQHandler(void)
{
    if(USART2->SR & USART_SR_RXNE)
        Mavlink_RxByte((uint8_t)USART2->DR);
}
