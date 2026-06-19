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
    }
}




int main(void)
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
	
	Servo_Release();
	
	uint8_t EndPrev = End_Read();

  while(1)
  {
		if (!EndPrev && End_Read()) {
			if (End_Read()) {
				EndPrev = End_Read();
				DelayMs(200);
				Servo_Hold();
			}
		}
		
		
    if(cyclops_connected) {
			Main_ReadMessages();
    } 
	  else {
			LogicDrone_Process();
    }
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
  if(USART2->SR & USART_SR_RXNE)  Mavlink_RxByte((uint8_t)USART2->DR);
}
