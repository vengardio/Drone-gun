#ifndef FSM_H
#define FSM_H

#include "stm32f10x.h"

typedef enum
{
    FSM_WAIT_END=0,
    FSM_WAIT_SAFETY,
    FSM_PREPARED,
    FSM_FIRED,
    FSM_CANCELLED

}FSM_State;

extern FSM_State State;

void FSM_Init(void);
void FSM_Process(void);

#endif
