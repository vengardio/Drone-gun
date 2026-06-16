#ifndef LOGIC_DRONE_H
#define LOGIC_DRONE_H

#include "stm32f10x.h"
#include "message_queue.h"

typedef enum
{
    LOGIC_DRONE_WAIT_END=0,
    LOGIC_DRONE_WAIT_SAFETY,
    LOGIC_DRONE_PREPARED,
    LOGIC_DRONE_FIRED,
    LOGIC_DRONE_CANCELLED

}LogicDroneState;

typedef enum
{
    DRONE_STATE_IDLE=0,
    DRONE_STATE_PREPARED,
    DRONE_STATE_CANCELLED,
    DRONE_STATE_FIRED

}DroneState;

extern LogicDroneState logic_drone_state;

void LogicDrone_Init(void);
void LogicDrone_Process(void);
void LogicDrone_ProcessMessage(Message *msg);

void LogicDrone_SetDroneState(DroneState state);
DroneState LogicDrone_GetDroneState(void);

#endif
