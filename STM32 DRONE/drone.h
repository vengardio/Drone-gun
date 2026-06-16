#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>

void Drone_Init(void);

void Drone_SendPrepare(void);
void Drone_SendCancel(void);
void Drone_SendFly(void);

#endif
