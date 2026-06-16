#ifndef MAVLINK_RX_H
#define MAVLINK_RX_H

#include "message_queue.h"
#include <stdint.h>

void Mavlink_RxByte(uint8_t data);
uint8_t MavlinkRx_MessageIsHeartbeat(Message *msg);

#endif
