#ifndef MAVLINK_RX_H
#define MAVLINK_RX_H

#include "message_queue.h"
#include "common/mavlink.h"
#include <stdint.h>

void Mavlink_RxByte(uint8_t data);
uint8_t MavlinkRx_MessageIsHeartbeat(Message *msg);
uint8_t Convert_msg_to_mavlink(
    Message *raw_msg,
    mavlink_message_t *mav_msg
);

#endif
