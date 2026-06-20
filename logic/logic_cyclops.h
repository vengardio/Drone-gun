#ifndef LOGIC_CYCLOPS_H
#define LOGIC_CYCLOPS_H

#include "message_queue.h"
#include <stdbool.h>

#define CYCLOPS_MASTER_ADDRESS  0xCA
#define CMD_UNKNOWN             0x00
#define CMD_PING                0x81
#define CMD_HW_RESET            0x82
#define CMD_SW_RESET            0x83
#define CMD_GET_ERROR           0x84
#define CMD_ZEROIZE_ERROR       0x85
#define CMD_SET_PARAM           0x86
#define CMD_GET_PARAM           0x87

#define PARAM_GET_ACC_STATE          0x01
#define PARAM_GET_DRONE_STATE  0x02
#define PARAM_GET_DRONE_INFO   0x03
#define PARAM_ENGINES_START_STOP     0x12
#define PARAM_DRONE_LAUNCH     0x13
#define PARAM_SET_ORIENT             0x14

#define MAVLINK_MSGID_HEARTBEAT 0
#define MAVLINK_MSGID_SYSTEM_TIME 2
#define MAVLINK_MSGID_BATTERY_STATUS 147
#define MAVLINK_MSGID_DEBUG 254

#define DRONE_REQUEST_TIMEOUT 5000

void LogicCyclops_ProcessMessage(Message *msg);

#endif
