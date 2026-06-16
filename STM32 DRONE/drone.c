#include "drone.h"
#include "uart.h"
#include "/common/mavlink.h"

#define DRONE_TARGET_SYSTEM       1
#define DRONE_TARGET_COMPONENT    0
#define GUN_SYSTEM_ID             255
#define GUN_COMPONENT_ID          0

#define DRONE_CMD_PREPARE         31003
#define DRONE_CMD_CANCEL          31006
#define DRONE_CMD_FLY             31009

static void Drone_SendCommand(uint16_t command)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len;

    mavlink_msg_command_long_pack(
        GUN_SYSTEM_ID,
        GUN_COMPONENT_ID,
        &msg,
        DRONE_TARGET_SYSTEM,
        DRONE_TARGET_COMPONENT,
        command,
        0,
        0,0,0,0,0,0,0
    );

    len = mavlink_msg_to_send_buffer(buffer, &msg);
    UART_SendArray(buffer, len);
}

void Drone_Init(void)
{
    mavlink_set_proto_version(MAVLINK_COMM_0, 2);
}

void Drone_SendPrepare(void)
{
    Drone_SendCommand(DRONE_CMD_PREPARE);
}

void Drone_SendCancel(void)
{
    Drone_SendCommand(DRONE_CMD_CANCEL);
}

void Drone_SendFly(void)
{
    Drone_SendCommand(DRONE_CMD_FLY);
}
