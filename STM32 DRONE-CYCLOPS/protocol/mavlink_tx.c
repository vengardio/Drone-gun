#include "mavlink_tx.h"
#include "usart.h"
#include "common/mavlink.h"

#define DRONE_TARGET_SYSTEM       1
#define DRONE_TARGET_COMPONENT    0
#define GUN_SYSTEM_ID             255
#define GUN_COMPONENT_ID          0

#define DRONE_CMD_SW_RESET        31001
#define DRONE_CMD_HW_RESET        31002
#define DRONE_CMD_PREPARE         31003
#define DRONE_CMD_GET_DETECTIONS  31004
#define DRONE_CMD_CANCEL          31006
#define DRONE_CMD_FLY             31009

static void MavlinkTx_SendCommand(uint16_t command)
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
    USART2_SendArray(buffer, len);
}

void MavlinkTx_Init(void)
{
    mavlink_set_proto_version(MAVLINK_COMM_0, 2);
}

void MavlinkTx_SwReset(void)
{
    MavlinkTx_SendCommand(DRONE_CMD_SW_RESET);
}

void MavlinkTx_HwReset(void)
{
    MavlinkTx_SendCommand(DRONE_CMD_HW_RESET);
}

void MavlinkTx_SendPrepare(void)
{
    MavlinkTx_SendCommand(DRONE_CMD_PREPARE);
}

void MavlinkTx_SendCancel(void)
{
    MavlinkTx_SendCommand(DRONE_CMD_CANCEL);
}

void MavlinkTx_SendFly(void)
{
    MavlinkTx_SendCommand(DRONE_CMD_FLY);
}
