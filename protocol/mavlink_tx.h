#ifndef MAVLINK_TX_H
#define MAVLINK_TX_H

void MavlinkTx_Init(void);

void MavlinkTx_SwReset(void);
void MavlinkTx_HwReset(void);
void MavlinkTx_SendPrepare(void);
void MavlinkTx_SendCancel(void);
void MavlinkTx_SendFly(void);
void MavlinkTx_SendGetDetections(void);
void MavlinkTx_SendInit(void);

#define MAVLINK_MSGID_HEARTBEAT 0
#define MAVLINK_MSGID_SYSTEM_TIME 2
#define MAVLINK_MSGID_BATTERY_STATUS 147
#define MAVLINK_MSGID_DEBUG 254
#define MAVLINK_MSGID_ACK 77

#endif
