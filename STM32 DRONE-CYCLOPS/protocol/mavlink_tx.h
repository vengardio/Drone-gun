#ifndef MAVLINK_TX_H
#define MAVLINK_TX_H

void MavlinkTx_Init(void);

void MavlinkTx_SendPrepare(void);
void MavlinkTx_SendCancel(void);
void MavlinkTx_SendFly(void);

#endif
