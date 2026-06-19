#ifndef CYCLOPS_TX_H
#define CYCLOPS_TX_H

#include <stdint.h>
#include "message_queue.h"

#define CYCLOPS_MASTER_ADDRESS  0xCA

uint16_t CyclopsTx_BuildPacket(uint8_t adr,
                               uint8_t packet_id,
                               uint8_t cmd,
                               uint8_t *data,
                               uint8_t data_count,
                               uint8_t *out);
															 
void Cyclops_SendResponse(uint8_t packet_id, uint8_t cmd, uint8_t *payload, uint8_t payload_size);
	
#endif
