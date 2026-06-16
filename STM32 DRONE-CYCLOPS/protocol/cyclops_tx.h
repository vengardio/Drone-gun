#ifndef CYCLOPS_TX_H
#define CYCLOPS_TX_H

#include <stdint.h>

uint16_t CyclopsTx_BuildPacket(uint8_t adr,
                               uint8_t packet_id,
                               uint8_t cmd,
                               uint8_t *data,
                               uint8_t data_count,
                               uint8_t *out);

#endif
