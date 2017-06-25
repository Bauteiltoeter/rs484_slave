#ifndef __RS485_SLAVE_H
#define __RS485_SLAVE_H

#include "messagehandler.h"

#define RS485_BAUD 115200


void rs485_init(void);
void rs485_tx(void);
void rs485_rx(void);
void rs485_tx_autoreset(uint8_t ms);
void rs485_run(void);

extern void process_message(uint16_t msg_id, uint8_t length, uint8_t* data);
extern void answer_message(uint16_t msg_id, uint8_t* buffer);

#endif
