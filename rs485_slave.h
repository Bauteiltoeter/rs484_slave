#ifndef __RS485_SLAVE_H
#define __RS485_SLAVE_H

#define RS485_BAUD 115200

#define RS485_CTL_DDR DDRC
#define RS485_CTL_PORT PORTC
#define RS485_RX 5
#define RS485_TX 4

#define HW_TYPE 2

void rs485_init(void);
void rs485_tx(void);
void rs485_rx(void);
void rs485_tx_autoreset(uint8_t ms);
void rs485_run(void);

//MSG 2
typedef struct {
    char characters[4*20];
    uint32_t dots[4];
}msg_complete_update_t;

extern msg_complete_update_t msgstore_complete_update;

#endif
