#include <avr/io.h>
#include <avr/eeprom.h>
#include <string.h>
#include "rs485_slave.h"
#include "uart.h"
#include "rs485_configuration.h"

uint16_t stored_id EEMEM;
uint16_t id=0xFFFF;
uint32_t clock=0;
uint32_t autoreset=0;

static void process_byte(uint8_t b);
static void process_new_id(uint16_t new_id);
static uint16_t calc_checksum(uint8_t *data, uint16_t length);



void rs485_init(void)
{
    RS485_init( UART_BAUD_SELECT(RS485_BAUD,F_CPU) );
    RS485_CTL_DDR |= (1<<RS485_RX) | (1<<RS485_TX);
    rs485_rx();

    id = eeprom_read_word(&stored_id);

    if(id==0xFFFF)
        uart_puts("No ID configured, waiting for ID\r\n");
    else
        uart_puts("Found ID\r\n");

}

void rs485_tx(void)
{
    //Activate transmitter and deactivate receiver
    RS485_CTL_PORT|=(1<<RS485_RX) | (1<<RS485_TX);
}

void rs485_tx_autoreset(uint8_t ms)
{
    rs485_tx();
    autoreset=clock+ms;
}

void rs485_rx(void)
{
    //Deactivate transmitter and activate receiver
    RS485_CTL_PORT&=~((1<<RS485_RX) | (1<<RS485_TX));
}

void rs485_run(void)
{
    clock++;

    if(clock==autoreset)
        rs485_rx();


    unsigned int c;
    while (!((c= RS485_getc()) & UART_NO_DATA))
    {
        if(c & UART_FRAME_ERROR)
        {

        }
        else if(c & UART_PARITY_ERROR)
        {

        }
        else
        {
            process_byte(c);
        }
    }
}

typedef enum {
    WAITING_FOR_START,
    WAIT_ID,
    WAIT_NEW_ID,
    WAIT_MSG_INFO,
    WAIT_DATA,
    IGNORE_BYTES
} state_t;

static void process_byte(uint8_t b)
{
    char t[50];
  //  sprintf(t,"R: %d\r\n",b);
  //  uart_puts(t);

    static state_t state = WAITING_FOR_START;
    static uint8_t multibytecounter=0;
    static uint16_t ignorecounter;
    static uint16_t package_id=0;
    static uint16_t new_id;
    static uint16_t msg_id;
    static uint8_t type;
    static uint8_t length;
    static uint8_t data[257]; //255 + 2 Byte checksum

    switch(state)
    {
        case WAITING_FOR_START:
            if(b==0xAA)
            {
                uart_puts("Got package start\r\n");
                state = WAIT_ID;
                multibytecounter=0;
            }
        break;
        case WAIT_ID:
            if(multibytecounter==0)
            {
                package_id=b<<8;
                multibytecounter++;
            }
            else
            {
                package_id|=b;
                sprintf(t,"Received package ID: %u\r\n",package_id);
                uart_puts(t);
                if(package_id == 0xFFFF)
                {
                    if(id == 0xFFFF)
                    {
                        state = WAIT_NEW_ID;
                        multibytecounter=0;
                    }
                    else
                    {
                        state=IGNORE_BYTES;
                        ignorecounter=5;
                        uart_puts("Ignoring the next 5 bytes\r\n");
                    }
                }
                else
                {
                    state=WAIT_MSG_INFO;
                    multibytecounter=0;
                    msg_id=0;

                }
            }
        break;

        case WAIT_MSG_INFO:
            switch(multibytecounter)
            {
            case 0: msg_id=b<<8; break;
            case 1: msg_id|=b; break;
            case 2: type=b; break;
            case 3: length=b; break;
            }

            multibytecounter++;
            if(multibytecounter>3)
            {
                if(package_id!=id) //package not for us
                {
                    uart_puts("Ignoring the next bytes because\r\n");
                    ignorecounter=length;
                    state= IGNORE_BYTES;
                }
                else //package for us
                {
                    if(type==1) //master->slave transmit
                    {
                        uart_puts("Receiving data\r\n");
                        multibytecounter=0;
                        state = WAIT_DATA;
                    }
                    else
                    {
                        uart_puts("have to send\r\n");
                        answer_message(msg_id);
                        state=WAITING_FOR_START;
                    }
                }
            }

        break;

        case WAIT_DATA:
            data[multibytecounter]=b;


            multibytecounter++;
            if(multibytecounter>=length+2)
            {
                process_message(msg_id,length,data);

                rs485_tx_autoreset(2);
                RS485_putc(0xAA);
                RS485_putc(data[length+0]);
                RS485_putc(data[length+1]);

                state= WAITING_FOR_START;
            }


        break;

        case WAIT_NEW_ID:
            if(multibytecounter==0)
            {
                new_id=b<<8;
                multibytecounter++;
            }
            else
            {
                new_id|=b;
                process_new_id(new_id);
                state=WAITING_FOR_START;
            }

        break;

        case IGNORE_BYTES:
            if(ignorecounter==0)
                state=WAITING_FOR_START;

            ignorecounter--;

        break;

    }
}

static void process_new_id(uint16_t new_id)
{
    id = new_id;

    eeprom_write_word(&stored_id, id);
    char t[50];
    sprintf(t,"Setting new ID: %u\r\n",id);
    uart_puts(t);

    uint8_t buffer[11];
    buffer[0] = 0xAA;
    buffer[1] = 0xFF;
    buffer[2] = 0xFF;
    buffer[3] = new_id>>8;
    buffer[4] = new_id;
    buffer[5] = new_id>>8;
    buffer[6] = new_id;
    buffer[7] = HW_TYPE>>8;
    buffer[8] = HW_TYPE;
    uint16_t checksum=calc_checksum(buffer,9);
    buffer[9] = checksum>>8;
    buffer[10] = checksum;

    rs485_tx_autoreset(2);
    for(uint8_t i=5; i < 11; i++)
    {
        RS485_putc(buffer[i]);
    }

}

static uint16_t calc_checksum(uint8_t *data, uint16_t length)
{
    uint16_t sum=0;

    for(uint16_t i=0; i < length; i++)
    {
        sum^= data[i];
    }

    return sum;
}


