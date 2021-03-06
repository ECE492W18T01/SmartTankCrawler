// Created by Brain Ofrim
#ifndef __SERIAL_COMMUNICATION_H__
#define __SERIAL_COMMUNICATION_H__
#include <stdbool.h>
#define UART_0_BASE_ADDRESS 0xFFC02000
#define UART_0_IER_DLH_REG (*(( CPU_REG32 *) (UART_0_BASE_ADDRESS + 0x4)))
#define UART_0_RX_DATA_AVALABLE_INT_ENABLE  DEF_BIT_00 // or with this
#define UART_0_RX_DATA_AVALABLE_INT_DISABLE  ~DEF_BIT_00 // and with this
#define UART_0_THER_INT_ENABLE  DEF_BIT_07 // or with this
#define UART_0_THER_INT_DISABLE  ~DEF_BIT_07 // and with this
#define UART_0_FCR_REG (*(( CPU_REG32 *) (UART_0_BASE_ADDRESS + 0x8)))
#define UART_0_FCR_RT_ONE_CHAR (~DEF_BIT_07 & ~DEF_BIT_06) // and with this

#define UART_0_RX_FIFO_LEVEL (*(( CPU_REG32 *) (UART_0_BASE_ADDRESS + 0x84)))
#define UART_0_RX_FIFO_LEVEL_MASK (0b11111)

#define MSG_BUFFER_LEN 255

#define START_CHARACTER '*'
#define STOP_CHARACTER '&'
#define MESSAGE_START_STR "!"
#define MESSAGE_END_CHAR '\r'
#define MESSAGE_END_STR "\r"
#define ACKNOWLEDGE_STR "@"
#define DELIMINATING_STR ","


#define MOTOR_ZERO (float)0.0
#define STEERING_ZERO (int8_t)0

typedef struct incoming_msg{
	float motor_level;
	int8_t steering_value;
}incoming_msg;

bool serial_communication_init(void);
int rx_fifo_level(void);
char serial_getc(void);
int serial_send(char * print_str);
incoming_msg parse_incomming_msg(char * msg);
bool read_rx_buffer(char* incoming_data_buffer, uint32_t *characters_read);
bool look_for_end_byte(char * incoming_message);
bool look_for_start_byte(char * incoming_message, int incomming_message_size);
bool complete_message_revived(char * incoming_message);
void UART0_IRS_handeler(CPU_INT32U cpu_id);

// from http://pubs.opengroup.org/onlinepubs/009696799/functions/bzero.html
#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)



#endif
