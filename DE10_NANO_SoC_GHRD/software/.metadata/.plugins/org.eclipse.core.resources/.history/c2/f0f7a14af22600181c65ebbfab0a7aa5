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

#define MSG_BUFFER_LEN 127
bool serial_communication_init(void);
char serial_getc(void);
void UART0_IRS_Handeler(CPU_INT32U cpu_id);

// from http://pubs.opengroup.org/onlinepubs/009696799/functions/bzero.html
#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)
#endif
