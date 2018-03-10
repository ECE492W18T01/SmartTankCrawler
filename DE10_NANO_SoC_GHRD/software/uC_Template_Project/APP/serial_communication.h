#ifndef __SERIAL_COMMUNICATION_H__
#define __SERIAL_COMMUNICATION_H__
#include <stdbool.h>
#define UART_0_BASE_ADDRESS 0xFFC02000

bool serial_communication_init();
char serial_getc();
void UART0_IRS_Handeler(CPU_INT32U cpu_id);
#endif
