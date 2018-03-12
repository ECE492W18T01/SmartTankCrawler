#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <hwlib.h>
#include "alt_uart.h"
#include "lib_def.h"
#include "bsp_int.h"
#include "serial_communication.h"
#include "uart0_support.h"

char current_buffer[MSG_BUFFER_LEN] = {};
char incoming_buffer[MSG_BUFFER_LEN] = {};

bool serial_communication_init(){
	// Initialize message buffers
	bzero(current_buffer, MSG_BUFFER_LEN);
	bzero(incoming_buffer, MSG_BUFFER_LEN);

	ALT_STATUS_CODE status = uart0_init();

	if(status == ALT_E_SUCCESS){
		// set the interrupt to trigger when there is a character in the rx fifo
		UART_0_FCR_REG &= UART_0_FCR_RT_ONE_CHAR;

		// enable RX data available interrupt
		UART_0_IER_DLH_REG |= UART_0_RX_DATA_AVALABLE_INT_ENABLE;

		BSP_IntVectSet(194u,   // 194 is for UART0 interrupt
						1,	    // prio
						DEF_BIT_00,	    // cpu target list
						UART0_IRS_Handeler  // ISR
						);
		BSP_IntSrcEn(194u);
		return true;
	}else{
		return false;
	}

}


char serial_getc(){
	return uart0_getc();
}

void serial_printf(char * print_str){
	uart0_printf("%s", print_str);
}

void UART0_IRS_Handeler(CPU_INT32U cpu_id){
	char incomingChar[1] = {};
	incomingChar[0] = serial_getc();

	if(incomingChar[0] == '\r'){
		bzero(current_buffer ,MSG_BUFFER_LEN);
		strncpy(current_buffer,incoming_buffer,MSG_BUFFER_LEN);
		printf("%s\n",current_buffer);
		bzero(incoming_buffer,MSG_BUFFER_LEN);
	}else{
		strncat(incoming_buffer,incomingChar,1);
	}
}
