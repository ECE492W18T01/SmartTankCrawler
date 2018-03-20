#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <hwlib.h>
#include "alt_uart.h"
#include "lib_def.h"
#include "bsp_int.h"
#include "serial_communication.h"
#include "uart0_support.h"
#include <watchdog.h>

char current_buffer[MSG_BUFFER_LEN] = {};
char incoming_buffer[MSG_BUFFER_LEN] = {};
bool communications_established = false;




bool serial_communication_init(){
	communications_established = false;
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
						UART0_IRS_handeler  // ISR
						);
		BSP_IntSrcEn(194u);
		init_communication_watchdog();
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

incoming_msg* parse_incomming_msg(char * msg){
	    char *motor_level_str;
	    char *steering_value_str;

	    incoming_msg *new_msg;

	    motor_level_str = strtok(msg,",");
	    steering_value_str = strtok(NULL,",");

	    if( motor_level_str==NULL || steering_value_str == NULL )
	    {
	    	// invalid message
	    	new_msg = NULL;
	    }else{
	    	new_msg->motor_level = (float) atof(motor_level_str);
	    	new_msg->steering_value = (int8_t) atoi(steering_value_str);
	    }

	    return new_msg;
}

// TODO: refactor into task,
void UART0_IRS_handeler(CPU_INT32U cpu_id){
	char incomingChar[1] = {};
	incomingChar[0] = serial_getc();

	if(incomingChar[0] ==  START_CHARACTER){
		// clear buffer
		bzero(current_buffer ,MSG_BUFFER_LEN);
		communications_established = true;
		serial_printf(ACKNOWLEDGE_STR);
		printf("Communications establisbed...\n");
	}else if(incomingChar[0] == STOP_CHARACTER){
		// clear buffer
		bzero(current_buffer ,MSG_BUFFER_LEN);
		uart0_fifo_clear_rx();
		communications_established = false;
		printf("Communications terminated...\n");
		// send ack message
		serial_printf(ACKNOWLEDGE_STR);
	}else if(communications_established == true && incomingChar[0] == '\r'){
		// clear buffer
		bzero(current_buffer ,MSG_BUFFER_LEN);
		strncpy(current_buffer,incoming_buffer,MSG_BUFFER_LEN);
		printf("%s\n",current_buffer);
		incoming_msg *new_msg = parse_incomming_msg(current_buffer);
		if(new_msg != NULL){
			// TODO: JOSH send to keith
			serial_printf(ACKNOWLEDGE_STR);
		}

		bzero(incoming_buffer,MSG_BUFFER_LEN);
	}else if(communications_established){
		strncat(incoming_buffer,incomingChar,1);
	}else{
		// error state clear fifo
		uart0_fifo_clear_rx();
	}
}


