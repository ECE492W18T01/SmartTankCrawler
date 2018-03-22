#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <hwlib.h>
#include "alt_uart.h"
#include "lib_def.h"
#include "bsp_int.h"
#include "serial_communication.h"
#include "uart0_support.h"
#include <watchdog.h>
#include "globals.h"
#include <os_cpu.h>

bool serial_communication_init(){

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
		//init_communication_watchdog();
		return true;
	}else{
		return false;
	}

}

int rx_fifo_level(){
	return UART_0_RX_FIFO_LEVEL & UART_0_RX_FIFO_LEVEL_MASK;;
}

char serial_getc(){
	return uart0_getc();
}

int serial_send(char * print_str){
	return uart0_printf("%s", print_str);
}

incoming_msg* parse_incomming_msg(char * msg){
	    char *motor_level_str;
	    char *steering_value_str;

	    incoming_msg *new_msg;

	    motor_level_str = strtok(msg,DELIMINATING_STR);
	    steering_value_str = strtok(NULL,DELIMINATING_STR);

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


bool read_rx_buffer(char* incoming_data_buffer, uint32_t *characters_read){
	ALT_STATUS_CODE status= uart0_gets(incoming_data_buffer, characters_read,  MSG_BUFFER_LEN);
	if(status == ALT_E_SUCCESS)
		return true;
	else
		return false;
}

bool look_for_end_byte(char * incoming_message){
	const char *ptr = strchr(incoming_message, STOP_CHARACTER);
	if(ptr)
		return true;
	else
		return false;
}

bool look_for_start_byte(char * incoming_message, int incomming_message_size){

	const char *ptr = strchr(incoming_message, START_CHARACTER);
	if(ptr) {
	   int index = ptr - incoming_message;
	   int chars_to_copy = incomming_message_size - incomming_message_size;
	   char subbuff[chars_to_copy + 1];
	   memcpy( subbuff, &incoming_message[index],  chars_to_copy);
	   bzero(incoming_message,MSG_BUFFER_LEN);
	   subbuff[chars_to_copy] = '\0';
	   memcpy(incoming_message, subbuff, chars_to_copy);
	   return true;
	}
	return false;
}

bool complete_message_revived(char * incoming_message){
	const char *ptr = strchr(incoming_message, '\r');
	if(ptr)
		return true;
	else
		return false;
}


void UART0_IRS_handeler(CPU_INT32U cpu_id){
	INT8U err = OSSemPost(RxDataAvailabeSemaphore);
}


