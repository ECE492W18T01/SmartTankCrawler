#include <stdbool.h>
#include <hwlib.h>
#include "serial_communication.h"

char* current_full_msg = "";
char* incoming_msg = "";


bool serial_communication_init(){
	ALT_STATUS_CODE status = uart0_init();
	if(status == ALT_E_SUCCESS)
		return true;
	else
		return false;
}


char serial_getc(){
	return uart0_getc();
}
