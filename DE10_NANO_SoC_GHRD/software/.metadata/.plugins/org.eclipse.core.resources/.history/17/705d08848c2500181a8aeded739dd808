#include <stdbool.h>
#include <hwlib.h>
#include "lib_def.h"
#include "bsp_int.h"
#include "serial_communication.h"
#include "uart0_support.h"

char* current_full_msg = "";
char* incoming_msg = "";


bool serial_communication_init(){
	ALT_STATUS_CODE status = uart0_init();

	if(status == ALT_E_SUCCESS){
		BSP_IntVectSet(194u,   // 194 is for UART0 interupt
						1,	    // prio
						DEF_BIT_00,	    // cpu target list
						UART0_IRS_Handeler  // ISR
						);
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
	//printf("c\n", serial_getc());
}
