#include "watchdog.h"
#include <bsp_int.h>
#include <stdio.h>

void init_communication_watchdog(){
	WD_DBG_PAUSE_REG &= WD_DBG_PAUSE_REG_CLEAR;

	// to set initial timeout period
	WDT1_TORR &= WTD_TIMEOUR_INIT_CLEAR;
	WDT1_TORR |=  WTD_TIMEOUT_INIT;

	// set the restart timeout period
	WDT1_TORR &= WTD_TIMEOUT_RESET_CLEAR;
	WDT1_TORR |=  WTD_TIMEOUT_RESET;

	// set the response mode to inturrupt
	WDT1_CR |= WDT_INTERUPT_MODE;


	BSP_IntVectSet(204u,   // 194 is for UART0 interrupt
					1,	    // prio
					DEF_BIT_00,	    // cpu target list
					WTD1_ISR_handeler  // ISR
					);

	BSP_IntSrcEn(204u);
}

void enable_communication_watchdog(){
	// enable the watchdog timer
	WDT1_CR |= WDT_ENABLE;
	reset_communication_watchdog();

}

void disable_communication_watchdog(){
	// disable the watchdog time
	WDT1_CR &= WDT_DISABLE;
}


void reset_communication_watchdog(){
	WDT1_CRR = 0x76;
}

void WTD1_ISR_handeler(CPU_INT32U cpu_id){
	printf("WDT1\n");
	reset_communication_watchdog();
}
