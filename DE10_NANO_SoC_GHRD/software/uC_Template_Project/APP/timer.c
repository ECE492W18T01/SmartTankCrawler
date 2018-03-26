/*
 * timer.c
 *
 *  Created on: Nov 1, 2017
 *      Author: nemtech
 */

#include "socal/hps.h"
#include "socal/socal.h"
#include "lib_def.h"
#include "os_cpu.h"
#include "os.h"
#include "timer.h"
#include "fpga_to_hps.h"
#include "wrap.h"

void InitHallSensorInterrupt(void) {
	// This version will use the oscl_clk based timers on the HPS core
	// Process
	// Choose one of the four timers (oscl timer 0, oscl timer 1, sptimer 0, sptimer 1)
	// Set to use user-defined count mode
	// Choose timer value should be equal to osc1_clk (50000000 HZ) == 2FAF080 actuall is 100000000
	// PSEUDOCODE:
	//  - Disable the timer
	//  - Program the Timer Mode: User Defined
	//  - Initialize the vector table: source 201 for oscl_0_timer
	//  - Set the interrupt mask to unmasked
	//  - Load the counter value
	//  - Enable the timer

	ARM_OSCL_TIMER_0_REG_CONTROL &= ARM_OSCL_TIMER_0_DISABLE;

	ARM_OSCL_TIMER_0_REG_CONTROL |= ARM_OSCL_TIMER_0_USER_MODE;

	ARM_OSCL_TIMER_0_REG_CONTROL &= ARM_OSCL_TIMER_0_INT_UNMASKED;

	//TODO: Externalize this variable to a "settings" header. I think the comment below is wrong too
	// 250000000 of the oscl_clk should be one second --> I think it's actually 10 seconds
	//12500000 should be 2Hz
	//125000000 should be every five seconds, or 0.2 Hz

	ARM_OSCL_TIMER_0_REG_LOADCOUNT = 12500000; // 250000000 of the oscl_clk should be one second


	BSP_IntVectSet   (201u,   // 201 is source for oscl_timer 0
	                  2,	    // prio
					  DEF_BIT_00,	    // cpu target list
					  HallSensor_ISR_Handler  // ISR
					 );

	ARM_OSCL_TIMER_0_REG_CONTROL |= ARM_OSCL_TIMER_0_ENABLE;

	BSP_IntSrcEn(201u);
}


void HallSensor_ISR_Handler(CPU_INT32U cpu_id) {

	INT8U err;
	HallSensorMessage *outgoing = OSMemGet(StandardMemoryStorage, &err);
	if (err == OS_ERR_NONE) {
		// Read Hall Sensor Raw Data
		outgoing->frontLeft = alt_read_byte(F_LEFT_BASE);   // Front Left
		outgoing->frontRight = alt_read_byte(F_RIGHT_BASE); // Front Right
		outgoing->backLeft = alt_read_byte(R_LEFT_BASE);	  // Rear Left
		outgoing->backRight = alt_read_byte(R_RIGHT_BASE);  // Rear Right


		alt_write_byte(LEDR_BASE, alt_read_byte(LEDR_BASE) +1);

		OSQPost(FuzzyQueue, outgoing);
	}


	// READ EOI Reg to clear interrupt (PAGE 23-10/23-11 of Cyclone V Hard Processor System
	// Technical Reference Manual
	ARM_OSCL_TIMER_0_REG_EOI;
}

