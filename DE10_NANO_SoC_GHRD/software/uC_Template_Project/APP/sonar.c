// Created by Brain Ofrim
#include <lib_def.h>
#include <math.h>
#include <stdarg.h>

#include <ucos_ii.h>
#include "globals.h"
#include "socal.h"
#include <bsp_int.h>

#include "sonar.h"
#include "timer.h"

// From Nancy's EClass code
void InitDistanceSensorInterrupt(void) {

	ARM_OSCL_TIMER_1_REG_CONTROL &= ARM_OSCL_TIMER_1_DISABLE;

	ARM_OSCL_TIMER_1_REG_CONTROL |= ARM_OSCL_TIMER_1_USER_MODE;

	ARM_OSCL_TIMER_1_REG_CONTROL &= ARM_OSCL_TIMER_1_INT_UNMASKED;

	ARM_OSCL_TIMER_1_REG_LOADCOUNT = OSCL1_TICKS_PER_50MS;

	BSP_IntVectSet   (202u,
	                  14,	    // prio
					  DEF_BIT_00,	    // cpu target list
					  DistanceSensor_ISR_Handler  // ISR
					 );

	ARM_OSCL_TIMER_1_REG_CONTROL |= ARM_OSCL_TIMER_1_ENABLE;

	BSP_IntSrcEn(202u);
}

void DistanceSensor_ISR_Handler(CPU_INT32U cpu_id) {

	INT8U err;
	uint8_t dist = alt_read_word(SONAR_BASE);

	// if no object is detected then set distance to maximum
	if(dist == 0){
		dist = MAXIMUM_DETECTABLE_DISTANCE;
	}

	// sonar measurements cannot be lower than MINIMUM_DETECTABLE_DISTANCE
	if(dist < MINIMUM_DETECTABLE_DISTANCE){
		dist = MINIMUM_DETECTABLE_DISTANCE;
	}
	dist_circular_buf_put(distance_buffer,dist);

	OSSemPost(SonarDataAvailableSemaphore);
	ARM_OSCL_TIMER_1_REG_EOI;
}

bool dist_sample_window_validator(uint8_t next, uint8_t current, uint8_t previous){
	if(abs(current - next) <= DISTANCE_FILTER_MAX && abs(current - previous) <= DISTANCE_FILTER_MAX ){
		return true;
	}
	else{
		return false;
	}
}

int sample_window_avg(uint8_t next, uint8_t current, uint8_t previous){
	return ((next + current + previous) / 3);
}


// modified from https://www.tutorialspoint.com/cprogramming/c_variable_arguments.htm
int weighted_avg(int num,...){
	va_list valist;
	va_start(valist, num);
	int sum = 0;
	int divisor = (num*(num + 1))/2;
   /* initialize valist for num number of arguments */

   for (int i = 0; i < num; i++) {
	   sum += va_arg(valist, int) * (num - i);
   }

   va_end ( valist );
   return sum/divisor;

}

// distance required to stop at a given v
int distance_to_stop(int velocity){
	if(velocity <= 0)
		return 0;

	float dist_to_stop = pow(velocity,2) / (2* GRAVITY_CONSTANT *FICTION_COEFFICIENT);
	return (int) ceil(dist_to_stop);

}


