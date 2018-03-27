// Created by Brain Ofrim
#include "sonar.h"
#include "timer.h"
#include <lib_def.h>
#include <stdbool.h>
#include <math.h>
#include "ucos_ii.h"
#include "globals.h"
#include "socal.h"


void InitDistanceSensorInterrupt(void) {
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

	ARM_OSCL_TIMER_1_REG_CONTROL &= ARM_OSCL_TIMER_1_DISABLE;

	ARM_OSCL_TIMER_1_REG_CONTROL |= ARM_OSCL_TIMER_1_USER_MODE;

	ARM_OSCL_TIMER_1_REG_CONTROL &= ARM_OSCL_TIMER_1_INT_UNMASKED;

	ARM_OSCL_TIMER_1_REG_LOADCOUNT = OSCL1_TICKS_PER_49MS;

	BSP_IntVectSet   (202u,   // 201 is source for oscl_timer 0 TODO: Find out what this is for timer 1
	                  14,	    // prio
					  DEF_BIT_00,	    // cpu target list
					  DistanceSensor_ISR_Handler  // ISR
					 );

	ARM_OSCL_TIMER_1_REG_CONTROL |= ARM_OSCL_TIMER_1_ENABLE;

	BSP_IntSrcEn(202u); //TODO: change this for timer 1.
}

void DistanceSensor_ISR_Handler(CPU_INT32U cpu_id) {

	int dist = alt_read_word(SONAR_BASE);

	circular_buf_put(distance_buffer, alt_read_word(SONAR_BASE));

	ARM_OSCL_TIMER_1_REG_EOI;
}


/*
Circular buffer code from:
https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
*/

int circular_buf_reset(circular_distance_buf * cbuf)
{
    int r = -1;

    if(cbuf)
    {
        cbuf->head = 0;
        cbuf->tail = 0;
        r = 0;
    }

    return r;
}
int circular_buf_put(circular_distance_buf * cbuf, uint8_t data)
{
    int r = -1;

    if(cbuf)
    {
        cbuf->distances[cbuf->head] = data;
        cbuf->head = (cbuf->head + 1) % cbuf->size;

        if(cbuf->head == cbuf->tail)
        {
            cbuf->tail = (cbuf->tail + 1) % cbuf->size;
        }

        r = 0;
    }

    return r;
}

int circular_buf_get(circular_distance_buf * cbuf, uint8_t * data)
{
    int r = -1;

    if(cbuf && data && !circular_buf_empty(*cbuf))
    {
        *data = cbuf->distances[cbuf->tail];
        cbuf->tail = (cbuf->tail + 1) % cbuf->size;

        r = 0;
    }

    return r;
}

//int circular_buffer_get_nth(circular_distance_buf * cbuf, uint8_t * data, int nth){
//    int r = -1;
//    nth = nth % DISTANCE_HISTORY_LENGTH;
//
//    if(cbuf && data && !circular_buf_empty(*cbuf))
//    {
//
//    	int currHead = cbuf->head;
//    	int currTail = cbuf->tail;
//
//
//
//    	if(abs((cbuf->head - cbuf->tail)) > nth){
//    		*data = -1;
//    	}else{
//    		if(cbuf->head >= cbuf->tail){
//    	    	*data = cbuf->distances[cbuf->tail];
//
//    	        *data = cbuf->distances[cbuf->tail];
//    	        cbuf->tail = (cbuf->tail + 1) % cbuf->size;
//    		}else{
//
//    		}
//    	}
//
//
//        r = 0;
//    }
//
//    return r;
//}


bool circular_buf_empty(circular_distance_buf cbuf)
{
    // We define empty as head == tail
    return (cbuf.head == cbuf.tail);
}

bool circular_buf_full(circular_distance_buf cbuf)
{
    // We determine "full" case by head being one position behind the tail
    // Note that this means we are wasting one space in the buffer!
    // Instead, you could have an "empty" flag and determine buffer full that way
    return ((cbuf.head + 1) % cbuf.size) == cbuf.tail;
}

