// Created by Brain Ofrim
#include "sonar.h"
#include "timer.h"
#include <lib_def.h>
#include <stdbool.h>
#include <math.h>
#include "ucos_ii.h"
#include "globals.h"
#include "socal.h"

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
	// initialize the circular buffer
	circular_buf_init(distance_buffer, MAXIMUM_DETECTABLE_DISTANCE + 1);
}

void DistanceSensor_ISR_Handler(CPU_INT32U cpu_id) {

	uint8_t dist = alt_read_word(SONAR_BASE);
	circular_buf_put(distance_buffer,dist);
	ARM_OSCL_TIMER_1_REG_EOI;

	OSSemPost(SonarDataAvailableSemaphore);
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

void circular_buf_init(circular_distance_buf * cbuf, uint8_t init_val){
	for(int i = 0; i < cbuf->size; i++){
		cbuf->distances[i] = init_val;
	}
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

// made by Brian Ofrim
int circular_buffer_get_nth(circular_distance_buf * cbuf, uint8_t * data, int nth){
    int r = -1;
    nth = nth % DISTANCE_HISTORY_LENGTH;
    if(cbuf && data && !circular_buf_empty(*cbuf))
    {

    	int headNthDelta = cbuf->head - nth;

    	if(headNthDelta<0){
    		headNthDelta += DISTANCE_HISTORY_LENGTH; // wrap around
    	}

    	*data = cbuf->distances[headNthDelta];
        r = 0;
    }

    return r;
}


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

