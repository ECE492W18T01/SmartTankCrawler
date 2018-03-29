// Created by Brain Ofrim
#include <lib_def.h>
#include <stdbool.h>
#include <math.h>

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
	circular_buf_put(distance_buffer,dist);


	OSSemPost(SonarDataAvailableSemaphore);
	ARM_OSCL_TIMER_1_REG_EOI;
}


/*
Circular buffer code from:
https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
*/

int circular_buf_reset(circular_buf_uint8_t * cbuf)
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

void circular_buf_init(circular_buf_uint8_t * cbuf){
	cbuf->head = 0;
	cbuf->tail = 0;
	cbuf->size = DISTANCE_HISTORY_LENGTH;
    for (int i = 0; i < DISTANCE_HISTORY_LENGTH; i++) {
    	cbuf->values[i] = 0;
    }
}

bool sample_window_validator(uint8_t next, uint8_t current, uint8_t previous){
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

int circular_buf_put(circular_buf_uint8_t * cbuf, uint8_t data)
{
    int r = -1;

    if(cbuf)
    {
        cbuf->values[cbuf->head] = data;
        cbuf->head = (cbuf->head + 1) % cbuf->size;

        if(cbuf->head == cbuf->tail)
        {
            cbuf->tail = (cbuf->tail + 1) % cbuf->size;
        }

        r = 0;
    }

    return r;
}


int circular_buf_get(circular_buf_uint8_t * cbuf, uint8_t * data)
{
    int r = -1;

    if(cbuf && data && !circular_buf_empty(*cbuf))
    {
        *data = cbuf->values[cbuf->tail];
        cbuf->tail = (cbuf->tail + 1) % cbuf->size;

        r = 0;
    }

    return r;
}

// made by Brian Ofrim
int circular_buffer_get_nth(circular_buf_uint8_t * cbuf, uint8_t * data, int nth){
    int r = -1;
    nth = nth % (DISTANCE_HISTORY_LENGTH-1);
    if(cbuf && data && !circular_buf_empty(*cbuf))
    {

    	int headNthDelta = cbuf->head - nth;

    	if(headNthDelta<0){
    		headNthDelta += DISTANCE_HISTORY_LENGTH; // wrap around
    	}

    	*data = cbuf->values[headNthDelta];
        r = 0;
    }

    return r;
}


bool circular_buf_empty(circular_buf_uint8_t cbuf)
{
    // We define empty as head == tail
    return (cbuf.head == cbuf.tail);
}

bool circular_buf_full(circular_buf_uint8_t cbuf)
{
    // We determine "full" case by head being one position behind the tail
    // Note that this means we are wasting one space in the buffer!
    // Instead, you could have an "empty" flag and determine buffer full that way
    return ((cbuf.head + 1) % cbuf.size) == cbuf.tail;
}

