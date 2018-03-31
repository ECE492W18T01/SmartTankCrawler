// circular buffer implementation from: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
// Adapted by Brian Ofrim
#include "circular_buf_velocity.h"
#include <stdbool.h>

int vel_circular_buf_reset(circular_buf_velocity * cbuf)
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

void vel_circular_buf_init(circular_buf_velocity * cbuf){
	cbuf->head = 0;
	cbuf->tail = 0;
	cbuf->size = VELOCITY_HISTORY_LENGTH;
    for (int i = 0; i < VELOCITY_HISTORY_LENGTH; i++) {
    	cbuf->values[i] = 0;
    }
}


int vel_circular_buf_put(circular_buf_velocity * cbuf, int8_t data)
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


int vel_circular_buf_get(circular_buf_velocity * cbuf, int8_t * data)
{
    int r = -1;

    if(cbuf && data && !vel_circular_buf_empty(*cbuf))
    {
        *data = cbuf->values[cbuf->tail];
        cbuf->tail = (cbuf->tail + 1) % cbuf->size;

        r = 0;
    }

    return r;
}

// made by Brian Ofrim
int vel_circular_buffer_get_nth(circular_buf_velocity * cbuf, int8_t * data, int nth){
    int r = -1;
    nth = nth % (VELOCITY_HISTORY_LENGTH-1);
    if(cbuf && data && !vel_circular_buf_empty(*cbuf))
    {

    	int headNthDelta = cbuf->head - nth;

    	if(headNthDelta<0){
    		headNthDelta += VELOCITY_HISTORY_LENGTH; // wrap around
    	}

    	*data = cbuf->values[headNthDelta];
        r = 0;
    }

    return r;
}


bool vel_circular_buf_empty(circular_buf_velocity cbuf)
{
    // We define empty as head == tail
    return (cbuf.head == cbuf.tail);
}

bool vel_circular_buf_full(circular_buf_velocity cbuf)
{
    // We determine "full" case by head being one position behind the tail
    // Note that this means we are wasting one space in the buffer!
    // Instead, you could have an "empty" flag and determine buffer full that way
    return ((cbuf.head + 1) % cbuf.size) == cbuf.tail;
}
