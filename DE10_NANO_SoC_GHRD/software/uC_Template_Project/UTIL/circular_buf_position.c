// circular buffer implementation from: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
// Adapted by Brian Ofrim
#include "circular_buf_position.h"
#include <stdbool.h>

int dist_circular_buf_reset(circular_buf_position * cbuf)
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

void dist_circular_buf_init(circular_buf_position * cbuf){
	cbuf->head = 0;
	cbuf->tail = 0;
	cbuf->size = DISTANCE_HISTORY_LENGTH;
    for (int i = 0; i < DISTANCE_HISTORY_LENGTH; i++) {
    	cbuf->values[i] = 0;
    }
}


int dist_circular_buf_put(circular_buf_position * cbuf, uint8_t data)
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


int dist_circular_buf_get(circular_buf_position * cbuf, uint8_t * data)
{
    int r = -1;

    if(cbuf && data && !dist_circular_buf_empty(*cbuf))
    {
        *data = cbuf->values[cbuf->tail];
        cbuf->tail = (cbuf->tail + 1) % cbuf->size;

        r = 0;
    }

    return r;
}

// made by Brian Ofrim
int dist_circular_buffer_get_nth(circular_buf_position * cbuf, uint8_t * data, int nth){
    int r = -1;
    nth = nth % (DISTANCE_HISTORY_LENGTH-1);
    if(cbuf && data && !dist_circular_buf_empty(*cbuf))
    {

    	int headNthDelta = cbuf->head - 1 - nth;

    	if(headNthDelta<0){
    		headNthDelta += DISTANCE_HISTORY_LENGTH; // wrap around
    	}

    	*data = cbuf->values[headNthDelta];
        r = 0;
    }

    return r;
}


bool dist_circular_buf_empty(circular_buf_position cbuf)
{
    // We define empty as head == tail
    return (cbuf.head == cbuf.tail);
}

bool dist_circular_buf_full(circular_buf_position cbuf)
{
    // We determine "full" case by head being one position behind the tail
    // Note that this means we are wasting one space in the buffer!
    // Instead, you could have an "empty" flag and determine buffer full that way
    return ((cbuf.head + 1) % cbuf.size) == cbuf.tail;
}
