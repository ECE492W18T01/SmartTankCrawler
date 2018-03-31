// circular buffer implementation from: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
// Adapted by Brian Ofrim

#ifndef _CIRCULAR_BUF_POSITION_H_
#define _CIRCULAR_BUF_POSITION_H_

#define DISTANCE_HISTORY_LENGTH 20
#include <stdint.h>
#include <stdbool.h>


typedef struct circular_buf_position{

    int head;
    int tail;
    int size;
    uint8_t values[DISTANCE_HISTORY_LENGTH];
} circular_buf_position;

int dist_circular_buf_reset(circular_buf_position * cbuf);
int dist_circular_buf_put(circular_buf_position * cbuf, uint8_t data);
int dist_circular_buf_get(circular_buf_position * cbuf, uint8_t * data);
bool dist_circular_buf_empty(circular_buf_position cbuf);
bool dist_circular_buf_full(circular_buf_position cbuf);
int dist_circular_buffer_get_nth(circular_buf_position * cbuf, uint8_t * data, int nth);
void dist_circular_buf_init(circular_buf_position * cbuf);
#endif /* APP_CIRCULAR_BUF_UINT8_T_H_ */
