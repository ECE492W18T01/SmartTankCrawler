// circular buffer implementation from: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
// Adapted by Brian Ofrim
#ifndef UTIL_CIRCULAR_BUF_VELOCITY_H_
#define UTIL_CIRCULAR_BUF_VELOCITY_H_

#define VELOCITY_HISTORY_LENGTH 20
#include <stdint.h>
#include <stdbool.h>


typedef struct circular_buf_velocity{

    int head;
    int tail;
    int size;
    int8_t values[VELOCITY_HISTORY_LENGTH];
} circular_buf_velocity;

int vel_circular_buf_reset(circular_buf_velocity * cbuf);
int vel_circular_buf_put(circular_buf_velocity * cbuf, int8_t data);
int vel_circular_buf_get(circular_buf_velocity * cbuf, int8_t * data);
bool vel_circular_buf_empty(circular_buf_velocity cbuf);
bool vel_circular_buf_full(circular_buf_velocity cbuf);
int vel_circular_buffer_get_nth(circular_buf_velocity * cbuf, int8_t * data, int nth);
void vel_circular_buf_init(circular_buf_velocity * cbuf);


#endif /* UTIL_CIRCULAR_BUF_VELOCITY_H_ */
