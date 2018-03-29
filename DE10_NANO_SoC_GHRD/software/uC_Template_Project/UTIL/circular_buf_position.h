/*
 * circular_buf_uint8_t.h
 *
 *  Created on: Mar 29, 2018
 *      Author: bofrim
 */

#ifndef _CIRCULAR_BUF_POSITION_H_
#define _CIRCULAR_BUF_POSITION_H_

#define DISTANCE_HISTORY_LENGTH 20
#include <stdint.h>
#include <stdbool.h>

// circular buffer implementation from: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
typedef struct circular_buf_uint8_t{

    int head;
    int tail;
    int size;
    uint8_t values[DISTANCE_HISTORY_LENGTH];
} circular_buf_uint8_t;

int circular_buf_reset(circular_buf_uint8_t * cbuf);
int circular_buf_put(circular_buf_uint8_t * cbuf, uint8_t data);
int circular_buf_get(circular_buf_uint8_t * cbuf, uint8_t * data);
bool circular_buf_empty(circular_buf_uint8_t cbuf);
bool circular_buf_full(circular_buf_uint8_t cbuf);
int circular_buffer_get_nth(circular_buf_uint8_t * cbuf, uint8_t * data, int nth);
bool sample_window_validator(uint8_t next,uint8_t current, uint8_t previous);
void circular_buf_init(circular_buf_uint8_t * cbuf);
void circular_buf_init(circular_buf_uint8_t * cbuf);
#endif /* APP_CIRCULAR_BUF_UINT8_T_H_ */
