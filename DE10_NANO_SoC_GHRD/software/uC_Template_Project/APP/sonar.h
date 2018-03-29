// Created By Brian Ofrim
#ifndef  __SONAR__
#define  __SONAR__
#include <cpu.h>
#include <stdint.h>
#include <stdbool.h>
#include "fpga_to_hps.h"
// Ultrasonic Range Finder
// Type:  Output
// Width: Longword
// GPIO:  0_5
#define SONAR_ADD 0x00000120
#define SONAR_BASE FPGA_TO_HPS_LW_ADDR(SONAR_ADD)

#define DISTANCE_HISTORY_LENGTH 20

// circular buffer implementation from: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
typedef struct circular_buf{

    int head;
    int tail;
    int size;
    uint8_t values[DISTANCE_HISTORY_LENGTH];
} circular_buf;

void InitDistanceSensorInterrupt (void);
void DistanceSensor_ISR_Handler(CPU_INT32U cpu_id);
int circular_buf_reset(circular_buf * cbuf);
int circular_buf_put(circular_buf * cbuf, uint8_t data);
int circular_buf_get(circular_buf * cbuf, uint8_t * data);
bool circular_buf_empty(circular_buf cbuf);
bool circular_buf_full(circular_buf cbuf);
int circular_buffer_get_nth(circular_buf * cbuf, uint8_t * data, int nth);
bool sample_window_validator(uint8_t next,uint8_t current, uint8_t previous);
void circular_buf_init(circular_buf * cbuf);
int sample_window_avg(uint8_t next, uint8_t current, uint8_t previous);

#define OSCL1_TICKS_PER_50MS 1250000
#define SONAR_INTERUPT_PERIOD 0.05
#define DISTANCE_FILTER_MAX 4

#define MIN_DETECTABLE_CHANGE 2

#define MINIMUM_DETECTABLE_DISTANCE 6
#define MAXIMUM_DETECTABLE_DISTANCE 254


#endif
