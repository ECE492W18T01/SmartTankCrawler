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
typedef struct circular_distance_buf{
    uint8_t distances[DISTANCE_HISTORY_LENGTH];
    int head;
    int tail;
    int size;
} circular_distance_buf;

void InitDistanceSensorInterrupt (void);
void DistanceSensor_ISR_Handler(CPU_INT32U cpu_id);
int circular_buf_reset(circular_distance_buf * cbuf);
int circular_buf_put(circular_distance_buf * cbuf, uint8_t data);
int circular_buf_get(circular_distance_buf * cbuf, uint8_t * data);
bool circular_buf_empty(circular_distance_buf cbuf);
bool circular_buf_full(circular_distance_buf cbuf);

#define OSCL1_TICKS_PER_49MS 1225000

#define MINIMUM_DETECTABLE_DISTANCE 6



#endif
