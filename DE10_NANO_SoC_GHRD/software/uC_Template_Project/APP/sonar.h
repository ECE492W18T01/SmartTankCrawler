// Created By Brian Ofrim
#ifndef  __SONAR__
#define  __SONAR__
#include <cpu.h>
#include <stdint.h>
// Ultrasonic Range Finder
// Type:  Output
// Width: Longword
// GPIO:  0_5
#define SONAR_ADD 0x00000120
#define SONAR_BASE FPGA_TO_HPS_LW_ADDR(SONAR_ADD)

void InitDistanceSensorInterrupt (void);
void DistanceSensor_ISR_Handler(CPU_INT32U cpu_id);

#define OSCL1_TICKS_PER_49MS 1225000

#define MINIMUM_DETECTABLE_DISTANCE 6

// circular buffer implementation from: https://embeddedartistry.com/blog/2017/4/6/circular-buffers-in-cc
typedef struct {
    uint8_t distances[20];
    uint8_t *head;
    uint8_t *tail;
    uint8_t size;
} circular_distance_buf;
#endif
