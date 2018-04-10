// Created By Brian Ofrim
#ifndef  __SONAR__
#define  __SONAR__
#include <cpu.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "fpga_to_hps.h"
#include <circular_buf_position.h>

// Ultrasonic Range Finder
// Type:  Output
// Width: Longword
// GPIO:  0_5
#define SONAR_ADD 0x00000120
#define SONAR_BASE FPGA_TO_HPS_LW_ADDR(SONAR_ADD)

void InitDistanceSensorInterrupt (void);
void DistanceSensor_ISR_Handler(CPU_INT32U cpu_id);
int distance_to_stop(int velocity);

#define OSCL1_TICKS_PER_50MS 1250000
#define OSCL1_TICKS_PER_100MS 2500000
#define SONAR_INTERUPT_PERIOD 0.05
#define DISTANCE_FILTER_MAX 4

#define MIN_DETECTABLE_CHANGE 2

#define MINIMUM_DETECTABLE_DISTANCE 6
#define MAXIMUM_DETECTABLE_DISTANCE 254

#define FICTION_COEFFICIENT 1 // parameter to very when testing
#define GRAVITY_CONSTANT 9.81

#define INT8_T_ABS_MAX 127

#define N_VELOCITES_TO_AVG 3

#define LOWER_DISTANCE_THRESHOLD 18 // under this distance use LOWER_TALLY_THRESHOLD
#define LOWER_TALLY_THRESHOLD 3
#define HIGHER_TALLY_THRESHOLD 5

#define STOPPING_SAFETY_FACTOR 1.25 // parameter to very when testing

#define STOPPING_LIMIT 42 // do not stop at distances less than this. parameter to very when testing

#define SIGN_OF(val) ((val < 0 ? -1 : 1))

#define LIMIT_VALUE_SIGNED(val, max) ((abs(val) <= abs(max)) ? val: (abs(max) * SIGN_OF(val)))

#define DISTANCE_SAMPLES 20
#define SAMPLE_OFFSET 3

#endif
