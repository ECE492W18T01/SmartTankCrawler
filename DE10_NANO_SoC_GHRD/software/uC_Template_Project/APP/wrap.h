/*
 * ECE 492 Wi18 Team 1
 * wrap.c, wrap.h
 * March 5th, 2018
 * Fredric Mendi and Keith Mills
 * with data structures from Joshua Robertson
 * Purpose of this C/H file pair:
 * 1.Abstract I/O driver memory locations to functions.
 *
 * 2. Compute and store base addresses for GPIO components.
 * Provide proper functions for sending/receiving data to
 * FPGA
 *
 * 3. Provide the upper and lower bounds of the acceptable
 * data inputs to VHDL components, and using wrapper functions
 * allow another programmer to give these functions any value
 * but filter said input so that it does not cause weird behaviour.
 *
 * 4. Provide data structures for the information passing between tasks
 */

#ifndef wrap_H
#define wrap_H

#include  <app_cfg.h>
#include  <lib_mem.h>

#include  <bsp.h>
#include  <bsp_int.h>
#include  <bsp_os.h>
#include  <cpu_cache.h>

#include  <cpu.h>
#include  <cpu_core.h>

#include  <os.h>
#include  <hps.h>
#include  <socal.h>
#include  <hwlib.h>

#include <stdio.h>
#include <stdlib.h>

// Full Clockwise.
#define BackServoMin 0x00

// Full Counter Clockwise - Go above this and it moves no further but rattles.
#define BackServoMax 0x2E

// Full Clockwise, steering rod 60 degrees right turn.
#define FrontServoMin 0x00

// Steering rod 60 degrees left turn.
#define FrontServoMax 0x10

// Full reverse.
#define DCMotorMin -127

// Full forward.
#define DCMotorMax 127

// Numerical labels for motors
#define FRONTLEFT 0
#define FRONTRIGHT 1
#define REARLEFT 2
#define REARRIGHT 3

// Compute absolute address of any slave component attached to lightweight bridge
// base is address of component in QSYS window
// This computation only works for slave components attached to the lightweight bridge
// base should be ranged checked from 0x0 - 0x1fffff

#define FPGA_TO_HPS_LW_ADDR(base)  ((void *) (((char *)  (ALT_LWFPGASLVS_ADDR))+ (base)))

/*
*********************************************************************************************************
*										Qsys Components
*********************************************************************************************************
*/
// Green FPGA Leds
// Type:  Input
// Width: Byte
// GPIO:  N/A
#define LEDR_ADD 0x00000100
#define LEDR_BASE FPGA_TO_HPS_LW_ADDR(LEDR_ADD)

// Servo #1, Steering Control
// Type:  Input
// Width: Byte
// GPIO:  0_0
#define STEER_SERVO_ADD 0x00000110
#define STEER_SERVO_BASE FPGA_TO_HPS_LW_ADDR(STEER_SERVO_ADD)

// Servo #2, Emergency Braking System
// Type:  Input
// Width: Byte
// GPIO:  0_1
#define BRAKE_SERVO_ADD 0x00000111
#define BRAKE_SERVO_BASE FPGA_TO_HPS_LW_ADDR(BRAKE_SERVO_ADD)

// Drive Motor #1, Front Left
// Type:  Input
// Width: Byte
// GPIO:  dir_1 - 0_10; dir_2 - 0_11 mag - 0_12
#define FRONT_LEFT_MOTOR_ADD 0x00000112
#define FRONT_LEFT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(FRONT_LEFT_MOTOR_ADD)

// Drive Motor #2, Front Right
// Type:  Input
// Width: Byte
// GPIO:  dir_1 - 0_13; dir_2 - 0_14 mag - 0_15
#define FRONT_RIGHT_MOTOR_ADD 0x00000113
#define FRONT_RIGHT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(FRONT_RIGHT_MOTOR_ADD)

// Drive Motor #3, Rear Left
// Type:  Input
// Width: Byte
// GPIO:  dir_1 - 0_16; dir_2 - 0_17 mag - 0_18
#define REAR_LEFT_MOTOR_ADD 0x00000114
#define REAR_LEFT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(REAR_LEFT_MOTOR_ADD)

// Drive Motor #4, Rear Right
// Type:  Input
// Width: Byte
// GPIO:  dir_1 - 0_19; dir_2 - 0_20 mag - 0_21
#define REAR_RIGHT_MOTOR_ADD 0x00000115
#define REAR_RIGHT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(REAR_RIGHT_MOTOR_ADD)


/* Hall Sensor GPIO
* FL - 0_4
* FR - 0_5
* RL - 0_6
* RR - 0_7
*/
// Front Left Byte
#define F_LEFT_ADD 0x00000116
#define F_LEFT_BASE FPGA_TO_HPS_LW_ADDR(F_LEFT_ADD)

// Front Right Byte
#define F_RIGHT_ADD 0x00000117
#define F_RIGHT_BASE FPGA_TO_HPS_LW_ADDR(F_RIGHT_ADD)

// Rear Left Byte
#define R_LEFT_ADD 0x00000118
#define R_LEFT_BASE FPGA_TO_HPS_LW_ADDR(R_LEFT_ADD)

// Rear Right Byte
#define R_RIGHT_ADD 0x00000119
#define R_RIGHT_BASE FPGA_TO_HPS_LW_ADDR(R_RIGHT_ADD)

// Ultrasonic Range Finder
// Type:  Output
// Width: Longword
// GPIO:  0_8
#define SONAR_ADD 0x00000120
#define SONAR_BASE FPGA_TO_HPS_LW_ADDR(SONAR_ADD)

typedef struct {
	char *_taskName;
	char *_sourceName;
	INT8U _error;
} ErrorMessage;

typedef struct {
	uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t backLeft;
    uint8_t backRight;
} MotorSpeedMessage;

typedef struct {
	float frontLeft;
    float frontRight;
    float backLeft;
    float backRight;
    uint8_t steeringServo;
} MotorChangeMessage;

extern OS_EVENT *FuzzyQueue;

// Function for controlling rear, emergency brake servo.
void MoveBackServo(uint8_t hex);

// Function for controlling front, steering servo.
void MoveFrontServo(uint8_t hex);

// Function for giving instructions to a wheel.
void MoveDCMotor(uint8_t num, int wheel);

#endif
