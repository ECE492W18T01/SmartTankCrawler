#ifndef fuzzyMotorDrive_H
#define fuzzyMotorDrive_H

/*
 * ECE 492 Wi18 Team 1
 * Keith Mills and Fredric Mendi
 * This H file contains the function header and constants for
 * translating user input and fuzzy set outputs into
 * commands the vehicle will take.
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "motorDriveR.h"
#include "motor_control.h"
#include "wrap.h"

// Indexes for the ideal, slip-free steering modifiers
// See motorDriveR.c/h
// Inner front wheel index
#define opposite       	0
// Wheel behind the fastest wheel
#define behind         	1
// Wheel diagonally across the fastest wheel.
#define across         	2

// Killswitch command. Set all motors to zero.
#define kill           	0

// If we're travelling forward the input to driveMotors, driveSpeed is more than 1
#define forward        	0
// Maximum wheel speed ratio - 1 will translate to the PWM duty cycle of 100%
#define wheelSpeed     	1

// Index for the steering motor.
#define STEERING_MOTOR  4

// 0 is ahead, greater than 0 is a right turn, less than zero is a left turn.
#define straightAngle 	0

/*
 * driveMotors
 *
 */
void driveMotors(float driveSpeed, MotorChangeMessage *fuzzyMods, int8_t steering, bool stopMask);

#endif
