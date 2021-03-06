#ifndef FuzzyLogicProcessor_H
#define FuzzyLogicProcessor_H

/*
* ECE 492 Wi18
* March 4th, 2018
* Keith Mills
* Header File for Fuzzy Logic Processing
* The .c file this header refers to contains numerous tests
* In the directory is a given Makefile
* which produces a file fuzzy that runs the tests
*/

// Includes, we use small int types as well as math.
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
// I don't expect this name to change.
#include "fMatrices_servoFuzzyThree.h"
// On the other hand, change this name according to your needs.
#include "sliplessSteeringRatios_45R_17W_30L.h"

// Front and Rear Axle Multiplier, Overall Multiplier, two numbers to adjust the calculation
// Results so that they can be used as indices to access the decision matrices.
#define axleRatio    256
#define overallRatio 512
#define adjustForFS  10
#define shiftIndex   10

// This function takes the calculation of one of the two below functions
// And "cleans it, somewhat", by rounding it and fitting it within the range of
// -10 and 10 as integers.
int boundOutputs(float error);

// Computes the %error of the sampled axle ratio for the wheels "left" and "right"
// compared to the expected value at a given angle given by "expected"
// left and right are hall sensor data, expected comes from the look up table.
// The result is an integer which can be used as an index in the decision matrix.
int computeAxleDeviation(uint8_t left, uint8_t right, int expected);

// Basically the same as computeAxleDeviation, but for the overall ratio.
int computeOverallDeviation(uint8_t fl, uint8_t fr, uint8_t rl, uint8_t rr, int expected);

// This is the function that a MicroC Task will actually call
// IGNORE THE OTHER ONES THEY ARE CALLED BY THIS FUNCTION OR EACH OTHER
// 
// INPUTS: 
//wheelSpeeds is an array containing the Hall Sensor sampling data from each wheel
// in the order of FRONT LEFT, FRONT RIGHT, REAR LEFT, REAR RIGHT.
// steeringAngle is the current steering angle of the vehicle, either a positive (turning right)
// or negative (turning left) number.
// 
// OUTPUTS:
// A DYNAMICALLY ALLOCATED (E.G. MUST BE FREED) array of 5 floats.
// The speed modifiers for the wheels and the steering columns,
// in the order of 
// FRONT LEFT, FRONT RIGHT, REAR LEFT, REAR RIGHT, ANGLE
float* calculateMotorModifiers(uint8_t wheelSpeeds[4], int8_t steeringAngle);

#endif