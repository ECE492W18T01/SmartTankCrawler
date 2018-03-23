#include "FuzzyLogicProcessor.h"
#include "wrap.h"
/*
 * ECE 492
 * Wi18
 * Keith Mills 1442515
 * March 4th, 2018
 * Program for testing the fuzzy logic system, from dummy inputs meant to be from the hall sensors. 
*/

// Our universes of discourse for each input FS are 
// [-10, -9, -8,... -1, 0, 1,... 8, 9, 10]
// We need to take the errors and ensure they fit. 
// Higher then 10 is moved to 10,
// Lower than -10 is moved to -10,
// Anything in between is rounded towards the larger
// value in absolute terms. 
int boundOutputs(float error) {
	if (error > 10) {
		return 10;
	}

	else if (error < -10) {
		return -10;
	}

	else if (error < 0) {
		return (int)floor(error);
	}

	else {
		return (int)ceil(error);
	}
}

// Formula for computing axle difference
// And ensuring it does not get bogged down in floating points.
// Multiply numerator by maximum value of denominator, then divide.
// Take the difference between the actual ratio and the expected
// ratio given by steering LUT
// Compute the ratio, process to be integer between -10 and 10, 
// shift by 10 for array access, and return.
int computeAxleDeviation(uint8_t left, uint8_t right, int expected) {

	int numerator = 0;
	if (expected > 0) {
		numerator = (left * axleRatio) / right;
	}
	else {
		numerator = (right * axleRatio) / left;
	} 

	int difference = numerator - axleRatio;
	float error = (difference * adjustForFS) / expected;
	return boundOutputs(error) + shiftIndex;
}

// Formula for computing overall axle difference
// And ensuring it does not get bogged down in floating points.
// Multiply numerator by maximum value of denominator, then divide.
// Take the difference between the actual ratio and the expected
// ratio given by steering LUT
// Compute the ratio, process to be integer between -10 and 10, 
// shift by 10 for array access, and return.
int computeOverallDeviation(uint8_t fl, uint8_t fr, uint8_t rl, uint8_t rr, int expected) {

	int actualNumerator = (fl + fr) * overallRatio;
	int actualDenomentator = rl + rr;
	int actualFraction = actualNumerator / actualDenomentator;

	int difference = (actualFraction - overallRatio) * adjustForFS;
	float error = difference / expected;

	return boundOutputs(error) + shiftIndex;
}

float* calculateMotorModifiers(uint8_t wheelSpeeds[4], int8_t steeringAngle) {

	INT8U err;

	int8_t absoluteSteering = abs(steeringAngle);
	int8_t direction = steeringAngle / absoluteSteering;

	int8_t index = absoluteSteering/6;

	int frontAxleDeviation = computeAxleDeviation(wheelSpeeds[0], wheelSpeeds[1],
		direction * SLIPRATIOS[index][1]);
	int rearAxleDeviation = computeAxleDeviation(wheelSpeeds[2], wheelSpeeds[3],
		direction * SLIPRATIOS[index][2]);
	int overallDeviation = computeOverallDeviation(wheelSpeeds[0], wheelSpeeds[1],
		wheelSpeeds[2], wheelSpeeds[3], SLIPRATIOS[index][0]);


	float* fsOutputs = OSMemGet(StandardMemoryStorage, &err);

	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDeviation][rearAxleDeviation][frontAxleDeviation][i];
	}

	return fsOutputs;
}

int getMinWheelDiff(uint8_t wheelSpeeds[4]) {

	int min = wheelSpeeds[0];
	if (wheelSpeeds[1] < min) {
		min = wheelSpeeds[1];
	}

	if (wheelSpeeds[2] < min) {
		min = wheelSpeeds[2];
	}

	if (wheelSpeeds[3] < min) {
		min = wheelSpeeds[3];
	}

	return min;
}
