#include "FuzzyLogicProcessor.h"
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

	int direction = 1;
	int angle = steeringAngle;

	if (steeringAngle < 0) {
		direction = -1;
		angle = angle * -1;
	}

	int frontAxleDeviation = computeAxleDeviation(wheelSpeeds[0], wheelSpeeds[1],
		direction * SLIPRATIOS[angle][1]);
	int rearAxleDeviation = computeAxleDeviation(wheelSpeeds[2], wheelSpeeds[3],
		direction * SLIPRATIOS[angle][2]);
	int overallDeviation = computeOverallDeviation(wheelSpeeds[0], wheelSpeeds[1],
		wheelSpeeds[2], wheelSpeeds[3], SLIPRATIOS[angle][0]);

	float* fsOutputs = malloc(sizeof(float) * 5);

	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDeviation][rearAxleDeviation][frontAxleDeviation][i];
	}

	return fsOutputs;
}

int main(int argc, char **argv) {
    
   	/*
	*	TEST CASE 1
	*/
	// Test case for 15 degrees left turn, no slippage
	int fl = 40; // Slowest wheel in left turn
	int fr = 54; // Fastest wheel in left turn
	int rl = 43;
	int rr = 50;
	int index = 15; // If i = 0 corresponds to -60 degrees, i = 45 corresponds to -15 degrees
	int dir = -1; // -1 for left turn, 1 for right turn

	int frontDev = computeAxleDeviation(fl, fr, dir*SLIPRATIOS[45][1]);
	int rearDev = computeAxleDeviation(rl, rr, dir*SLIPRATIOS[45][2]);
	int overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[45][0]);
	printf("Test Case 1: 15 degrees left turn, no slippage.\n");
	printf("Results in no corrective actions taken.\n");
	printf("Front Ratio - Expecting 10; Got: %d\n", frontDev);
	printf("Rear Ratio - Expecting 10; Got: %d\n", rearDev);
	printf("Overall Ratio - Expecting 10; Got: %d\n", overallDev);

	// Use this code to avoid needing to screw around with Dynamic memory.
	float fsOutputs[5];

	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {

		// ORDER OF LOOKUP - Overall, Rear, Front
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 2
	*/
	// Test case for 0 degrees left turn, no rear axle slippage
	fl = 25; // Have traction, therefore slower.
	fr = 25; 
	rl = 50; // No traction, so faster. 
	rr = 50;
	index = 0;
	dir = 1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 2: 0 degrees, rear axle slippage.\n");
	printf("Results in reduced power to rear axle (like Traction control) and increased output to front wheels.\n");
	printf("Front Ratio - Expecting 10; Got: %d\n", frontDev);
	printf("Rear Ratio - Expecting 10; Got: %d\n", rearDev);
	printf("Overall Ratio - Expecting 5; Got: %d\n", overallDev);

	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 3
	*/
	// Test case for 30 degree right turn, left wheels lose traction
	fl = 128; // If it had traction it would spin at 160
	fr = 32;  // Slowest wheel in right turn 
	rl = 120; // If it had traction it would spin at 140 or so 
	rr = 40; // Turning normally
	index = 30;
	dir = 1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 3: 30 degree right turn, left wheels lose traction.\n");
	printf("Results in increased power to right wheels, decreases to left, and a sharper right (+) turn.\n");
	printf("Front Ratio - Expecting 20; Got: %d\n", frontDev);
	printf("Rear Ratio - Expecting 20; Got: %d\n", rearDev);
	printf("Overall Ratio - Expecting 10; Got: %d\n", overallDev);

	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 4
	*/
	// Test case for 30 degree LEFT turn, right wheels lose traction
	// Should see the opposite as the above example
	fl = 32; // If it had traction it would spin at 160
	fr = 128;  // Slowest wheel in right turn 
	rl = 40; // If it had traction it would spin at 140 or so 
	rr = 120; // Turning normally
	index = 30;
	dir = -1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 4: 30 degree LEFT turn, right wheels lose traction.\n");
	printf("Results in increased power to left wheels, decreases to right, and a sharper left (-) turn.\n");
	printf("Front Ratio - Expecting 20; Got: %d\n", frontDev);
	printf("Rear Ratio - Expecting 20; Got: %d\n", rearDev);
	printf("Overall Ratio - Expecting 10; Got: %d\n", overallDev);

	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 5
	*/
	// Test case for 25 degree right turn, front wheels lose traction partially
	fl = 50; // If i
	fr = 40; // Slowest wheel in right turn 
	rl = 39; // Faster 
	rr = 30; // Has traction therefore slowest
	index = 25;
	dir = 1;
	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 5: 25 degree right turn, front wheels lose traction\n");
	printf("Results in no action as while there is slippage it isn't enough to trip the set.\n");
	printf("Front Ratio: %d\n", frontDev);
	printf("Rear Ratio; Got: %d\n", rearDev);
	printf("Overall Ratio; Got: %d\n", overallDev);

	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

		// Test 5.5
	uint8_t wheelSpeeds1[4] = {50, 40, 39, 30};
	uint8_t angle = 25;

	float* mallocFSOutputs = calculateMotorModifiers(wheelSpeeds1, angle);

	printf("Test Case 5.5: 25 degree right turn, front wheels lose traction\n");
	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		mallocFSOutputs[0], mallocFSOutputs[1], mallocFSOutputs[2], mallocFSOutputs[3], mallocFSOutputs[4]);

	// THIS IS CRITICALLY IMPORTANT!!!!
	free(mallocFSOutputs);

	/*
	*	TEST CASE 6
	*/
	// Test case for 25 degree left turn, front wheels lose traction partially
	fl = 40; // If i
	fr = 50; // Slowest wheel in right turn 
	rl = 30; // Has traction therefore slowest 
	rr = 39; // Faster being on outer circle
	index = 25;
	dir = -1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 6: 25 degree left turn, front wheels lose traction\n");
	printf("Results in no action as while there is slippage it isn't enough to trip the set.\n");
	printf("Front Ratio: %d\n", frontDev);
	printf("Rear Ratio; Got: %d\n", rearDev);
	printf("Overall Ratio; Got: %d\n", overallDev);

	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	// Test 6.5
	uint8_t wheelSpeeds2[4] = {40, 50, 30, 39};
	angle = -25;

	mallocFSOutputs = calculateMotorModifiers(wheelSpeeds2, angle);

	printf("Test Case 6.5: 25 degree left turn, front wheels lose traction\n");
	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		mallocFSOutputs[0], mallocFSOutputs[1], mallocFSOutputs[2], mallocFSOutputs[3], mallocFSOutputs[4]);

	// THIS IS CRITICALLY IMPORTANT!!!!
	free(mallocFSOutputs);

	/*
	*	TEST CASE 7
	*/
	// Test case for 25 degree right turn, front wheels lose traction (more than 5)
	fl = 54; // If i
	fr = 44; // Slowest wheel in right turn 
	rl = 39; // Faster 
	rr = 30; // Has traction therefore slowest
	index = 25;
	dir = 1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 7: 25 degree right turn, front wheels lose traction more than test 5\n");
	printf("Results in increase of power to rear wheels, decrease to front.\n");
	printf("Front Ratio: %d\n", frontDev);
	printf("Rear Ratio; Got: %d\n", rearDev);
	printf("Overall Ratio; Got: %d\n", overallDev);
	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 8
	*/
	// Test case for 25 degree left turn, front wheels lose traction (more than 6)
	fl = 44; // If i
	fr = 54; // Slowest wheel in right turn 
	rl = 30; // Has traction therefore slowest 
	rr = 39; // Faster
	index = 25;
	dir = -1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 8: 25 degree left turn, front wheels lose traction more than test 6\n");
	printf("Results in increase of power to rear wheels, decrease to front.\n");
	printf("Front Ratio: %d\n", frontDev);
	printf("Rear Ratio; Got: %d\n", rearDev);
	printf("Overall Ratio; Got: %d\n", overallDev);
	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 9
	*/
	// Test case for 10 degree right turn, low values
	fl = 6; // If i
	fr = 4; // Slowest wheel in right turn 
	rl = 4; // One rotation in a sampling period.
	rr = 3;
	index = 10;
	dir = 1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 9: 10 degree right turn, low speed\n");
	printf("Slight hampering to the front wheels, increased power to slowest (Rear right).\n");
	printf("Front Ratio: %d\n", frontDev);
	printf("Rear Ratio; Got: %d\n", rearDev);
	printf("Overall Ratio; Got: %d\n", overallDev);
	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 10
	*/
	// Test case for 10 degree left turn, low values
	fl = 4; // If i
	fr = 6; // Slowest wheel in right turn 
	rl = 3; // One rotation in a sampling period.
	rr = 4;
	index = 10;
	dir = -1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 10: 10 degree left turn, low speed\n");
	printf("Slight hampering to the front wheels, increased power to slowest (Rear left).\n");
	printf("Front Ratio: %d\n", frontDev);
	printf("Rear Ratio; Got: %d\n", rearDev);
	printf("Overall Ratio; Got: %d\n", overallDev);
	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 11
	*/
	// Test case for straight ahead, all lose traction instead of the front left wheel
	fl = 5; // Only one with traction.
	fr = 15; // Other front wheel slips, but not as much. 
	rl = 25; // Other wheels slip and spin faster.
	rr = 25;
	index = 0;
	dir = 1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 11: Straight ahead, only front left has traction\n");
	printf("Decrease rear axle, increase front axle.\n");
	printf("Front Ratio: %d\n", frontDev);
	printf("Rear Ratio; Got: %d\n", rearDev);
	printf("Overall Ratio; Got: %d\n", overallDev);
	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 12
	*/
	// Test case for straight ahead, all lose traction except for rear left wheel.
	fl = 40; // Only one with traction.
	fr = 40; // Other front wheel slips, but not as much. 
	rl = 20; // Other wheels slip and spin faster.
	rr = 40;
	index = 0;
	dir = 1;

	frontDev = computeAxleDeviation(fl, fr, dir * SLIPRATIOS[index][1]);
	rearDev = computeAxleDeviation(rl, rr, dir * SLIPRATIOS[index][2]);
	overallDev = computeOverallDeviation(fl, fr, rl, rr, SLIPRATIOS[index][0]);
	printf("Test Case 12: Straight ahead, only rear left has traction.\n");
	printf("Decrease power to everything but rear left. Minor coarse correction\n");
	printf("Front Ratio: %d\n", frontDev);
	printf("Rear Ratio; Got: %d\n", rearDev);
	printf("Overall Ratio; Got: %d\n", overallDev);
	// Use this code for assignments in the actual task.
	for (int i = 0; i < 5; i++) {
		fsOutputs[i] = FUZZYLOOKUP[overallDev][rearDev][frontDev][i];
	}

	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		fsOutputs[0], fsOutputs[1], fsOutputs[2], fsOutputs[3], fsOutputs[4]);

	/*
	*	TEST CASE 13
	*/ 
	// Test conditions from Test Case 2, now with the entirety wrapper function AND ENSURING WE FREE THE MEMROY
	// Test case for 0 degrees left turn, no rear axle slippage

	uint8_t wheelSpeeds[4] = {25, 25, 50, 50};
	angle = 0;

	mallocFSOutputs = calculateMotorModifiers(wheelSpeeds, angle);

	printf("Test Case 13: Same as Test 2, but with the overall wrapper. Just confirming functionality.\n");
	printf("Fuzzy Set Adjustment Coefficients: FL %f, FR %f, RL %f, RR %f, Angle %f\n\n",
		mallocFSOutputs[0], mallocFSOutputs[1], mallocFSOutputs[2], mallocFSOutputs[3], mallocFSOutputs[4]);

	// THIS IS CRITICALLY IMPORTANT!!!!
	free(mallocFSOutputs);

	return 0;

}