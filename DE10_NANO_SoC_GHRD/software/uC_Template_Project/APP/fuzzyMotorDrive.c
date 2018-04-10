#include "fuzzyMotorDrive.h"


void driveMotors(float driveSpeed, MotorChangeMessage *fuzzyMods, float* motorVals, int8_t steering, bool stopMask) {

	// Kill switch
	if (stopMask){
		// using Brian's wrapper function for the motors
		// kill power to motors (give no PWM signal to wheels)
		motorVals[0] = coast;
		motorVals[1] = coast;
		motorVals[2] = coast;
		motorVals[3] = coast;
		return;
	}

	// Absolute value of speed
	float magnitude = fabsf(driveSpeed);
	int8_t dir = driveSpeed / magnitude;

    // Index to motoDriveR.
    int8_t steeringIndex = abs(steering) / steerDivisor;
    // Relative speed ratios, assuming a right turn (steering > 0)
    // The speeds from greatest to least are fl > rl > rr >fr
    float fl = magnitude;
	float fr = magnitude * DRIVERATIOS[steeringIndex][opposite];
	float rl = magnitude * DRIVERATIOS[steeringIndex][behind];
	float rr = magnitude * DRIVERATIOS[steeringIndex][across];

	// We are making a left turn, so the ratios are swapped.
	// fr > rr > rl > fl
	if (steering < straightAngle) {

		float fl_tmp = fl;
		float rl_tmp = rl;

		fl = fr;
		fr = fl_tmp;

		rl = rr;
		rr = rl_tmp;
	}

	// We are going forward.
	// Modulate ratios by fuzzy set.
	if (dir > forward) {
		fl *= (1 + fuzzyMods->frontLeft);
	    fr *= (1 + fuzzyMods->frontRight);
	    rl *= (1 + fuzzyMods->backLeft);
	    rr *= (1 + fuzzyMods->backRight);
	}

	// Truncate all motor speed to be less than 1
	if (fl > wheelSpeed) {
		fl = wheelSpeed;
	}

	if (fr > wheelSpeed) {
		fr = wheelSpeed;
	}

	if (rl > wheelSpeed) {
		rl = wheelSpeed;
	}

	if (rr > wheelSpeed) {
		rr = wheelSpeed;
	}

	// Move the front axle, assign new speeds
	motorVals[0] = dir * rescaleInput(fl);
	motorVals[1] = dir * rescaleInput(fr);
	motorVals[2] = dir * rescaleInput(rl);
	motorVals[3] = dir * rescaleInput(rr);
}

float rescaleInput(float input) {

	// Min value to get the wheels working is 0.6 while on the jackstand.
	// Max value of output is 1.0.
	// Use 0 -> 0.5, no move.
	// y = m*x + b
	// 1.0 = m*1 + b
	// 0.5 = m*0 + b
	return slope * input + intercept;
}
