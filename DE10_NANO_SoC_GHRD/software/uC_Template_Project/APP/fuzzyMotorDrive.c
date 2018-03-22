#include "fuzzyMotorDrive.h"


void driveMotors(float driveSpeed, MotorChangeMessage *fuzzyMods, int8_t steering, bool stopMask) {

	// Kill switch
	if (stopMask){
		// using Brian's wrapper function for the motors
		// kill power to motors (give no PWM signal to wheels)
		update_motor_control(kill, FRONT_LEFT_MOTOR );
		update_motor_control(kill, FRONT_RIGHT_MOTOR);
		update_motor_control(kill, REAR_LEFT_MOTOR  );
		update_motor_control(kill, REAR_RIGHT_MOTOR );
		return;
	}

	// Absolute value of speed
	float magnitude = fabsf(driveSpeed);
	int8_t dir = driveSpeed / magnitude;

    // Index to motoDriveR.
    int8_t steeringIndex = abs(steering) / 6;
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
	update_motor_control(dir * fl, FRONT_LEFT_MOTOR);
	update_motor_control(dir * fr, FRONT_RIGHT_MOTOR);
	update_motor_control(dir * rl, REAR_LEFT_MOTOR  );
	update_motor_control(dir * rr, REAR_RIGHT_MOTOR );
}
