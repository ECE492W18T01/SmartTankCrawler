#include "fpga_to_hps.h"
#include "wrap.h"

void MoveBackServo(int8_t hex) {
  if (hex > BackServoMax) {
	  alt_write_byte(BRAKE_SERVO_BASE, BackServoMax);
  }

  else {
	  alt_write_byte(BRAKE_SERVO_BASE, hex);
  }
}

void MoveFrontServo(int8_t hex ) {

	if (hex <= -60) {
		alt_write_byte(STEER_SERVO_BASE, FrontServoMax);
	}

	else if (hex >= 60) {
		alt_write_byte(STEER_SERVO_BASE, FrontServoMin);
	}

	else if (hex < 11 && hex > -11) {
		alt_write_byte(STEER_SERVO_BASE, FrontServoCen);
	}

	else {
		int8_t adder = 60;
		int8_t divider = 6;
		alt_write_byte(STEER_SERVO_BASE, (-hex + adder) / divider);
	}
}
