#include "fpga_to_hps.h"
#include "wrap.h"

void MoveBackServo(uint8_t hex) {
  if (hex > BackServoMax) {
	  alt_write_byte(BRAKE_SERVO_BASE, BackServoMax);
  }

  //else if (hex < BackServoMin) {
//	  alt_write_byte(BRAKE_SERVO_BASE, BackServoMin);
  //}

  else {
	  alt_write_byte(BRAKE_SERVO_BASE, hex);
  }
}

void MoveFrontServo(uint8_t hex ) {
	  alt_write_byte(BRAKE_SERVO_BASE, (int8_t)(-hex+60)/6);
}
