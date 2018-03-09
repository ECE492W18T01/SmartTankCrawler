#include "fpga_to_hps.h"
#include "wrap.h"

void MoveBackServo(uint8_t hex) {
  if (hex > BackServoMax) {
	  alt_write_byte(BRAKE_SERVO_BASE, BackServoMax);
  }

  else if (hex < BackServoMin) {
	  alt_write_byte(BRAKE_SERVO_BASE, BackServoMin);
  }

  else {
	  alt_write_byte(BRAKE_SERVO_BASE, hex);
  }
}

void MoveFrontServo(uint8_t hex ) {
  if (hex > FrontServoMax) {
	  alt_write_byte(BRAKE_SERVO_BASE, FrontServoMax);
  }

  else if (hex < FrontServoMin) {
	  alt_write_byte(BRAKE_SERVO_BASE, FrontServoMin);
  }

  else {
	  alt_write_byte(BRAKE_SERVO_BASE, hex);
  }

}
