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

// Input a num from -127 to 127, then a wheel, Front left = 0, Front right = 1, rear left = 2, rear right = 3

void MoveDCMotor(uint8_t hex, int wheel) {
  if (hex >= DCMotorMin && hex <= DCMotorMax) {
    if(wheel == FRONTLEFT){
      alt_write_byte(FRONT_LEFT_MOTOR_BASE, hex);
      //printf("%d wheel: %d\n", num, wheel);
    }
    else if (wheel == FRONTRIGHT){
      alt_write_byte(FRONT_RIGHT_MOTOR_BASE, hex);
      //printf("%d wheel: %d\n", num, wheel);
    }
    else if (wheel == REARLEFT){
      alt_write_byte(REAR_LEFT_MOTOR_BASE, hex);
      //printf("%d wheel: %d\n", num, wheel);
    }
    else if (wheel == REARRIGHT){
      alt_write_byte(REAR_RIGHT_MOTOR_BASE, hex);
      //printf("%d wheel: %d\n", num, wheel);
    }
  }
  

}
