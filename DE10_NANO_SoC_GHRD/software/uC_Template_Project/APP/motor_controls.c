#include "motor_control.h"

void* MOTOR_ADDRESSES[] = {FRONT_LEFT_MOTOR_BASE, FRONT_RIGHT_MOTOR_BASE, REAR_LEFT_MOTOR_BASE, REAR_RIGHT_MOTOR_BASE};

// drive_val = value from -1 to 1
void update_motor_control(float drive_val, int motor_position) {
	printf("Motor %d: %f, %d\n", motor_position, drive_val, (int8_t)(drive_val * OUTPUT_ABSOLUTE_MAX));
	//alt_write_byte(MOTOR_ADDRESSES[motor_position], (int8_t)(drive_val * OUTPUT_ABSOLUTE_MAX));
}
