#include <hps.h>
#include <stdint.h>
#include  <socal.h>
#include "fpga_to_hps.h"
#include "motor_control.h"

void* MOTOR_ADDRESSES[] = {FRONT_LEFT_MOTOR_BASE, FRONT_RIGHT_MOTOR_BASE, REAR_LEFT_MOTOR_BASE, REAR_RIGHT_MOTOR_BASE};

// drive_val = value from -1 to 1
void update_motor_control(float drive_val, int motor_position) {
	if(drive_val >= 0){
		drive_val = fmodf(drive_val, INPUT_DRIVE_VALUE_MAX);
	}else{
		drive_val = fmodf(drive_val, INPUT_DRIVE_VALUE_MIN);
	}
	int8_t new_motor_val = drive_val * OUTPUT_ABSOLUTE_MAX;
	alt_write_byte(MOTOR_ADDRESSES[motor_position], new_motor_val);
}
