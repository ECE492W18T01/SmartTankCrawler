#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
// Drive Motor #1, Front Left
// Type:  Input
// Width: Byte
#define FRONT_LEFT_MOTOR_ADD 0x00000112
#define FRONT_LEFT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(FRONT_LEFT_MOTOR_ADD)
#define FRONT_LEFT_MOTOR 0

// Drive Motor #2, Front Right
// Type:  Input
// Width: Byte
#define FRONT_RIGHT_MOTOR_ADD 0x00000113
#define FRONT_RIGHT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(FRONT_RIGHT_MOTOR_ADD)
#define FRONT_RIGHT_MOTOR 1

// Drive Motor #3, Rear Left
// Type:  Input
// Width: Byte
#define REAR_LEFT_MOTOR_ADD 0x00000114
#define REAR_LEFT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(REAR_LEFT_MOTOR_ADD)
#define REAR_LEFT_MOTOR 2

// Drive Motor #4, Rear Right
// Type:  Input
// Width: Byte
#define REAR_RIGHT_MOTOR_ADD 0x00000115
#define REAR_RIGHT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(REAR_RIGHT_MOTOR_ADD)
#define REAR_RIGHT_MOTOR 3

#define INPUT_DRIVE_VALUE_MAX 1.0
#define INPUT_DRIVE_VALUE_MIN -1.0

#define OUTPUT_ABSOLUTE_MAX 127

const void* MOTOR_ADDRESSES[] = {FRONT_LEFT_MOTOR_BASE,FRONT_RIGHT_MOTOR,REAR_LEFT_MOTOR,REAR_RIGHT_MOTOR};

#endif
