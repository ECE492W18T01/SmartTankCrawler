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

	if (hex <= -steeringMaxAngle) {
		alt_write_byte(STEER_SERVO_BASE, FrontServoMax);
	}

	else if (hex >= steeringMaxAngle) {
		alt_write_byte(STEER_SERVO_BASE, FrontServoMin);
	}

	else if (hex < steeringThres && hex > -steeringThres) {
		alt_write_byte(STEER_SERVO_BASE, FrontServoCen);
	}

	else {
		alt_write_byte(STEER_SERVO_BASE, (-hex + steerFormulaAdd) / steerFormulaDiv);
	}
}

LogMessage *_message_generator(INT8U taskID, INT8U sourceID, INT8U error, INT8U messageType, void *message) {
	INT8U err;
	LogMessage *outgoing = OSMemGet(StandardMemoryStorage, &err);
	outgoing->taskID = taskID;
	outgoing->sourceID = sourceID;
	outgoing->error = error;
	outgoing->messageType = messageType;
	outgoing->message = message;
	return outgoing;
}

LogMessage *CreateErrorMessage(INT8U taskID, INT8U sourceID, INT8U error) {
	return _message_generator(taskID, sourceID, error, 0, 0);
}

LogMessage *CreateLogMessage(INT8U messageType, void *message) {
	return _message_generator(0, 0, OS_ERR_NONE, messageType, message);
}
