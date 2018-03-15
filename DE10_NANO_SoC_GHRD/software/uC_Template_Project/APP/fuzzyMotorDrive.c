#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "motorDriveR.h"
#include "motor_control.h"
#include "wrap.h"


void driveMotors(float driveSpeed, float fuzzyMods[5], int8_t steering, bool stopMask){
	
  int convert = OUTPUT_ABSOLUTE_MAX;
  // is the driveSpeed going reverse. and put the sign from driveSpeed to convert
  // to avoid copying code for both forward and reverse paths
  if (driveSpeed < forward){
    covert = -127; 
    driveSpeed = abs(driveSpeed) 
      }
  
  // else going Forward, add fuzzyMod[4] to steering
  else {
    steering += fuzzyMods[STEERING_MOTOR];
  }
  
  // holding variables 
  float fl = 0;
  float fl_tmp = 0;
  float fr = 0;
  float rl = 0;
  float rl_tmp = 0;
  float rr = 0; 
  
  //If the stop mask is true
  if (stopMask){
    // using Brian's wrapper function for the motors
    // kill power to motors (give no PWM signal to wheels)
    update_motor_control(kill, FRONT_LEFT_MOTOR );
    update_motor_control(kill, FRONT_RIGHT_MOTOR);
    update_motor_control(kill, REAR_LEFT_MOTOR  );
    update_motor_control(kill, REAR_RIGHT_MOTOR );	
  }
  
  //another else if would be too much copying, I left the reverse part near the bottom
  else{
    if (abs(steering) >= DegreeRange60){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D60][opposite];
      rl = driveSpeed*DRIVERATIO[D60][behind];
      rr = driveSpeed*DRIVERATIO[D60][across]
	}
    else if (abs(steering) >= DegreeRange54){
	    fl = driveSpeed;
	    fr = driveSpeed*DRIVERATIO[D54][opposite];
	    rl = driveSpeed*DRIVERATIO[D54][behind];
	    rr = driveSpeed*DRIVERATIO[D54][across];
    }
    else if (abs(steering) >= DegreeRange48){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D48][opposite];
      rl = driveSpeed*DRIVERATIO[D48][behind];
      rr = driveSpeed*DRIVERATIO[D48][across];
    }
    else if (abs(steering) >= DegreeRange42){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D42][opposite];
      rl = driveSpeed*DRIVERATIO[D42][behind];
      rr = driveSpeed*DRIVERATIO[D42][across];
    }
    else if (abs(steering) >= DegreeRange36){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D36][opposite];
      rl = driveSpeed*DRIVERATIO[D36][behind];
      rr = driveSpeed*DRIVERATIO[D36][across];
    }
    else if (abs(steering) >= DegreeRange30){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D30][opposite];
      rl = driveSpeed*DRIVERATIO[D30][behind];
      rr = driveSpeed*DRIVERATIO[D30][across];
    }
    else if (abs(steering) >= DegreeRange24){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D24][opposite];
      rl = driveSpeed*DRIVERATIO[D24][behind];
      rr = driveSpeed*DRIVERATIO[D24][across];
    }
    else if (abs(steering) >= DegreeRange18){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D18][opposite];
      rl = driveSpeed*DRIVERATIO[D18][behind];
      rr = driveSpeed*DRIVERATIO[D18][across];
    }
    else if (abs(steering) >= DegreeRange12){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D12][opposite];
      rl = driveSpeed*DRIVERATIO[D12][behind];
      rr = driveSpeed*DRIVERATIO[D12][across];
    }
    else if (abs(steering) >= DegreeRange6){
      fl = driveSpeed;
      fr = driveSpeed*DRIVERATIO[D6][opposite];
      rl = driveSpeed*DRIVERATIO[D6][behind];
      rr = driveSpeed*DRIVERATIO[D6][across];
    }
    else {
      fl = driveSpeed;
      fr = driveSpeed;
      rl = driveSpeed;
      rr = driveSpeed;
    }
    
    //reverse FL/RL with FR/RR variables if steering is negative.
    if (steering < reverse){
      fl_tmp = fl;
      fl = fr;
      fr = fl_tmp;
      
      rl_tmp = rl;
      rl = rr; 
      rr = rl_tmp;
    }
    
    // if any of the wheelSpeeds is greater than 1, hard cap it at 1.
    if (fl > wheelSpeed){fl = wheelSpeed;}
    else if (fr > wheelSpeed) {fr = wheelSpeed;}
    else if (rl > wheelSpeed) {rl = wheelSpeed;}
    else if (rr > wheelSpeed) {rr = wheelSpeed;}
    
    //augment if moving forward
    if(convert > forward){
      fl = fl * (1 * fuzzyMods[FRONT_LEFT_MOTOR]);
      fr = fr * (1 * fuzzyMods[FRONT_RIGHT_MOTOR]);
      rl = rl * (1 * fuzzyMods[REAR_LEFT_MOTOR]);
      rr = rr * (1 * fuzzyMods[REAR_RIGHT_MOTOR]);
    }
    
    //finally convert by multiply by +/- 127
    fl = fl * convert;
    fr = fr * convert;
    rl = rl * convert;
    rr = rr * convert;
    
    // ********************************************************************************************
    // Brian's wrapper should have the input as a range from -1 to 1
    // unsure which code we should change, Brian already multiples his input value by 127
    // just delete above four lines or 
    // delete int8_t new_motor_val = drive_val * OUTPUT_ABSOLUTE_MAX; in motor_controls.c
    update_motor_control(fl, FRONT_LEFT_MOTOR );
    update_motor_control(fr, FRONT_RIGHT_MOTOR);
    update_motor_control(rl, REAR_LEFT_MOTOR  );
    update_motor_control(rr, REAR_RIGHT_MOTOR );
    MoveFrontServo(steering);
    
  }
  
  
}
