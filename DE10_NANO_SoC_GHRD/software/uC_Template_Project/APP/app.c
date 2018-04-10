/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*                          (c) Copyright 2009-2014; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                          APPLICATION CODE
*
*                                            CYCLONE V SOC
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : JBL
* Modifications	: Nancy Minderman nancy.minderman@ualberta.ca, Brendan Bruner bbruner@ualberta.ca
* 				  Changes to this project include scatter file changes and BSP changes for port from
* 				  Cyclone V dev kit board to DE1-SoC
*
*				  Keith Mills kgmills@ualberta.ca
*				  Reworked for project, testing of components, for usage on the DE10-Nano.
*				  Later programmed Motor Task and Fuzzy Task
*
*				  Joshua Robertson jcrobert@ualberta.ca
*				  Further reworked to add our necessary tasks, queues, and other data structures.
*
*				  Brian Ofrim bofrim@ualberta.ca
*				  Sonar Task
*
*				  Fredric Mendi fmendi@ualberta.ca
*				  Toggle Task
*
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/
#include <math.h>

#include  <app_cfg.h>
#include  <lib_mem.h>

#include  <bsp.h>
#include  <bsp_int.h>
#include  <bsp_os.h>
#include  <cpu_cache.h>

#include  <cpu.h>
#include  <cpu_core.h>

#include  <os.h>
#include  <hps.h>
#include  <socal.h>
#include  <hwlib.h>

#include "globals.h"
#include "motor_control.h"
#include "wrap.h"
#include "serial_communication.h"
#include "FuzzyLogicProcessor.h"
#include "fuzzyMotorDrive.h"
#include "timer.h"
#include "sonar.h"

#include <alt_bridge_manager.h>

// From ../UTIL
#include <circular_buf_position.h>

// Added in to get OS_ENTER/EXIT_CRITCAL working
// Taken from os_mem.c
#if OS_CRITICAL_METHOD == 3u                          /* Allocate storage for CPU status register      */
    OS_CPU_SR  cpu_sr = 0u;
#endif

/*
*********************************************************************************************************
*                              Priority Definitions and Task Stack Size
*********************************************************************************************************
*/
#define APP_TASK_PRIO 4
#define COLLISION_TASK_PRIO 5
#define MOTOR_TASK_PRIO 6
#define FUZZY_TASK_PRIO 7
#define COMMUNICATION_TASK_PRIO 8
#define LOG_TASK_PRIO 9
#define TOGGLE_TASK_PRIO 10

#define TASK_STACK_SIZE 4096

/*
*********************************************************************************************************
*                                            Communication Size
*********************************************************************************************************
*/
// actually only 127
#define RX_FIFO_SIZE 255

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

CPU_STK AppTaskStartStk[TASK_STACK_SIZE];
CPU_STK ColTaskStk[TASK_STACK_SIZE];
CPU_STK MotorTaskStk[TASK_STACK_SIZE];
CPU_STK FuzzyTaskStk[TASK_STACK_SIZE];
CPU_STK ComTaskStk[TASK_STACK_SIZE];
CPU_STK LogTaskStk[TASK_STACK_SIZE];
CPU_STK ToggleTaskStk[TASK_STACK_SIZE];

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart              (void        *p_arg);
static  void  CollisionTask             (void        *p_arg);
static  void  MotorTask                 (void        *p_arg);
static  void  FuzzyTask                 (void        *p_arg);
static  void  CommunicationTask         (void        *p_arg);
static  void  LogTask               	(void        *p_arg);
static  void  ToggleTask                (void        *p_arg);


/*
*********************************************************************************************************
*                                          GLOBAL ITEMS
*********************************************************************************************************
*/

// MessageQueues
OS_EVENT *LogQueue;
OS_EVENT *MotorQueue;
OS_EVENT *FuzzyQueue;
OS_EVENT *CollisionQueue;

// Semaphores
OS_EVENT *SteeringSemaphore;
OS_EVENT *MaskSemaphore;
OS_EVENT *CommunicationSemaphore;
OS_EVENT *RxDataAvailableSemaphore;
OS_EVENT *UserSemaphore;
OS_EVENT *FuzzyToggleSemaphore;
OS_EVENT *SonarDataAvailableSemaphore;

// Resources
int8_t userSteer;
float userMag;
int8_t globalSteeringAngle;
bool FuzzyToggle = true;          //starting true, flip the switch to turn off
bool motorMask = false;
char* userMessage;
bool enable_sonar;
circular_buf_position* distance_buffer;

OS_MEM *StandardMemoryStorage;
INT8U StandardMemory[1024][32];

OS_MEM *LargeMemoryStorage;
INT8U LargeMemory[16][256];

/*
*********************************************************************************************************
*                                               main()
*
* Description : Entry point for C code.
*
* Arguments   : none.
*
* Returns     : none.
*
* Note(s)     : (1) It is assumed that your code will call main() once you have performed all necessary
*                   Initialization.
*********************************************************************************************************
*/
int main ()
{
    INT8U os_err;

    BSP_WatchDog_Reset();                                       /* Reset the watchdog as soon as possible.              */

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                /* Scatter loading is complete. Now the caches can be activated.*/
    BSP_BranchPredictorEn();                                    /* Enable branch prediction.                            */
    BSP_L2C310Config();                                         /* Configure the L2 cache controller.                   */
    BSP_CachesEn();                                             /* Enable L1 I&D caches + L2 unified cache.             */


    CPU_Init();

    Mem_Init();

    BSP_Init();

    OSInit();

    ALT_BRIDGE_t lw_bridge = ALT_BRIDGE_LWH2F; // Code from Tutorial for Flashing to SD Card
    ALT_STATUS_CODE err2 = alt_bridge_init(lw_bridge, NULL, NULL); // Placed here before I/O calls

    // Set the global steering angle to 0 on start.
    globalSteeringAngle = initSteeringAngle;
    userSteer = initSteeringAngle;
    userMag = initUserMag	;
    MoveFrontServo(FrontServoCen); // 'Center Steering Rod'
    MoveBackServo(BackServoMin); // Raise back servo up.

    //Array of pointers that the Queues will use. Should never be touched again.
    void *LogMessageArray[LogMessageSize];
    void *MotorMessageArray[StdMessageSize];
    void *FuzzyMessageArray[StdMessageSize];
    void *CollisionMessageArray[StdMessageSize];

    // Create the Queues using above arrays to save message pointers.
    CollisionQueue = OSQCreate(CollisionMessageArray, StdMessageSize);
    LogQueue = OSQCreate(LogMessageArray, StdMessageSize);
    MotorQueue = OSQCreate(MotorMessageArray, StdMessageSize);
    FuzzyQueue = OSQCreate(FuzzyMessageArray, StdMessageSize);
    /* Create the semaphores
     * Zero means that the semaphore will be used as a flag
     * NonZero means the semaphore will be used to control access to shared resource(s).
     */
    UserSemaphore = OSSemCreate(initSemWith);
    SteeringSemaphore = OSSemCreate(initSemWith);			// One shared resource
    MaskSemaphore = OSSemCreate(initSemWith);				// One shared resource
    CommunicationSemaphore = OSSemCreate(initSemWithout);	// Turn flag on when interrupt writes
    RxDataAvailableSemaphore = OSSemCreate(initSemWithout);	// Turn flag on when interrupt writes
    SonarDataAvailableSemaphore = OSSemCreate(initSemWithout);// Turn flag on when interrupt writes
    FuzzyToggleSemaphore = OSSemCreate(initSemWith);      // One Shared resource

    // Initialize the Memory.
    INT8U err;

    StandardMemoryStorage = OSMemCreate(StandardMemory, 1024, 32, &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    LargeMemoryStorage = OSMemCreate(LargeMemory, 4, 256, &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }


    userMessage =  OSMemGet(LargeMemoryStorage, &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    distance_buffer = (circular_buf_position*)OSMemGet(LargeMemoryStorage, &err);
    dist_circular_buf_init(distance_buffer);


    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    // Watchdog task.
    os_err = OSTaskCreateExt((void (*)(void *)) AppTaskStart,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&AppTaskStartStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) APP_TASK_PRIO,
                             (INT16U          ) APP_TASK_PRIO,  // reuse prio for ID
                             (OS_STK        * )&AppTaskStartStk[0],
                             (INT32U          ) TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    if (os_err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
    // Collision Task
    os_err = OSTaskCreateExt((void (*)(void *)) CollisionTask,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&ColTaskStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) COLLISION_TASK_PRIO,
                             (INT16U          ) COLLISION_TASK_PRIO,  // reuse prio for ID
                             (OS_STK        * )&ColTaskStk[0],
                             (INT32U          ) TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    if (os_err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    // Motor Driver Task
    os_err = OSTaskCreateExt((void (*)(void *)) MotorTask,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&MotorTaskStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) MOTOR_TASK_PRIO,
                             (INT16U          ) MOTOR_TASK_PRIO,  // reuse prio for ID
                             (OS_STK        * )&MotorTaskStk[0],
                             (INT32U          ) TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    if (os_err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    // Fuzzy Logic Task
    os_err = OSTaskCreateExt((void (*)(void *)) FuzzyTask,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&FuzzyTaskStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) FUZZY_TASK_PRIO,
                             (INT16U          ) FUZZY_TASK_PRIO,  // reuse prio for ID
                             (OS_STK        * )&FuzzyTaskStk[0],
                             (INT32U          ) TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    if (os_err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    // Communication Task
    os_err = OSTaskCreateExt((void (*)(void *)) CommunicationTask,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&ComTaskStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) COMMUNICATION_TASK_PRIO,
                             (INT16U          ) COMMUNICATION_TASK_PRIO,  // reuse prio for ID
                             (OS_STK        * )&ComTaskStk[0],
                             (INT32U          ) TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    if (os_err != OS_ERR_NONE) {
         ; /* Handle error. */
     }

    // Logging Task
    os_err = OSTaskCreateExt((void (*)(void *)) LogTask,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&LogTaskStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) LOG_TASK_PRIO,
                             (INT16U          ) LOG_TASK_PRIO,  // reuse prio for ID
                             (OS_STK        * )&LogTaskStk[0],
                             (INT32U          ) TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    if (os_err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    // Switch Toggle Task
    os_err = OSTaskCreateExt((void (*)(void *)) ToggleTask,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&ToggleTaskStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) TOGGLE_TASK_PRIO,
                             (INT16U          ) TOGGLE_TASK_PRIO,  // reuse prio for ID
                             (OS_STK        * )&ToggleTaskStk[0],
                             (INT32U          ) TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    if (os_err != OS_ERR_NONE) {
            ; /* Handle error. */
    }

    CPU_IntEn();

    OSStart();

}


/*
*********************************************************************************************************
*                                           App_TaskStart()
*
* Description : Pats the watchdog every second
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*********************************************************************************************************
*/
static  void  AppTaskStart (void *p_arg)
{

    BSP_OS_TmrTickInit(OS_TICKS_PER_SEC);                       /* Configure and enable OS tick interrupt.              */
    InitHallSensorInterrupt();
    InitDistanceSensorInterrupt();
    serial_communication_init();

    for(;;) {
        BSP_WatchDog_Reset();
        OSTimeDlyHMSM(0, 0, 1, 0);
    }
}

/*
*********************************************************************************************************
*                                           CollisionTask
*
* Description : Reads from the Sonar and determines whether or not to deploy the emergency brake/halt motors
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
*********************************************************************************************************
*/
static void CollisionTask (void *p_arg)
{
	INT8U err;

	// let values accumulate to a while
	OSTimeDlyHMSM(0,0,1,0);
	enable_sonar = true;

	int8_t stop_tally = 0;

	int8_t tally_threshold = HIGHER_TALLY_THRESHOLD;

    for(;;) {

		OSSemPend(SonarDataAvailableSemaphore, 0, &err);

		uint8_t latest_distance_sample = 0;
		uint8_t offset_distance_sample = 0;

		 CPU_CRITICAL_ENTER();
		 dist_circular_buffer_get_nth(distance_buffer, &latest_distance_sample, 0);
		 dist_circular_buffer_get_nth(distance_buffer, &offset_distance_sample, SAMPLE_OFFSET);
		 CPU_CRITICAL_EXIT();

		// set the tally threshold based on distance
		tally_threshold = (latest_distance_sample < LOWER_DISTANCE_THRESHOLD) ? LOWER_TALLY_THRESHOLD : HIGHER_TALLY_THRESHOLD;

		// calculate the delta between current and offset
		int position_delta =  offset_distance_sample - latest_distance_sample;

		// velocity in inches per second
		int velocity = position_delta * (1/(SONAR_INTERUPT_PERIOD * SAMPLE_OFFSET));

		// velocity cutoff
		int8_t cutoff_velocity = (int8_t) LIMIT_VALUE_SIGNED(velocity, INT8_T_ABS_MAX);


		int estimated_stopping_distance = distance_to_stop(cutoff_velocity);
		int estimated_stopping_distance_safety = estimated_stopping_distance * STOPPING_SAFETY_FACTOR;

		if(latest_distance_sample <= STOPPING_LIMIT && estimated_stopping_distance_safety >= latest_distance_sample){
			//printf("STOP!!\n");
			stop_tally ++;
		}
		else if(stop_tally > 0){ // no collision detected decrement stop tally
			stop_tally--;
		}

		if (stop_tally == tally_threshold) {
			//printf("STOP Tally hit!!\n");
			stop_tally = 0;
			OS_ENTER_CRITICAL();
			stop_all_motors();
			MoveBackServo(BackServoMax);
			OS_EXIT_CRITICAL();

			OSSemPend(MaskSemaphore, OS_TICKS_PER_SEC/100, &err);
			motorMask = true;

			if (err == OS_ERR_NONE) {
				OSSemPost(MaskSemaphore);
			}

			OSTimeDlyHMSM(0, 0, brakeTimeDelaySec, 0);
			OSSemPend(MaskSemaphore, OS_TICKS_PER_SEC/100, &err);
			motorMask = false;

			if (err == OS_ERR_NONE) {
				OSSemPost(MaskSemaphore);
			}

		}else if (stop_tally > fakeOutThreshold) {
			MoveBackServo(BackServoMax * (tally_threshold - stop_tally) / (tally_threshold -1));
		}else{
			MoveBackServo(BackServoMin);
		}

		if (err == OS_ERR_NONE)
		{
			uint8_t *sendDist = OSMemGet(StandardMemoryStorage, &err);
			*sendDist = latest_distance_sample;
			OSQPost(LogQueue, CreateLogMessage(DISTANCE_MESSAGE, sendDist));

		} else {
			OSQPost(LogQueue, CreateErrorMessage(COLLISION_TASK, OS_Q_PEND, err));
        }
    }
}

/*
*********************************************************************************************************
*                                           MotorTask
*
* Description : Gets motor magnitudes from Fuzzy Task and Communication Task. Responsible for writing
* to DC motor PWM VHDL Components and steering servo.
* Control of motors can be masked by a global (semaphore protected) boolean set/unset by CollisionTask
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
*********************************************************************************************************
*/
static void MotorTask (void *p_arg)
{
	INT8U err;

	// Incoming from Fuzzy Task, staticFuzzy is local and static allocated
    MotorChangeMessage *incoming;
    MotorChangeMessage staticFuzzy = { staticFuzzyInit };

    // Allocated ONCE. We use OS_ENTER_CRITICAL to drive motors so we give this array to a function and it returns values we use
    // Inside critical section
    float *indMotorVals = malloc(sizeof(float) * numMotorFloat);

    // User-input speed.
    float userDriveSpeed = initUserSpeed;

    // This is NOT the global emergency brake variable, but later in the code it is assigned that variable's value.
    bool allStop = false;

    // Old steering, new steering, and the actual value to be plugged.
    int8_t oldUserSteer, newUserSteer, actualSteeringAngle = initSteeringAngle;
    MoveFrontServo(actualSteeringAngle);

    // Loop forever.
    for(;;) {

    	// Get user mag/direction data.
		OSSemPend(UserSemaphore, motorCommTO, &err);
		newUserSteer = userSteer;
		userDriveSpeed = userMag;
		OSSemPost(UserSemaphore);

		// Fuzzy Set Pend.
    	// Only pend for so long - Change timeout value to be define, but the number now is 1/8 of a second.
    	// Subject to change alongside timeouts for other queues.
    	incoming = (MotorChangeMessage*)OSQPend(MotorQueue, OS_TICKS_PER_SEC / motorFuzzTO, &err);

    	// We timed out, so re-use the old fuzzy information.
    	if (err == OS_ERR_TIMEOUT) {
    		// And let us know that everything is fine.
    		err = OS_ERR_NONE;
    	}

    	// We got info from the Fuzzy Task (or used old data), so assign it to the static task.
    	else {
    		staticFuzzy.frontLeft = incoming->frontLeft;
    		staticFuzzy.frontRight = incoming->frontRight;
    		staticFuzzy.backLeft = incoming->backLeft;
    		staticFuzzy.backRight = incoming->backRight;
    		staticFuzzy.steeringServo = incoming->steeringServo;

    		// Send fuzzy info to Log Task to pass out of system.
    		OSQPost(LogQueue, CreateLogMessage(MOTOR_CHANGE_MESSAGE, incoming));
    	}

		if(err == OS_ERR_NONE) {

			// If the user wants the servo "moved", don't listen to the fuzzy task.
			if ((newUserSteer - oldUserSteer) / steerDivisor == 0) {
				// But if they didn't move the joystick enough, move it.
				OSSemPend(SteeringSemaphore, 0, &err);
				globalSteeringAngle += staticFuzzy.steeringServo;
				actualSteeringAngle = globalSteeringAngle;
				OSSemPost(SteeringSemaphore);
			}

			// Only listen to the user.
			else {
				OSSemPend(SteeringSemaphore, 0, &err);
				globalSteeringAngle = newUserSteer;
				OSSemPost(SteeringSemaphore);

				actualSteeringAngle = newUserSteer;
			}

			// Old = new, for the next iteration.
			oldUserSteer = newUserSteer;

			// Check if we're supposed to cut the engines.
			OSSemPend(MaskSemaphore, 0, &err);
			allStop = motorMask;
			OSSemPost(MaskSemaphore);


			// Pass user speed, fuzzy info, motor value array, steering, and stop mask.
			// This function does not return anything, rather it will modify the values
			// in indMotorVals
			driveMotors(userDriveSpeed, &staticFuzzy, indMotorVals, actualSteeringAngle, allStop);

			// Critical Section when moving the hardware. We don't want to move 2/4 wheels and then get pulled out.
			OS_ENTER_CRITICAL();
			MoveFrontServo(actualSteeringAngle);
			update_motor_control(indMotorVals[0], FRONT_LEFT_MOTOR);
			update_motor_control(indMotorVals[1], FRONT_RIGHT_MOTOR);
			update_motor_control(indMotorVals[2], REAR_LEFT_MOTOR);
			update_motor_control(indMotorVals[3], REAR_RIGHT_MOTOR);
			OS_EXIT_CRITICAL();

			// Assemble a message to be sent off the system about what the vehicle is outputing to each motor.
			MotorChangeMessage *mOutput = OSMemGet(StandardMemoryStorage, &err);
			mOutput->frontLeft = indMotorVals[0];
			mOutput->frontRight = indMotorVals[1];
			mOutput->backLeft = indMotorVals[2];
			mOutput->backRight = indMotorVals[3];
			mOutput->steeringServo = actualSteeringAngle;

			OSQPost(LogQueue, CreateLogMessage(MOTOR_OUTPUT_MESSAGE, mOutput));

		} else {
			OSQPost(LogQueue, CreateErrorMessage(MOTOR_TASK, OS_Q_PEND, err));
		}
    }
}

/*
*********************************************************************************************************
*                                           FuzzyTask
*
* Description : Gets information from Hall Sensors at a 2Hz rate.
* Processes information to determine relative speeds.
* Accesses global variable on steering for lookup table, does not modify though.
* Uses Fuzzy Logic LUTs to determine fuzzy modifiers.
* Sends to Motor Task.
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
*********************************************************************************************************
*/
static void FuzzyTask (void *p_arg) {
	INT8U err;

	// First struct from Hall Sensor Interrupt.
	// Second to Motor Task.
    HallSensorMessage *incoming;
    MotorChangeMessage outgoing;

	// 'new' Variables, for incoming hall sensor information.
	uint8_t newFrontLeft, newFrontRight, newRearLeft, newRearRight;

	// 'old' Variables, since we take the different in Software
	uint8_t oldFrontLeft = initOldWheels;
	uint8_t oldFrontRight = initOldWheels;
	uint8_t oldRearLeft = initOldWheels;
	uint8_t oldRearRight = initOldWheels;

	// Initialize the wheelspeeds.
	uint8_t wheelSpeeds[4] = {initOldWheels};

	// Local variable for steering, we only access the semaphore once.
	int8_t localSteeringAngle = initSteeringAngle;

	// Loop forever.
    for(;;) {

    	// Await incoming Hall Sensor information at frequency set by timer.h/FuzzyTaskTimer
     	incoming = (HallSensorMessage*)OSQPend(FuzzyQueue, fuzzyIntTO, &err);

     	// Assuming nothing bad happened, get new hall sensor information.
    	if (err == OS_ERR_NONE) {
            newFrontLeft = incoming->frontLeft;
            newFrontRight = incoming->frontRight;
            newRearLeft = incoming->backLeft;
            newRearRight = incoming->backRight;

            // And 'free' the memory.
            OSMemPut(StandardMemoryStorage, incoming);

            // Get the steering angle.
    		OSSemPend(SteeringSemaphore, fuzzyIntTO, &err);
    		localSteeringAngle = globalSteeringAngle;
    		OSSemPost(SteeringSemaphore);

    		// Calculate relative speeds, new - old; the + 1 exists to nullify divide by zero errors.
            wheelSpeeds[0] = newFrontLeft - oldFrontLeft + diffAvoidDiv0;
            wheelSpeeds[1] = newFrontRight - oldFrontRight + diffAvoidDiv0;
            wheelSpeeds[2] = newRearLeft - oldRearLeft + diffAvoidDiv0;
            wheelSpeeds[3] = newRearRight - oldRearRight + diffAvoidDiv0;

            // Decide whether or not we even want to access the Fuzzy Logic Matrix.
            if ((getMinWheelDiff(wheelSpeeds) < speedThres && abs(localSteeringAngle) < lowAngle) || !FuzzyToggle) {

            	// Assign zero to all modifiers.
				outgoing.frontLeft = nullFuzzy;
				outgoing.frontRight = nullFuzzy;
				outgoing.backLeft = nullFuzzy;
				outgoing.backRight = nullFuzzy;
				outgoing.steeringServo = nullFuzzy;

            }

            // If we do access the matrix,
            else {

            	// Insert inputs, get outputs. See FuzzyLogicProcessor.c for more info on how this works.
                float *fuzzyOutput = calculateMotorModifiers(wheelSpeeds, localSteeringAngle);

                // Assign outgoing variables to be these.
                outgoing.frontLeft = fuzzyOutput[0];
                outgoing.frontRight = fuzzyOutput[1];
                outgoing.backLeft = fuzzyOutput[2];
                outgoing.backRight = fuzzyOutput[3];
                outgoing.steeringServo = fuzzyOutput[4];

                // Put the temporary memory borrowed by calculateMotorModifiers back.
                OSMemPut(StandardMemoryStorage, fuzzyOutput);

            }

            // Old is now new.
            oldFrontLeft = newFrontLeft;
            oldFrontRight = newFrontRight;
            oldRearLeft = newRearLeft;
            oldRearRight = newRearRight;

            // Created a message to send to the Log Task
            HallSensorMessage *hsm = OSMemGet(StandardMemoryStorage, &err);
            hsm->frontLeft = wheelSpeeds[0];
            hsm->frontRight = wheelSpeeds[1];
            hsm->backLeft = wheelSpeeds[2];
            hsm->backRight = wheelSpeeds[3];

            err = OSQPost(LogQueue, CreateLogMessage(HALL_SENSOR_MESSAGE, hsm));

    	}

    	else {
    		OSQPost(LogQueue, CreateErrorMessage(FUZZY_TASK, OS_Q_PEND, err));
        }

    	// Send the message off.
    	MotorChangeMessage *_;
    	if (err == OS_ERR_NONE) _ = OSMemGet(StandardMemoryStorage, &err);
    	if (err == OS_ERR_NONE)
    	{
    		_->frontLeft = outgoing.frontLeft;
    		_->frontRight = outgoing.frontRight;
    		_->backLeft = outgoing.backLeft;
    		_->backRight = outgoing.backRight;
    		_->steeringServo = outgoing.steeringServo;

    		OSQPost(MotorQueue, _);
    	} else {
    		OSQPost(LogQueue, CreateErrorMessage(FUZZY_TASK, OS_MEM_GET, err));
        }
    }
}

/*
*********************************************************************************************************
*                                           CommunicationTask
*
* Description : Receives instructions from Raspberry Pi.
* Capable of timing out if it doesn't receive an instruction and shut down the motors.
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
*********************************************************************************************************
*/
static void CommunicationTask (void *p_arg)
{
	INT8U err;
	bool communications_established = false;
	//Get memory IncomingMessageStorage-->incomingMessage
	bzero(userMessage, MSG_BUFFER_LEN);

    for(;;) {

    	OSSemPend(RxDataAvailableSemaphore, OS_TICKS_PER_SEC/SerialTO, &err);
    	if (err == OS_ERR_TIMEOUT) {
    		err = OS_ERR_NONE;
    		// set motor values to 0
			OSSemPend(UserSemaphore, 0, &err);
			if (err != OS_ERR_NONE) OSQPost(LogQueue, CreateErrorMessage(COMMUNICATION_TASK, OS_SEM_PEND, err));
			userSteer = MOTOR_ZERO;
			userMag = STEERING_ZERO;
			communications_established = false;
			OSSemPost(UserSemaphore);
    	}
    	else if(err == OS_ERR_NONE){
    		// now determine what to do with the incoming message
    		if(communications_established == true){
    			// look for end byte
    			if(look_for_end_byte(userMessage) == true){
					// termination character received
					bzero(userMessage, RX_FIFO_SIZE);
					communications_established = false;
    			}else if(complete_message_revived(userMessage) == true){
					incoming_msg new_msg = parse_incomming_msg(userMessage);
					// valid message received
					serial_send(ACKNOWLEDGE_STR);
					bzero(userMessage, RX_FIFO_SIZE);

					// Post to Semaphore
					OSSemPend(UserSemaphore, 0, &err);
					if (err != OS_ERR_NONE) OSQPost(LogQueue, CreateErrorMessage(COMMUNICATION_TASK, OS_SEM_PEND, err));
					userSteer = new_msg.steering_value;
					userMag = new_msg.motor_level;

					OSSemPost(UserSemaphore);
    			}
    		}else{
    			//look for start byte
    			communications_established = look_for_start_byte(userMessage, strlen(userMessage));
    			if(communications_established == true){
    				OSTimeDlyHMSM(0, 0, 0, initTOmillisec); // delay
    				serial_send(ACKNOWLEDGE_STR);
    				OSSemPost(RxDataAvailableSemaphore);
    			}else{
    				// garbage, clear incoming buffer
    				bzero(userMessage, RX_FIFO_SIZE);
    			}
    		}
    	}
    }
}

/*
*********************************************************************************************************
*                                           LogTask
*
* Description : Accumulates messages from other tasks and sends back to Pi to be posted by website
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
*********************************************************************************************************
*/
static void LogTask (void *p_arg)
{

	INT8U err;
	LogMessage *incoming;
	void *message;
	char *outgoing = OSMemGet(LargeMemoryStorage, &err);

    for(;;) {
    	incoming = (LogMessage*)OSQPend(LogQueue, 0, &err);
    	bzero(outgoing, bufferSize);

    	if (incoming->error == OS_ERR_NONE) {
    		// This is a standard message

    		switch (incoming->messageType) {

    		// This is the Hall Sensor Differences. E.g. number of edges per wheel in the interval.
    		case HALL_SENSOR_MESSAGE:
    			message = (HallSensorMessage*)(incoming->message);
    			// Format the message as a JSON item.
    			sprintf(outgoing + strlen(outgoing), "%s{ ", MESSAGE_START_STR );
    			sprintf(outgoing + strlen(outgoing), "MessageType: %d , MessageData : { ", HALL_SENSOR_MESSAGE );
    			sprintf(outgoing + strlen(outgoing), "fl : %d, ", ((HallSensorMessage*)message)->frontLeft);
    			sprintf(outgoing + strlen(outgoing), "fr : %d, ", ((HallSensorMessage*)message)->frontRight);
    			sprintf(outgoing + strlen(outgoing), "bl : %d, ", ((HallSensorMessage*)message)->backLeft);
    			sprintf(outgoing + strlen(outgoing), "br : %d ", ((HallSensorMessage*)message)->backRight);
    			sprintf(outgoing + strlen(outgoing), "} }%s\n", MESSAGE_END_STR);

    			break;

    		// This is the Fuzzy Set output.
    		case MOTOR_CHANGE_MESSAGE:

    	    	message = (MotorChangeMessage*)(incoming->message);
    	    	// Format the message as a JSON item.
    	    	sprintf(outgoing + strlen(outgoing), "%s{ ", MESSAGE_START_STR );
    	    	sprintf(outgoing + strlen(outgoing), "MessageType: %d , MessageData : { ", MOTOR_CHANGE_MESSAGE );
    	    	sprintf(outgoing + strlen(outgoing), "fl : %f, ", ((MotorChangeMessage*)message)->frontLeft);
    	    	sprintf(outgoing + strlen(outgoing), "fr : %f, ", ((MotorChangeMessage*)message)->frontRight);
    	    	sprintf(outgoing + strlen(outgoing), "bl : %f, ", ((MotorChangeMessage*)message)->backLeft);
    	    	sprintf(outgoing + strlen(outgoing), "br : %f, ", ((MotorChangeMessage*)message)->backRight);
    	    	sprintf(outgoing + strlen(outgoing), "sS : %d ", ((MotorChangeMessage*)message)->steeringServo);
    	    	sprintf(outgoing + strlen(outgoing), "} }%s\n", MESSAGE_END_STR);

    			break;

    		// These are the values being sent to the FPGA.
    		case MOTOR_OUTPUT_MESSAGE:

    	    	message = (MotorChangeMessage*)(incoming->message);
    	    	// Format the message as a JSON item.
    	    	sprintf(outgoing + strlen(outgoing), "%s{ ", MESSAGE_START_STR );
    	    	sprintf(outgoing + strlen(outgoing), "MessageType: %d , MessageData : { ", MOTOR_OUTPUT_MESSAGE );
    	    	sprintf(outgoing + strlen(outgoing), "fl : %f, ", ((MotorChangeMessage*)message)->frontLeft);
    	    	sprintf(outgoing + strlen(outgoing), "fr : %f, ", ((MotorChangeMessage*)message)->frontRight);
    	    	sprintf(outgoing + strlen(outgoing), "bl : %f, ", ((MotorChangeMessage*)message)->backLeft);
    	    	sprintf(outgoing + strlen(outgoing), "br : %f, ", ((MotorChangeMessage*)message)->backRight);
    	    	sprintf(outgoing + strlen(outgoing), "sS : %d ", ((MotorChangeMessage*)message)->steeringServo);
    	    	sprintf(outgoing + strlen(outgoing), "} }%s\n", MESSAGE_END_STR);
    			break;

    		case TOGGLE_MESSAGE:
    	    	message = (toggleMessage*)(incoming->message);
    	    	// Format the message as a JSON item.
    	    	sprintf(outgoing + strlen(outgoing), "%s{ ", MESSAGE_START_STR );
    	    	sprintf(outgoing + strlen(outgoing), "MessageType: %d , MessageData : { ", TOGGLE_MESSAGE );
    	    	sprintf(outgoing + strlen(outgoing), "brake : %d, ", ((toggleMessage*)message)->brake);
    	    	sprintf(outgoing + strlen(outgoing), "fuzzy : %d ", ((toggleMessage*)message)->fuzzy);
    	    	sprintf(outgoing + strlen(outgoing), "} }%s\n", MESSAGE_END_STR);
    	    	// Send the message to the Pi.
    			break;

    		case DISTANCE_MESSAGE:
    	    	sprintf(outgoing + strlen(outgoing), "%s{ ", MESSAGE_START_STR );
    	    	sprintf(outgoing + strlen(outgoing), "MessageType: %d , MessageData : { ", DISTANCE_MESSAGE );
    	    	sprintf(outgoing + strlen(outgoing), "Distance : %d ", *(uint8_t*)(incoming->message));
    	    	sprintf(outgoing + strlen(outgoing), "} }%s\n", MESSAGE_END_STR);
    			break;

    		default:
    			// Should never get here...
    			break;
    		}

    	// Put back the embedded message memory after using it above.

    	OSMemPut(StandardMemoryStorage, incoming->message);

    	} else {
    		// This is an error message
    		// Format the message as a JSON item.
    		sprintf(outgoing + strlen(outgoing), "%s{ ", MESSAGE_START_STR );
    		sprintf(outgoing + strlen(outgoing), "MessageType: %d , MessageData : { ", ERROR_MESSAGE );
    		sprintf(outgoing + strlen(outgoing), "taskID : %d, ", incoming->taskID);
    		sprintf(outgoing + strlen(outgoing), "sourceID : %d, ", incoming->sourceID);
    		sprintf(outgoing + strlen(outgoing), "error : %d ", incoming->error);
    		sprintf(outgoing + strlen(outgoing), "} }%s\n", MESSAGE_END_STR);
    		// Send the message to the Pi.
    	};
    	// Send the message to the Pi.
    	OS_ENTER_CRITICAL();
    	serial_send(outgoing);
    	OS_EXIT_CRITICAL();


    	// Put back the incoming message memory. The embedded message was put back above.
    	OSMemPut(StandardMemoryStorage, incoming);
    }


}

/*
*********************************************************************************************************
*                                           ToggleTask
*
* Description : On a timer, checks the state of the DIP switches and disables/enables
* FuzzyTask and CollisionTask
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
*********************************************************************************************************
*/
static void ToggleTask(void *p_arg)
{

	INT8U err; //, send_err;
	int EnableFuzzy = alt_read_byte(SW_BASE) ;
	bool ftoggle, stoggle;
	toggleMessage myToggle;

	int prev = 0;
	int curr = 0;

    for(;;) {

    	// Use logical AND to check DIP switches
    	ftoggle = alt_read_byte(SW_BASE) & fuzzyANDConst;
    	stoggle = alt_read_byte(SW_BASE) & servoANDConst;

    	// Enable both features
    	if(!ftoggle && !stoggle){
    		OSSemPend(FuzzyToggleSemaphore, 0, &err);
    		FuzzyToggle = true;
        	OSSemPost(FuzzyToggleSemaphore);
    		enable_sonar = true;
    		alt_write_byte(LEDR_BASE, LEDSONALL);
    		curr = bothOn;

    		myToggle.brake = true;
    		myToggle.fuzzy = true;

    	}

    	// Enable only the emergency brake
    	else if(!stoggle){
    		OSSemPend(FuzzyToggleSemaphore, 0, &err);
    		FuzzyToggle = false;
        	OSSemPost(FuzzyToggleSemaphore);
    		enable_sonar = true;
    		alt_write_byte(LEDR_BASE, LEDSBrake);
    		curr = onlyBrake;

    		myToggle.brake = true;
    		myToggle.fuzzy = false;
    	}

    	// Enable only the Fuzzy Set
    	else if (!ftoggle) {
    		enable_sonar = false;
    		OSSemPend(FuzzyToggleSemaphore, 0, &err);
    		FuzzyToggle = true;
        	OSSemPost(FuzzyToggleSemaphore);

    		alt_write_byte(LEDR_BASE, LEDSFuzzy);
    		curr = onlyFuzzy;

    		myToggle.brake = false;
    		myToggle.fuzzy = true;
    	}

    	// Disable both
    	else {
    		enable_sonar = false;
    		OSSemPend(FuzzyToggleSemaphore, 0, &err);
    		FuzzyToggle = false;
        	OSSemPost(FuzzyToggleSemaphore);
    		alt_write_byte(LEDR_BASE, LEDSOFFALL);
    		curr = bothOff	;

    		myToggle.brake = false;
    		myToggle.fuzzy = false;
    	}

    	// Send message to log task for off-board notification
    	// When DIP switches change settings only
    	if (prev != curr) {

    		toggleMessage *_t = OSMemGet(StandardMemoryStorage, &err);
    		_t->brake = myToggle.brake;
    		_t->fuzzy = myToggle.fuzzy;
    		OSQPost(LogQueue, CreateLogMessage(TOGGLE_MESSAGE, _t));
    	}

    	prev = curr;
    	// lights will be on if the fuzzy logic is on, and off if fuzzy logic is off
    	OSTimeDlyHMSM(0,0,0,toggleTimeDlyMS);
    }
}
