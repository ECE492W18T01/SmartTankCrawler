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
* Filename      : app.h (used to be app.c)
* Version       : V1.00
* Programmer(s) : JBL
* Modifications	: Nancy Minderman nancy.minderman@ualberta.ca, Brendan Bruner bbruner@ualberta.ca
* 				  Changes to this project include scatter file changes and BSP changes for port from
* 				  Cyclone V dev kit board to DE1-SoC
*
*				  Keith Mills kgmills@ualberta.ca
*				  Reworked for project, testing of components, for usage on the DE10-Nano.
*
*				  Joshua Robertson jcrobert@ualberta.ca
*				  Further reworked to add our necessary tasks, queues, and other data structures.
*********************************************************************************************************
* Note(s)       : none.
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
#define EMERGENCY_TASK_PRIORITY 5
#define COLLISION_TASK_PRIO 7
#define MOTOR_TASK_PRIO 8
#define FUZZY_TASK_PRIO 9
#define COMMUNICATION_TASK_PRIO 10
#define LOG_TASK_PRIO 11
#define TOGGLE_TASK_PRIO 12           // maybe change later to higher prio, if necessary

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
CPU_STK EmgTaskStk[TASK_STACK_SIZE];
CPU_STK ColTaskStk[TASK_STACK_SIZE];
CPU_STK MotorTaskStk[TASK_STACK_SIZE];
CPU_STK FuzzyTaskStk[TASK_STACK_SIZE];
CPU_STK ComTaskStk[TASK_STACK_SIZE];
CPU_STK LogTaskStk[TASK_STACK_SIZE];
CPU_STK ToggleTaskStk[TASK_STACK_SIZE];

/*
*********************************************************************************************************
*                                 GLOBAL COMMUNICATION STATE VARIABLES
*********************************************************************************************************
*/

static bool communications_established = false;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart              (void        *p_arg);
static  void  EmergencyTask             (void        *p_arg);
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
circular_buf_uint8_t* distance_buffer;
circular_buf_uint8_t* velocity_buffer;

/* To access steering Angle:
OSSemPend(SteeringSemaphore, 0, &err);
 * Example:
 *
 * globalSteeringAngle = newSteeringAngle;
 *
OSSemPost(SteeringSemaphore);
*/

/* To access mask (In non protected code):
OSSemPend(MaskSemaphore, 0, &err);
 *
 * Example:
 *
 * if (motorMask) NewMotorSpeed = OldMotorSpeed*Change;
 * else NewMotorSpeed = 0;
 *
OSSemPost(MaskSemaphore);
*/


/* Memory:
 * --Name--Storage:
 * 		The object you use to get and put back memory blocks.
 * 		Standard is for any inter-task data
 * 		Large is for messages from and to the Pi.
 * --Name--Memory:
 * 		The actual memory block.
 * 		This should only ever be touched once, to create the memory block.
 * 		The first number is the amount of blocks to create.
 * 		The second number is the size of each block.
*/

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

    // Set the global steering angle to 0 on start.
    globalSteeringAngle = 0;
    MoveBackServo(BackServoMax);
    userSteer = 0;
    userMag = 0;
    //MoveFrontServo(globalSteeringAngle);
    //MoveBackServo(BackServoMax);

    //Array of pointers that the Queues will use. Should never be touched again.
    void *LogMessageArray[100];
    void *MotorMessageArray[10];
    void *FuzzyMessageArray[10];
    void *CollisionMessageArray[10];

    // Create the Queues using above arrays to save message pointers.
    CollisionQueue = OSQCreate(CollisionMessageArray, 10);
    LogQueue = OSQCreate(LogMessageArray, 10);
    MotorQueue = OSQCreate(MotorMessageArray, 10);
    FuzzyQueue = OSQCreate(FuzzyMessageArray, 10);
    /* Create the semaphores
     * Zero means that the semaphore will be used as a flag
     * NonZero means the semaphore will be used to control access to shared resource(s).
     */
    UserSemaphore = OSSemCreate(1);
    SteeringSemaphore = OSSemCreate(1);			// One shared resource
    MaskSemaphore = OSSemCreate(1);				// One shared resource
    CommunicationSemaphore = OSSemCreate(0);	// Turn flag on when interrupt writes
    RxDataAvailableSemaphore = OSSemCreate(0);	// Turn flag on when interrupt writes
    SonarDataAvailableSemaphore = OSSemCreate(0);// Turn flag on when interrupt writes
    FuzzyToggleSemaphore = OSSemCreate(1);      // One Shared resource

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

    distance_buffer = (circular_buf_uint8_t*)OSMemGet(LargeMemoryStorage, &err);
    circular_buf_init(distance_buffer);

    velocity_buffer = (circular_buf_uint8_t*)OSMemGet(LargeMemoryStorage, &err);
    circular_buf_init(velocity_buffer);

    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
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
    os_err = OSTaskCreateExt((void (*)(void *)) EmergencyTask,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&EmgTaskStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) EMERGENCY_TASK_PRIORITY,
                             (INT16U          ) EMERGENCY_TASK_PRIORITY,  // reuse prio for ID
                             (OS_STK        * )&EmgTaskStk[0],
                             (INT32U          ) TASK_STACK_SIZE,
                             (void          * )0,
                             (INT16U          )(OS_TASK_OPT_STK_CLR | OS_TASK_OPT_STK_CHK));

    if (os_err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
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
* Description : Startup task example code.
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
* Notes       : (1) The ticker MUST be initialised AFTER multitasking has started.
*********************************************************************************************************
*/

// TODO externalize below functions
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

static void EmergencyTask (void *p_arg)
{
//	INT8U err;
//	char *TaskName = "EmergencyTask";
//	LogMessage *errorMessage;
	//int8_t x = 64;
    for(;;) {
    	OSTimeDlyHMSM(0,0,1,0);
//    	MoveFrontServo(x);
//    	x--;
//    	if (x < -64) {
//    		x = 64;
//    	}
    } //Task does nothing currently
}


#define DISTANCE_SAMPLES 20
#define SAMPLE_OFFSET 6

static void CollisionTask (void *p_arg)
{
	INT8U err;

    for(;;) {

		OSSemPend(SonarDataAvailableSemaphore, 0, &err);
		CPU_CRITICAL_ENTER();
		uint8_t last_n_distances[DISTANCE_SAMPLES] = {0};
		for(int i = 0; i < DISTANCE_SAMPLES; i++ ){
			 circular_buffer_get_nth(distance_buffer, &last_n_distances[i], i);
		}
		CPU_CRITICAL_EXIT();
		uint8_t latest_distance_sample = 0;
		uint8_t offset_distance_sample = 0;

		// find the most recent valid distance
		// (actually current - 1)
		int latest_index = 1;
		int offset_index = latest_index + SAMPLE_OFFSET;

		while(offset_index < (DISTANCE_SAMPLES - SAMPLE_OFFSET -1)){
			if(sample_window_validator(last_n_distances[latest_index -1],last_n_distances[latest_index],last_n_distances[latest_index + 1]) &&
					sample_window_validator(last_n_distances[offset_index -1],last_n_distances[offset_index],last_n_distances[offset_index + 1])){
					// valid sample pair found
				latest_distance_sample = sample_window_avg(last_n_distances[latest_index -1], last_n_distances[latest_index], last_n_distances[latest_index + 1]);
				offset_distance_sample = sample_window_avg(last_n_distances[offset_index -1], last_n_distances[offset_index], last_n_distances[offset_index + 1]);
				break;
			}else{
				latest_index++;
				offset_index++;

			}
		}

		// calculate the delta between current and offset
		int position_delta =  offset_distance_sample - latest_distance_sample;

		// if change is too small disregard
		if(abs(position_delta) < MIN_DETECTABLE_CHANGE){
			position_delta = 0;
		}

		// velocity in inches per second
		float velocity = (float) position_delta * (1/(SONAR_INTERUPT_PERIOD * SAMPLE_OFFSET));

		//printf("Distance: %i, Delta: %i ,Velocity: %.6f\n", latest_distance_sample, position_delta, velocity);
		if (err == OS_ERR_NONE)
		{
			/*
			 * You have received a distance measurement. TODO: Implement
			 */
		} else {
			OSQPost(LogQueue, CreateErrorMessage(COLLISION_TASK, OS_Q_PEND, err));
        }
    }
}

/*
 * Motor Task
 * Responsible for accruing information from other tasks and sending output
 * instructions to the PWMs for the H_bridges and steering servo.
 */
static void MotorTask (void *p_arg)
{
	INT8U err; //, send_err;
//	char *TaskName = "MotorTask";

	// Incoming from Fuzzy Task, staticFuzzy is local and static allocated
    MotorChangeMessage *incoming;
    MotorChangeMessage staticFuzzy = { 0 };
    float *indMotorVals = malloc(sizeof(float) * 4);
//    LogMessage *errorMessage;

    float userDriveSpeed = 0;

    // This should not be changes; it should be initalized to zero and then changed later.
    // This is NOT the global emergency brake variable, but later in the code it is assigned that variable's value.
    bool allStop = false;

    // Old steering, new steering, and the actual value to be plugged.
    int8_t oldUserSteer, newUserSteer, actualSteeringAngle = 0;
    MoveFrontServo(actualSteeringAngle);

    // Loop forever.
    for(;;) {

		OSSemPend(UserSemaphore, 0, &err);
		newUserSteer = userSteer;
		userDriveSpeed = userMag;
		OSSemPost(UserSemaphore);

    	// Only pend for so long - Change timeout value to be define, but the number now is 1/4 of a second.
    	// Subject to change alongside timeouts for other queues.
    	incoming = (MotorChangeMessage*)OSQPend(MotorQueue, OS_TICKS_PER_SEC / 4, &err);

    	// We timed out, set the static struct's parameters to zero. Aka zero fuzzy mods.
    	if (err == OS_ERR_TIMEOUT) {
    		// And let us know that everything is fine.
    		err = OS_ERR_NONE;
    	}

    	// We got info from the Fuzzy Task, so assign it to the static task.
    	else {
    		staticFuzzy.frontLeft = incoming->frontLeft;
    		staticFuzzy.frontRight = incoming->frontRight;
    		staticFuzzy.backLeft = incoming->backLeft;
    		staticFuzzy.backRight = incoming->backRight;
    		staticFuzzy.steeringServo = incoming->steeringServo;

    		// Free Fuzzy Task struct memory
    		// TODOPotential source of error
    		OSQPost(LogQueue, CreateLogMessage(MOTOR_CHANGE_MESSAGE, incoming));
    		//OSMemPut(StandardMemoryStorage, incoming);
    	}

		if(err == OS_ERR_NONE) {

			// If the user wants the servo "moved", don't listen to the fuzzy task.
			if ((newUserSteer - oldUserSteer) / 6 == 0) {
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


			driveMotors(userDriveSpeed, &staticFuzzy, indMotorVals, actualSteeringAngle, allStop);
			// Move the servo, then the motors. Should probably protect this with OS_ENTER_CRITICAL...?
			OS_ENTER_CRITICAL();
			MoveFrontServo(actualSteeringAngle);
			update_motor_control(indMotorVals[0], FRONT_LEFT_MOTOR);
			update_motor_control(indMotorVals[1], FRONT_RIGHT_MOTOR);
			update_motor_control(indMotorVals[2], REAR_LEFT_MOTOR);
			update_motor_control(indMotorVals[3], REAR_RIGHT_MOTOR);
			OS_EXIT_CRITICAL();

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

/**
 * Fuzzy Task
 * Gets information from Hall Sensors at a 2Hz rate.
 * Processes information to determine relative speeds.
 * Accesses global variable on steering for lookup table, does not modify though.
 * Uses Fuzzy Logic LUTs to determine fuzzy modifiers.
 * Sends to Motor Task.
 */
static void FuzzyTask (void *p_arg) {
	INT8U err;
//	char *TaskName = "FuzzyTask";

	// First struct from Hall Sensor Interrupt.
	// Second to Motor Task.
	// Third to log task - if necessary.
    HallSensorMessage *incoming;
    MotorChangeMessage outgoing;
//    LogMessage *errorMessage;

	// 'new' Variables, for incoming hall sensor information.
	uint8_t newFrontLeft, newFrontRight, newRearLeft, newRearRight;

	// 'old' Variables, since we take the different in Software
	uint8_t oldFrontLeft = 0;
	uint8_t oldFrontRight = 0;
	uint8_t oldRearLeft = 0;
	uint8_t oldRearRight = 0;

	// Initialize the wheelspeeds.
	// TODO potential source of problem.
	uint8_t wheelSpeeds[4] = {0};

	// Local variable for steering, we only access the semaphore once.
	int8_t localSteeringAngle = 0;

	// Loop forever.
    for(;;) {

    	// Await incoming Hall Sensor information at 2Hz.
     	incoming = (HallSensorMessage*)OSQPend(FuzzyQueue, 0, &err);


     	// Assuming nothing bad happened, get new hall sensor information.
    	if (err == OS_ERR_NONE) {
            newFrontLeft = incoming->frontLeft;
            newFrontRight = incoming->frontRight;
            newRearLeft = incoming->backLeft;
            newRearRight = incoming->backRight;

            // And 'free' the memory.
            OSMemPut(StandardMemoryStorage, incoming);

            // Get the steering angle.
    		OSSemPend(SteeringSemaphore, 0, &err);
    		localSteeringAngle = globalSteeringAngle;
    		OSSemPost(SteeringSemaphore);

    		// Calculate relative speeds, new - old.
            wheelSpeeds[0] = newFrontLeft - oldFrontLeft;
            wheelSpeeds[1] = newFrontRight - oldFrontRight;
            wheelSpeeds[2] = newRearLeft - oldRearLeft;
            wheelSpeeds[3] = newRearRight - oldRearRight;

            // Decide whether or not we even want to access the Fuzzy Logic Matrix.
            if ((getMinWheelDiff(wheelSpeeds) < speedThres && abs(localSteeringAngle) < lowAngle) || !FuzzyToggle) {

            	// Assign zero to all modifiers.
				outgoing.frontLeft = 0;
				outgoing.frontRight = 0;
				outgoing.backLeft = 0;
				outgoing.backRight = 0;
				outgoing.steeringServo = 0;

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

                printf("%f, %f, %f, %f, %f\n", fuzzyOutput[0],
                		fuzzyOutput[1],
						fuzzyOutput[2],
						fuzzyOutput[3],
						fuzzyOutput[4]);


                // Put the temporary memory borrowed by calculateMotorModifiers back.
                OSMemPut(StandardMemoryStorage, fuzzyOutput);

            }

            // Old is now new.
            oldFrontLeft = newFrontLeft;
            oldFrontRight = newFrontRight;
            oldRearLeft = newRearLeft;
            oldRearRight = newRearRight;

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

static void CommunicationTask (void *p_arg)
{
	INT8U err;
//	char *TaskName = "CommunicationTask";
	char localCopy[100];

	communications_established = false;
	//Get memory IncomingMessageStorage-->incomingMessage
	bzero(userMessage, MSG_BUFFER_LEN);

    for(;;) {

    	OSSemPend(RxDataAvailableSemaphore, 0, &err);

    	if (err == OS_ERR_TIMEOUT) {
    		err = OS_ERR_NONE;
    		// set motor values to 0
			incoming_msg *outgoing = OSMemGet(StandardMemoryStorage, &err);
			OSSemPend(UserSemaphore, 0, &err);
			userSteer = MOTOR_ZERO;
			userMag = STEERING_ZERO;
			OSSemPost(UserSemaphore);
    	}else{
    		// now determine what to do with the incoming message
    		if(communications_established == true){
    			// look for end byte
    			if(look_for_end_byte(userMessage) == true){
					// termination character received
					bzero(userMessage, RX_FIFO_SIZE);
					communications_established = false;
    			}else if(complete_message_revived(userMessage) == true){
					//TODO marker for josh grab this and send to c
					incoming_msg new_msg = parse_incomming_msg(userMessage);
					// valid message received
					serial_send(ACKNOWLEDGE_STR);
					bzero(userMessage, RX_FIFO_SIZE);

					// Post to Semaphore
					OSSemPend(UserSemaphore, 0, &err);
					userSteer = new_msg.steering_value;
					userMag = new_msg.motor_level;
					OSSemPost(UserSemaphore);
    			}
    		}else{
    			//look for start byte
    			communications_established = look_for_start_byte(userMessage, strlen(userMessage));
    			if(communications_established == true){
    				OSTimeDlyHMSM(0, 0, 0, 500); // delay
    				serial_send(ACKNOWLEDGE_STR);
    				OSSemPost(RxDataAvailableSemaphore);
    			}else{
    				// garbage, clear incoming buffer
    				bzero(userMessage, RX_FIFO_SIZE);
    			}
    		}
    	}
    	if (err == OS_ERR_NONE)
    	{
    	//strncpy(localCopy, userMessage, 100);
    		// First thing copy to a local version so you're not working off a global variable

    		// I just copy the whole 100 characters. I'm assuming you'll having a parsing scheme
    		// TO-DO: Parse

    	} else {
    		; //Send to log through function
    	}
    }
}


static void LogTask (void *p_arg)
{
	INT8U err;
	LogMessage *incoming;
	void *message;
	char *outgoing;

    for(;;) {
    	incoming = (LogMessage*)OSQPend(LogQueue, 0, &err);
    	outgoing = OSMemGet(LargeMemoryStorage, &err);

    	if (incoming->error == OS_ERR_NONE) {
    		// This is a standard message


    		switch (incoming->messageType) {

    		// This is the Hall Sensor Differences. E.g. number of edges per wheel in the interval.
    		case HALL_SENSOR_MESSAGE:
    			message = (HallSensorMessage*)(incoming->message);
    			// Send the details:
    			sprintf(outgoing + strlen(outgoing), "%s{ ", MESSAGE_START_STR );
    			sprintf(outgoing + strlen(outgoing), "srv:%d, ", HALL_SENSOR_MESSAGE);
    			sprintf(outgoing + strlen(outgoing), "fl:%d, ", ((HallSensorMessage*)message)->frontLeft);
    			sprintf(outgoing + strlen(outgoing), "fr:%d, ", ((HallSensorMessage*)message)->frontRight);
    			sprintf(outgoing + strlen(outgoing), "bl:%d, ", ((HallSensorMessage*)message)->backLeft);
    			sprintf(outgoing + strlen(outgoing), "br:%d ", ((HallSensorMessage*)message)->backRight);
    			sprintf(outgoing + strlen(outgoing), "}%s", MESSAGE_END_STR);
    			serial_send(outgoing);
    			break;

    		// This is the Fuzzy Set output.
    		case MOTOR_CHANGE_MESSAGE:

    	    	message = (MotorChangeMessage*)(incoming->message);
    	    	// Send the details:
//    	    	((MotorChangeMessage*)message)->frontLeft;
//    	    	((MotorChangeMessage*)message)->frontRight;
//    	    	((MotorChangeMessage*)message)->backLeft;
//    	    	((MotorChangeMessage*)message)->backRight;
//    	    	((MotorChangeMessage*)message)->steeringServo;
    			break;

    		// These are the values being sent to the FPGA.
    		case MOTOR_OUTPUT_MESSAGE:
    			message = (MotorChangeMessage*)(incoming->message);

    			break;

    		case DISTANCE_MESSAGE:
    			//message = (DistanceMessage*)(incoming->message);
    			// Send the details:
//    			((DistanceMessage*)message)->distance;
    			break;

    		default:
    			// Should never get here...
    			break;
    		}
    	} else {
    		// This is an error message

    		// Send the details:
//    		incoming->taskID;		// task name is where there was an error thrown
//    		incoming->sourceID;	// source name is the source of the function that threw the error
//    		incoming->error;		// error is the error code thrown

    	};
    	OSMemPut(StandardMemoryStorage, incoming->message);
    	OSMemPut(StandardMemoryStorage, incoming);
    	OSMemPut(StandardMemoryStorage, outgoing);
    }


}

static void ToggleTask(void *p_arg)
{

	INT8U err; //, send_err;
	bool EnableFuzzy = alt_read_byte(SW_BASE) % 2;

    for(;;) {

    	// lights will be on if the fuzzy logic is on, and off if fuzzy logic is off
    	OSTimeDlyHMSM(0,0,0,50);
    	if(EnableFuzzy == 0){
    		alt_write_byte(LEDR_BASE, 0xff);
    		OSSemPend(FuzzyToggleSemaphore, 0, &err);
    		FuzzyToggle = true;
    		OSSemPost(FuzzyToggleSemaphore);
    	}
    	else{
    		alt_write_byte(LEDR_BASE, 0x00);
    		OSSemPend(FuzzyToggleSemaphore, 0, &err);
    	    FuzzyToggle = false;
    	    OSSemPost(FuzzyToggleSemaphore);
    	}
    	EnableFuzzy = alt_read_word(SW_BASE);


    }
}
