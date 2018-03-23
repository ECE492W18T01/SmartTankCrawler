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

/*
*********************************************************************************************************
*                              Priority Definitions and Task Stack Size
*********************************************************************************************************
*/
#define APP_TASK_PRIO 5
#define EMERGENCY_TASK_PRIORITY 6
#define COLLISION_TASK_PRIO 7
#define MOTOR_TASK_PRIO 8
#define FUZZY_TASK_PRIO 9
#define COMMUNICATION_TASK_PRIO 10
#define LOG_TASK_PRIO 11

#define TASK_STACK_SIZE 4096

/*
*********************************************************************************************************
*                                            Communication Size
*********************************************************************************************************
*/
// actually only 127
#define RX_FIFO_SIZE 100

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
OS_EVENT *InputQueue;

// Semaphores
OS_EVENT *SteeringSemaphore;
OS_EVENT *MaskSemaphore;
OS_EVENT *CommunicationSemaphore;
OS_EVENT *RxDataAvailableSemaphore;

// Resources
int8_t globalSteeringAngle;
bool motorMask = false;
char* userMessage;

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

    //Array of pointers that the Queues will use. Should never be touched again.
    void *LogMessageArray[100];
    void *MotorMessageArray[10];
    void *FuzzyMessageArray[10];
    void *CollisionMessageArray[10];
    void *InputMessageArray[10];

    // Create the Queues using above arrays to save message pointers.
    CollisionQueue = OSQCreate(CollisionMessageArray, 10);
    LogQueue = OSQCreate(LogMessageArray, 10);
    MotorQueue = OSQCreate(MotorMessageArray, 10);
    FuzzyQueue = OSQCreate(FuzzyMessageArray, 10);
    InputQueue = OSQCreate(InputMessageArray, 10);
    /* Create the semaphores
     * Zero means that the semaphore will be used as a flag
     * NonZero means the semaphore will be used to control access to shared resource(s).
     */
    SteeringSemaphore = OSSemCreate(1);			// One shared resource
    MaskSemaphore = OSSemCreate(1);				// One shared resource
    CommunicationSemaphore = OSSemCreate(0);	// Turn flag on when interrupt writes
    RxDataAvailableSemaphore = OSSemCreate(0);	// Turn flag on when interrupt writes
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

    for(;;) {OSTimeDlyHMSM(1,0,0,0);} //Task does nothing currently
}

static void CollisionTask (void *p_arg)
{
	INT8U err;

	DistanceMessage *incoming;

    for(;;) {

    	incoming = (DistanceMessage*)OSQPend(CollisionQueue, 0, &err);
    	// TODO: set data to local variables before freeing message
    	OSMemPut(StandardMemoryStorage, incoming);

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
//    LogMessage *errorMessage;

    // TODO assign this as something from the communication task.
    float userDriveSpeed = 0.85;

    // This should not be changes; it should be initalized to zero and then changed later.
    // This is NOT the global emergency brake variable, but later in the code it is assigned that variable's value.
    bool allStop = false;

    // Old steering, new steering, and the actual value to be plugged.
    int8_t oldUserSteer, newUserSteer, actualSteeringAngle = 0;

    // Loop forever.
    for(;;) {

    	// TODO: Replace this with a queue pend, or SOMETHING, to get the new user steering
    	// AS AN INT8_T.
    	newUserSteer = 0;

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
    		OSMemPut(StandardMemoryStorage, incoming);
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

			// Move the servo, then the motors. Should probably protect this with OS_ENTER_CRITICAL...? TODO
			MoveFrontServo(actualSteeringAngle);
			driveMotors(userDriveSpeed, &staticFuzzy, actualSteeringAngle, allStop);

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
            uint8_t wheelSpeeds[4] = {
            	newFrontLeft - oldFrontLeft,
            	newFrontRight - oldFrontRight,
				newRearLeft - oldRearLeft,
				newRearRight - oldRearRight
            };

            // Decide whether or not we even want to access the Fuzzy Logic Matrix.
            if (getMinWheelDiff(wheelSpeeds) < speedThres && abs(localSteeringAngle) < lowAngle) { // TODO Switch flipper

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

                // Put the temporary memory borrowed by calculateMotorModifiers back.
                OSMemPut(StandardMemoryStorage, fuzzyOutput);

            }

            // Old is now new.
            oldFrontLeft = newFrontLeft;
            oldFrontRight = newFrontRight;
            oldRearLeft = newRearLeft;
            oldRearRight = newRearRight;

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
    	// TODO: set motor values to 0 on timeout
    	OSSemPend(RxDataAvailableSemaphore, 0, &err);
		// now determine what to do with the incoming message
		if(communications_established == true){
			// parse message if entire message has been received
			if(complete_message_revived(userMessage) == true){
				if(look_for_end_byte(userMessage) == true){
					// termination character received
					bzero(userMessage, RX_FIFO_SIZE);
					communications_established = false;
				}else{
					//TODO marker for josh grab this and send to c
					incoming_msg new_msg = parse_incomming_msg(userMessage);
					// valid message received
					serial_send(ACKNOWLEDGE_STR);
					bzero(userMessage, RX_FIFO_SIZE);
				}
			}
		}else{
			//look for start byte
			communications_established = look_for_start_byte(userMessage, strlen(userMessage));
			if(communications_established == true){
				serial_send(ACKNOWLEDGE_STR);
				OSSemPost(RxDataAvailableSemaphore);
			}else{
				// garbage, clear incoming buffer
				bzero(userMessage, RX_FIFO_SIZE);
			}
		}
    }
}

static void LogTask (void *p_arg)
{
	INT8U err;
	LogMessage *incoming;
	void *message;
    for(;;) {

    	incoming = (LogMessage*)OSQPend(LogQueue, 0, &err);


    	if (incoming->error == OS_ERR_NONE) {
    		// This is a standard message
    		switch (incoming->messageType) {
    		case HALL_SENSOR_MESSAGE:

    			message = (HallSensorMessage*)(incoming->message);
    			// Send the details:
    			((HallSensorMessage*)message)->frontLeft;
    			((HallSensorMessage*)message)->frontRight;
    			((HallSensorMessage*)message)->backLeft;
				((HallSensorMessage*)message)->backRight;
    			break;

    		case MOTOR_CHANGE_MESSAGE:

    	    	message = (MotorChangeMessage*)(incoming->message);
    	    	// Send the details:
    	    	((MotorChangeMessage*)message)->frontLeft;
    	    	((MotorChangeMessage*)message)->frontRight;
    	    	((MotorChangeMessage*)message)->backLeft;
    	    	((MotorChangeMessage*)message)->backRight;
    	    	((MotorChangeMessage*)message)->steeringServo;
    			break;

    		case DISTANCE_MESSAGE:
    			message = (DistanceMessage*)(incoming->message);
    			// Send the details:
    			((DistanceMessage*)message)->distance;
    			break;

    		default:
    			// Should never get here...
    			break;
    		}
    	} else {
    		// This is an error message

    		// Send the details:
    		incoming->taskID;		// task name is where there was an error thrown
    		incoming->sourceID;	// source name is the source of the function that threw the error
    		incoming->error;		// error is the error code thrown

    	};
    	OSMemPut(StandardMemoryStorage, incoming);
    }
}
