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
#define RX_FIFO_SIZE 127

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
 * --Name--Memory:
 * 		The actual memory block.
 * 		This should only ever be touched once, to create the memory block.
 * 		The first number is the amount of blocks to create.
 * 		The second number is the size of each block. These --should-- be the size of a word. I'll fix later.
*/

OS_MEM *MotorMessageStorage;
INT8U MotorMessageMemory[10][sizeof(MotorChangeMessage)];

OS_MEM *FuzzyMessageStorage;
INT8U FuzzyMessageMemory[10][sizeof(MotorSpeedMessage)];

OS_MEM *LogMessageStorage;
INT8U LogMessageMemory[100][sizeof(LogMessage)];

OS_MEM *DistanceMessageStorage;
INT8U DistanceMessageMemory[10][sizeof(DistanceMessage)];

OS_MEM *StatusMessageStorage;
INT8U StatusMessageMemory[100][sizeof(StatusMessage)];

OS_MEM *MessageStorage;
INT8U MessageMemory[100][100]; //100 is a 100 character string

OS_MEM *IncomingMessageStorage;
INT8U IncomingMessageMemory[1][RX_FIFO_SIZE];

OS_MEM *FuzzyLogicProcessorStorage;
INT8U FuzzyLogicProcessorMemory[5][sizeof(float) * 5];

OS_MEM *UserInputStorage;
INT8U UserInputMemory[5][sizeof(incoming_msg)];

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
    // TODO get rid of all the magic numbers, I'll do this later as its low priority (Josh)
    // Initialize the Memory.
    INT8U err;
    MotorMessageStorage = OSMemCreate(MotorMessageMemory, 10, sizeof(MotorChangeMessage), &err);
    if (err != OS_ERR_NONE) {
    	; /* Handle error. TODO what's the best way to do this?...*/
    }
    FuzzyMessageStorage = OSMemCreate(FuzzyMessageMemory, 10, sizeof(MotorSpeedMessage), &err);
    if (err != OS_ERR_NONE) {
    	; /* Handle error. */
    }
    LogMessageStorage = OSMemCreate(LogMessageMemory, 100, sizeof(LogMessage), &err);
    if (err != OS_ERR_NONE) {
    	; /* Handle error. */
    }
    DistanceMessageStorage = OSMemCreate(DistanceMessageMemory, 10, sizeof(DistanceMessage), &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
    StatusMessageStorage = OSMemCreate(StatusMessageMemory, 100, sizeof(StatusMessage), &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
    MessageStorage = OSMemCreate(MessageMemory, 100, 100, &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
    UserInputStorage = OSMemCreate(UserInputMemory, 5, sizeof(incoming_msg), &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
    IncomingMessageStorage = OSMemCreate(IncomingMessageMemory, 1,RX_FIFO_SIZE,&err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
    FuzzyLogicProcessorStorage = OSMemCreate(FuzzyLogicProcessorMemory, 5,sizeof(float) * 5,&err);
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

static  void  AppTaskStart (void *p_arg)
{

    BSP_OS_TmrTickInit(OS_TICKS_PER_SEC);                       /* Configure and enable OS tick interrupt.              */
    InitHallSensorInterrupt();
    InitDistanceSensorInterrupt();
    InitCommunicationInterrupt();


    for(;;) {
        BSP_WatchDog_Reset();
        OSTimeDlyHMSM(0, 0, 1, 0);
    }
}

//This is going to be made into a function:
//			OSSemPend(logMessageStorageAccess, 10, &send_err);
//			errorMessage = OSMemGet(logMessageStorage, &send_err);
//			errorMessage = malloc(sizeof(LogMessage));
//			errorMessage->_taskName = TaskName;
//			errorMessage->_sourceName = "OSQPend";
//			errorMessage->_error = err;
//		    OSQPost(LogQueue, errorMessage);
//		}
//

static void EmergencyTask (void *p_arg)
{
//	INT8U err;
//	char *TaskName = "EmergencyTask";
//	LogMessage *errorMessage;

    for(;;) {OSTimeDlyHMSM(1,0,0,0);} //Task does nothing currently
}

static void CollisionTask (void *p_arg)
{
	INT8U err; //, send_err;
//	char *TaskName = "CollisionTask";
//	LogMessage *errorMessage;

	DistanceMessage *incoming;

    for(;;) {

    	incoming = (DistanceMessage*)OSQPend(CollisionQueue, 0, &err);
    	// TODO: set data to local variables before freeing message
    	OSMemPut(DistanceMessageStorage, incoming);

		if (err == OS_ERR_NONE)
		{
			/*
			 * You have received a distance measurement. TODO: Implement
			 */
		} else {
			;//Send to log through function
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
    		OSMemPut(MotorMessageStorage, incoming);
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
			;//Send to log through function
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
    MotorSpeedMessage *incoming;
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
     	incoming = (MotorSpeedMessage*)OSQPend(FuzzyQueue, 0, &err);

     	// Assuming nothing bad happened, get new hall sensor information.
    	if (err == OS_ERR_NONE) {
            newFrontLeft = incoming->frontLeft;
            newFrontRight = incoming->frontRight;
            newRearLeft = incoming->backLeft;
            newRearRight = incoming->backRight;

            // And 'free' the memory.
            OSMemPut(FuzzyMessageStorage, incoming);

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
                OSMemPut(FuzzyLogicProcessorStorage, fuzzyOutput);

            }

            // Old is now new.
            oldFrontLeft = newFrontLeft;
            oldFrontRight = newFrontRight;
            oldRearLeft = newRearLeft;
            oldRearRight = newRearRight;

    	}

    	else {
    		; // TODO Send to log function
        }

    	// Send the message off.
    	MotorChangeMessage *_;
    	if (err == OS_ERR_NONE) _ = OSMemGet(MotorMessageStorage, &err);
    	if (err == OS_ERR_NONE)
    	{
    		_->frontLeft = outgoing.frontLeft;
    		_->frontRight = outgoing.frontRight;
    		_->backLeft = outgoing.backLeft;
    		_->backRight = outgoing.backRight;
    		_->steeringServo = outgoing.steeringServo;

    		OSQPost(MotorQueue, _);
    	} else {
    		; //Send to log through function
        }
    }
}

static void CommunicationTask (void *p_arg)
{
	INT8U err;
//	char *TaskName = "CommunicationTask";

	communications_established = false;
	char *incomingMessage = OSMemGet(IncomingMessageStorage, &err);
	if (err != OS_ERR_NONE) /*TODO this should probably be dealt with*/ {;}
	bzero(IncomingMessageStorage, MSG_BUFFER_LEN);

    for(;;) {
    	OSSemPend(RxDataAvailableSemaphore, 0, &err);
    	char local_char_buffer[RX_FIFO_SIZE];
    	uint32_t chars_read = 0;
    	bool status = read_rx_buffer(local_char_buffer, &chars_read);
    	if(chars_read > 0 && status == true){
    		int incoming_buffer_len = strlen(incomingMessage);
    		if(incoming_buffer_len + chars_read > RX_FIFO_SIZE){
    			// overflow error
    		}else{
    			strncat(incomingMessage,local_char_buffer,chars_read);
    			// now determine what to do with the incoming message
    			if(communications_established == true){
    				// parse message if entire message has been received
    				if(complete_message_revived(incomingMessage) == true){
    					if(look_for_end_byte(incomingMessage) == true){
    						// termination character received
    						bzero(IncomingMessageStorage, RX_FIFO_SIZE);
    						communications_established = false;
    					}else{
    						//TODO marker for josh grab this and send to c
        					incoming_msg *new_msg = parse_incomming_msg(incomingMessage);
        					if(new_msg != NULL){
        						// valid message received
        						serial_send(ACKNOWLEDGE_STR);
        						bzero(IncomingMessageStorage, RX_FIFO_SIZE);

        						incoming_msg *outgoing = OSMemGet(UserInputStorage, &err);
        						outgoing->motor_level = new_msg->motor_level;
								outgoing->steering_value = new_msg->steering_value;
								OSQPost(InputQueue, outgoing);

								// TODO: Here you go Keith
								/*
								incoming_msg *incoming;
								(incoming_msg*)OSQPend(InputQueue, 0, &err);
								float temp1 = incoming->motor_level;
								int8_t temp2 = incoming->steering_value;
								OSMemPut(UserInputStorage, incoming);
								*/
        					}
    					}
    				}
    			}else{
    				//look for start byte
    				communications_established = look_for_start_byte(incomingMessage, incoming_buffer_len);
    				if(communications_established == true){
    					serial_send(ACKNOWLEDGE_STR);
    					OSSemPost(RxDataAvailableSemaphore);
    				}else{
    					// garbage, clear incoming buffer
    					bzero(IncomingMessageStorage, RX_FIFO_SIZE);
    				}
    			}
    		}
    	}
    }
}

static void LogTask (void *p_arg)
{
	INT8U err;
	LogMessage *incoming;

    for(;;) {

    	incoming = (LogMessage*)OSQPend(LogQueue, 0, &err);
        //TODO: Do something with the error.

    	OSMemPut(LogMessageStorage, incoming);
    }
}
