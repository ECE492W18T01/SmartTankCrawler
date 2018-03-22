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

#include <timer.h>


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
*                                       GLOBAL COMMUNICATION STATE VARIABLES
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
*                                               main()
*
* Description : Entry point for C code.
*
* Arguments   : none.
*
* Returns     : none.
*
* Note(s)     : (1) It is assumed that your code will call main() once you have performed all necessary
*                   initialisation.
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
OS_EVENT *RxDataAvailabeSemaphore;

// Resources
int8_t globalSteeringAngle;
bool motorMask = false; //TO-DO: Keith can you rename this to whatever you want?
char* userMessage;
/* To access steering Angle:
OSSemPend(SteeringSemaphore, 0, &err);
// Do your thing
OSSemPost(SteeringSemaphore);
*/

/* To access mask (In non protected code):
OSSemPend(MaskSemaphore, 0, &err);
// Do your thing
OSSemPost(MaskSemaphore);
*/

#define RX_FIFO_SIZE 127

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
INT8U IncomingMessageMemory[RX_FIFO_SIZE];

OS_MEM *_UserInputStorage;
INT8U UserInputMemory[1][100];

OS_MEM *FuzzyLogicProcessorStorage;
INT8U FuzzyLogicProcessorMemory[5][sizeof(float) * 5];


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



    void *LogMessageArray[100];
    void *MotorMessageArray[10];
    void *FuzzyMessageArray[10];
    void *CollisionMessageArray[10];

    CollisionQueue = OSQCreate(CollisionMessageArray, 10);
    LogQueue = OSQCreate(LogMessageArray, 10);
    MotorQueue = OSQCreate(MotorMessageArray, 10);
    FuzzyQueue = OSQCreate(FuzzyMessageArray, 10);

    SteeringSemaphore = OSSemCreate(1);			// One shared resource
    MaskSemaphore = OSSemCreate(1);				// One shared resource
    CommunicationSemaphore = OSSemCreate(0);	// Turn flag on when interrupt writes
    RxDataAvailabeSemaphore = OSSemCreate(0);	// Turn flag on when interrupt writes

    INT8U err;
    MotorMessageStorage = OSMemCreate(MotorMessageMemory, 10, sizeof(MotorChangeMessage), &err);
    if (err != OS_ERR_NONE) {
    	; /* Handle error. */
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
    _UserInputStorage = OSMemCreate(UserInputMemory, 1, 100, &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
    IncomingMessageStorage = OSMemCreate(IncomingMessageMemory, 1,RX_FIFO_SIZE,&err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }

    userMessage =  OSMemGet(_UserInputStorage, &err);
    if (err != OS_ERR_NONE) {
        ; /* Handle error. */
    }
    FuzzyLogicProcessorStorage =  OSMemGet(FuzzyLogicProcessorMemory, &err);
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
	INT8U err;
	char *TaskName = "EmergencyTask";
	LogMessage *errorMessage;

    for(;;) {OSTimeDlyHMSM(1,0,0,0);} //Task does nothing currently
}

static void CollisionTask (void *p_arg)
{
	INT8U err, send_err;
	char *TaskName = "CollisionTask";
	LogMessage *errorMessage;

	DistanceMessage *incoming;

    for(;;) {

    	incoming = (DistanceMessage*)OSQPend(CollisionQueue, 0, &err);
    	// TO-DO: set data to local variables before freeing message
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


static void MotorTask (void *p_arg)
{
	INT8U err, send_err;
	char *TaskName = "MotorTask";

    MotorChangeMessage *incoming;
    LogMessage *errorMessage;

    float userDriveSpeed;

    int8_t oldUserSteer = 0;
    int8_t newUserSteer, actualSteeringAngle;

    for(;;) {

    	// TO-DO: Replace this with a queue pend, or SOMETHING, to get the new user steering
    	// AS AN INT8_T.
    	newUserSteer = 0;

    	// TO-DO: Mock object in case, Fuzzy Set has no output.
    	// Only pend for so long.
    	incoming = (MotorChangeMessage*)OSQPend(MotorQueue, 0, &err);


    	if ((newUserSteer - oldUserSteer) / 6 == 0) {
    		OSSemPend(SteeringSemaphore, 0, &err);
    		globalSteeringAngle += incoming->steeringServo;
    		actualSteeringAngle = globalSteeringAngle;
    		OSSemPost(SteeringSemaphore);
    	}

    	else {
    		OSSemPend(SteeringSemaphore, 0, &err);
    		globalSteeringAngle = newUserSteer;
    		OSSemPost(SteeringSemaphore);

    		actualSteeringAngle = newUserSteer;
    	}



        //printf("Motor Task: %f, %f, %f, %f, %d\n", incoming->frontLeft, incoming->frontRight,
        //		incoming->backLeft, incoming->backRight, incoming->steeringServo);



		if(err == OS_ERR_NONE) // Message was received
		{
			globalSteeringAngle = incoming->steeringServo;


            driveMotors(0.85, incoming, 0, false);
            //TO-DO: Delete printf
            //printf("Incoming Message to MotorTask:\n fL = %f\n fR = %f\n rL = %f\n rR = %f\n sR = %d\n", frontLeft, frontRight, backLeft, backRight, steeringServo);
            MoveFrontServo(globalSteeringAngle);

            OSMemPut(MotorMessageStorage, incoming);
            /*
			Here's where you can actually do what you want to do
			*/
		} else {
			;//Send to log through function
		}
    }
}

static void FuzzyTask (void *p_arg) {
	INT8U err;
	char *TaskName = "FuzzyTask";

    MotorSpeedMessage *incoming;
    MotorChangeMessage outgoing;
    LogMessage *errorMessage;

	// 'new' Variables, for incoming hall sensor information.
	uint8_t newFrontLeft, newFrontRight, newRearLeft, newRearRight;

	// 'old' Variables, since we take the different in Software
	uint8_t oldFrontLeft = 0;
	uint8_t oldFrontRight = 0;
	uint8_t oldRearLeft = 0;
	uint8_t oldRearRight = 0;
	int8_t localSteeringAngle;

    for(;;) {
     	incoming = (MotorSpeedMessage*)OSQPend(FuzzyQueue, 0, &err);

    	if (err == OS_ERR_NONE) {
            newFrontLeft = incoming->frontLeft;
            newFrontRight = incoming->frontRight;
            newRearLeft = incoming->backLeft;
            newRearRight = incoming->backRight;
            OSMemPut(FuzzyMessageStorage, incoming);

    		OSSemPend(SteeringSemaphore, 0, &err);
    		localSteeringAngle = globalSteeringAngle;
    		OSSemPost(SteeringSemaphore);

            uint8_t wheelSpeeds[4] = {
            	newFrontLeft - oldFrontLeft,
            	newFrontRight - oldFrontRight,
				newRearLeft - oldRearLeft,
				newRearRight - oldRearRight
            };

            // TO-DO Change '0' in the abs to the actual steering angle.
            if (getMinWheelDiff(wheelSpeeds) < speedThres && abs(localSteeringAngle) < lowAngle) {

				outgoing.frontLeft = 0;
				outgoing.frontRight = 0;
				outgoing.backLeft = 0;
				outgoing.backRight = 0;
				outgoing.steeringServo = 0;

            }

            else {

                float *fuzzyOutput = calculateMotorModifiers(wheelSpeeds, localSteeringAngle);

                outgoing.frontLeft = fuzzyOutput[0];
                outgoing.frontRight = fuzzyOutput[1];
                outgoing.backLeft = fuzzyOutput[2];
                outgoing.backRight = fuzzyOutput[3];
                outgoing.steeringServo = fuzzyOutput[4];

                OSMemPut(FuzzyLogicProcessorStorage, fuzzyOutput);

            }

            oldFrontLeft = newFrontLeft;
            oldFrontRight = newFrontRight;
            oldRearLeft = newRearLeft;
            oldRearRight = newRearRight;

    	}

    	else {
    		; // TO-DO Send to log function
        }

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
	char *TaskName = "CommunicationTask";
	char localCopy[100];

	communications_established = false;
	bzero(IncomingMessageStorage, MSG_BUFFER_LEN);

    for(;;) {
    	OSSemPend(RxDataAvailabeSemaphore, 0, &err);
    	char local_char_buffer[RX_FIFO_SIZE];
    	uint32_t chars_read = 0;
    	bool status = read_rx_buffer(local_char_buffer, &chars_read);
    	if(chars_read > 0 && status == true){
    		int incoming_buffer_len = strlen(IncomingMessageStorage);
    		if(incoming_buffer_len + chars_read > RX_FIFO_SIZE){
    			// overflow error
    		}else{
    			strncat(IncomingMessageStorage,local_char_buffer,chars_read);
    			// now determine what to do with the incoming message
    			if(communications_established == true){
    				// parse message if entire message has been received
    				if(complete_message_revived(IncomingMessageStorage) == true){
    					if(look_for_end_byte(IncomingMessageStorage) == true){
    						// termination character received
    						bzero(IncomingMessageStorage, RX_FIFO_SIZE);
    						communications_established = false;
    					}else{
        					incoming_msg *new_msg = parse_incomming_msg(IncomingMessageStorage);
        					if(new_msg != NULL){
        						// valid message received
        						serial_send(ACKNOWLEDGE_STR);
        						bzero(IncomingMessageStorage, RX_FIFO_SIZE);
        						// TO DO: send to Keith
        					}
    					}
    				}
    			}else{
    				//look for start byte
    				communications_established = look_for_start_byte(IncomingMessageStorage, incoming_buffer_len);
    				if(communications_established == true){
    					serial_send(ACKNOWLEDGE_STR);
    					OSSemPost(RxDataAvailabeSemaphore);
    				}else{
    					// garbage, clear incoming buffer
    					bzero(IncomingMessageStorage, RX_FIFO_SIZE);
    				}
    			}
    		}

    	}
    	if (err == OS_ERR_NONE)
    	{
    	strncpy(localCopy, userMessage, 100);
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

    for(;;) {

    	incoming = (LogMessage*)OSQPend(LogQueue, 0, &err);
        //TO-DO: Do something with the error.

    	OSMemPut(LogMessageStorage, incoming);
    }
}
