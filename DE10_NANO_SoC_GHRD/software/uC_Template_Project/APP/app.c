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

#include "motor_control.h"
#include "wrap.h"

#define APP_TASK_PRIO 5
#define TASK_STACK_SIZE 4096

#define FUZZY_TASK_PRIO 10
#define MOTOR_TASK_PRIO 15
#define ERROR_TASK_PRIO 20

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

CPU_STK AppTaskStartStk[TASK_STACK_SIZE];
CPU_STK MotorTaskStk[TASK_STACK_SIZE];
CPU_STK FuzzyTaskStk[TASK_STACK_SIZE];
CPU_STK ErrorTaskStk[TASK_STACK_SIZE];


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart              (void        *p_arg);
static  void  AppTaskStart              (void        *p_arg);
static  void  MotorTask                 (void        *p_arg);
static  void  FuzzyTask                 (void        *p_arg);
static  void  ErrorTask                 (void        *p_arg);


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
OS_EVENT *ErrorQueue;
OS_EVENT *MotorQueue;
OS_EVENT *FuzzyQueue;

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

    void *ErrorMessageArray[100];
    void *MotorMessageArray[10];
    void *FuzzyMessageArray[10];

    //TODO: I'm pretty sure the below address is wrong. Need to test.
    ErrorQueue = OSQCreate(&ErrorMessageArray[0], 10);
    MotorQueue = OSQCreate(&MotorMessageArray[0], 10);
    FuzzyQueue = OSQCreate(&FuzzyMessageArray[0], 10);

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

    os_err = OSTaskCreateExt((void (*)(void *)) ErrorTask,   /* Create the start task.                               */
                             (void          * ) 0,
                             (OS_STK        * )&ErrorTaskStk[TASK_STACK_SIZE - 1],
                             (INT8U           ) ERROR_TASK_PRIO,
                             (INT16U          ) ERROR_TASK_PRIO,  // reuse prio for ID
                             (OS_STK        * )&ErrorTaskStk[0],
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
    for(;;) {
        BSP_WatchDog_Reset();                                   /* Reset the watchdog.                                  */
        MoveFrontServo(0x00);
        //alt_write_byte(STEER_SERVO_BASE, 0x00);
        OSTimeDlyHMSM(0, 0, 0, 999);
        // 0x00 all the way to the left for the stopping servo
        //BSP_LED_On();
        //MoveFrontServo(0x00);
        //alt_write_byte(LEDR_BASE, 0x00);

        BSP_WatchDog_Reset();
        //MoveFrontServo(0x20);
        //alt_write_byte(STEER_SERVO_BASE, 0x10);
        //BSP_LED_Off();
        MoveFrontServo(0x10);
        //OSTimeDlyHMSM(0, 0, 0, 999);
        //MoveFrontServo(0x40);
        OSTimeDlyHMSM(0, 0, 0, 999);

        //////////// Steer Servo Range 0x00 - 0x31 (markings on body).  Wrap functions work.
        // For Actual use 0x00 - 0x10

        //////////// Back Servo Range 0x00 - 0x2E, 0x2F servo makes noise

        //alt_write_byte(LEDR_BASE, 0xff);
        // 0x2f all the way to the right for the stopping servo.
        //printf("FL4: %d\n", alt_read_byte(F_LEFT_BASE));
        //printf("FR5: %d\n", alt_read_byte(F_RIGHT_BASE));
        //printf("RL6: %d\n", alt_read_byte(R_LEFT_BASE));
        //printf("RR7: %d\n", alt_read_byte(R_RIGHT_BASE));
        //printf("sensor: %d\n", alt_read_byte(SONAR_BASE));
    }

}

//TODO: Change the queue names, they're confusing

static void MotorTask (void *p_arg)
{
	INT8U err;
	char *TaskName = "MotorTask";

    MotorChangeMessage *msg;
    for(;;) {

        msg = (MotorChangeMessage*)OSQPend(MotorQueue, 0, &err);

		switch(err)
        {
		case 0: // Replace with OS_NO_ERR later.
				//Message was received

                float frontLeft = msg->frontLeft;
                //And etc...
                uint8_t steeringServo = msg->steeringServo;
                free(msg);
				/*
				Here's where you can actually do what you want to do
				*/


				break;

			default:
				ErrorMessage *msg = malloc(sizeof(ErrorMessage));
                msg->_taskName = TaskName;
                msg->_sourceName = "OSQPend";
                msg->_error = err;
                OSQPost(ErrorQueue, msg);
        }
    }
}

static void FuzzyTask (void *p_arg)
{
	INT8U err;
	char *TaskName = "FuzzyTask";

    MotorSpeedMessage *msg;
    for(;;) {

        msg = (MotorSpeedMessage*)OSQPend(FuzzyQueue, 0, &err);

		switch(err)
        {
			case 0: // Replace with OS_NO_ERR Later
				//Message was received

                uint8_t frontLeft = msg->frontLeft;
                //And etc.

				/*
				Here's where you can actually do what you want to do
				*/

                MotorChangeMessage* sentMessage;
                sentMessage = malloc(sizeof(MotorChangeMessage*));

                sentMessage->frontLeft = 0;
                //...

                OSQPost(FuzzyQueue, sentMessage);
				break;

			default:
				ErrorMessage *msg = malloc(sizeof(ErrorMessage));
				msg->_taskName = TaskName;
				msg->_sourceName = "OSQPend";
				msg->_error = err;
				OSQPost(ErrorQueue, msg);
        }
    }
}

static void ErrorTask (void *p_arg)
{
	INT8U err;

    struct errorMessage *msg;
    for(;;) {

        msg = (struct motorSpeedMessage*)OSQPend(FuzzyQueue, 0, &err);
        // Do something with the error.

        free(msg);
    }
}
