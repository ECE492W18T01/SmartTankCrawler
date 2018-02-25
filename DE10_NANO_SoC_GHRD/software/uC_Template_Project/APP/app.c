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


// Compute absolute address of any slave component attached to lightweight bridge
// base is address of component in QSYS window
// This computation only works for slave components attached to the lightweight bridge
// base should be ranged checked from 0x0 - 0x1fffff

#define FPGA_TO_HPS_LW_ADDR(base)  ((void *) (((char *)  (ALT_LWFPGASLVS_ADDR))+ (base)))

#define APP_TASK_PRIO 5
#define TASK_STACK_SIZE 4096

/*
*********************************************************************************************************
*										Qsys Components
*********************************************************************************************************
*/
// Green FPGA Leds
// Type:  Input
// Width: Byte
// GPIO:  N/A
#define LEDR_ADD 0x00000100
#define LEDR_BASE FPGA_TO_HPS_LW_ADDR(LEDR_ADD)

// Servo #1, Steering Control
// Type:  Input
// Width: Byte
// GPIO:  0_0
#define STEER_SERVO_ADD 0x00000110
#define STEER_SERVO_BASE FPGA_TO_HPS_LW_ADDR(STEER_SERVO_BASE)

// Servo #2, Emergency Braking System
// Type:  Input
// Width: Byte
// GPIO:  0_1
#define BRAKE_SERVO_ADD 0x00000111
#define BRAKE_SERV_BASE FPGA_TO_HPS_LW_ADDR(BRAKE_SERVO_ADD)

// Drive Motor #1, Front Left
// Type:  Input
// Width: Longword
// GPIO:  dir - 0_26; mag - 0_27
#define FRONT_LEFT_MOTOR_ADD 0x00000114
#define FRONT_LEFT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(FRONT_LEFT_MOTOR_ADD)

// Drive Motor #2, Front Right
// Type:  Input
// Width: Longword
// GPIO:  dir - 0_28; mag - 0_29
#define FRONT_RIGHT_MOTOR_ADD 0x00000118
#define FRONT_RIGHT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(FRONT_RIGHT_MOTOR_ADD)

// Drive Motor #3, Rear Left
// Type:  Input
// Width: Longword
// GPIO:  dir - 0_30; mag - 0_31
#define REAR_LEFT_MOTOR_ADD 0x0000011c
#define REAR_LEFT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(REAR_LEFT_MOTOR_ADD)

// Drive Motor #4, Rear Right
// Type:  Input
// Width: Longword
// GPIO:  dir - 0_32; mag - 0_33
#define REAR_RIGHT_MOTOR_ADD 0x00000120
#define REAR_RIGHT_MOTOR_BASE FPGA_TO_HPS_LW_ADDR(REAR_RIGHT_MOTOR_ADD)


/* Hall Sensor GPIO
* FL - 0_4
* FR - 0_5
* RL - 0_6
* RR - 0_7
*/
// Hall Sensor Front Left / Front Right Ratio
// Type:  Output
// Width: Longword
#define FRONT_RATIO_ADD 0x00000124
#define FRONT_RATIO_BASE FPGA_TO_HPS_LW_ADDR(FRONT_RATIO_ADD)

// Hall Sensor Rear Left / Rear Right Ratio
// Type:  Output
// Width: Longword
#define REAR_RATIO_ADD 0x00000128
#define REAR_RATIO_BASE FPGA_TO_HPS_LW_ADDR(REAR_RATIO_ADD)

// Hall Sensor Overall Ratio
// Type:  Output
// Width: Longword
#define OVERALL_RATIO_ADD 0x0000012c
#define OVERALL_RATIO_BASE FPGA_TO_HPS_LW_ADDR(OVERALL_RATIO_ADD)

// Ultrasonic Range Finder
// Type:  Output
// Width: Longword
// GPIO:  0_8
#define SONAR_ADD 0x00000130
#define SONAR_BASE FPGA_TO_HPS_LW_ADDR(SONAR_ADD)

// Lab 2 Sensor
// Type:  Output
// Width: Longword
// GPIO:  0_34
#define SENSOR_ADD 0x00000200
#define SENSOR_BASE FPGA_TO_HPS_LW_ADDR(SENSOR_ADD)


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

CPU_STK AppTaskStartStk[TASK_STACK_SIZE];


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart              (void        *p_arg);


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

        //OSTimeDlyHMSM(0, 0, 0, 999);
        // 0x00 all the way to the left for the stopping servo
        BSP_LED_On();
        alt_write_byte(LEDR_BASE, 0x00);

        OSTimeDlyHMSM(0, 0, 0, 999);

        BSP_WatchDog_Reset();
        BSP_LED_Off();
        alt_write_byte(LEDR_BASE, 0xff);
        // 0x2f all the way to the right for the stopping servo.
        printf("%d\n", alt_read_word(SENSOR_BASE));


        //servo (0x2F = 2.5ms, 0x20 = 2.0ms, 0x1A =1.8ms (need 1.5ms) )
    }

}
