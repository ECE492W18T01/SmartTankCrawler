################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../APP/FuzzyLogicProcessor.c \
../APP/app.c \
../APP/fMatrices_servoFuzzyThree.c \
../APP/fuzzyMotorDrive.c \
../APP/motorDriveR.c \
../APP/motor_controls.c \
../APP/red_leds.c \
../APP/serial_communication.c \
../APP/slipLessSteeringRatios.c \
../APP/timer.c \
../APP/wrap.c 

C_DEPS += \
./APP/FuzzyLogicProcessor.d \
./APP/app.d \
./APP/fMatrices_servoFuzzyThree.d \
./APP/fuzzyMotorDrive.d \
./APP/motorDriveR.d \
./APP/motor_controls.d \
./APP/red_leds.d \
./APP/serial_communication.d \
./APP/slipLessSteeringRatios.d \
./APP/timer.d \
./APP/wrap.d 

OBJS += \
./APP/FuzzyLogicProcessor.o \
./APP/app.o \
./APP/fMatrices_servoFuzzyThree.o \
./APP/fuzzyMotorDrive.o \
./APP/motorDriveR.o \
./APP/motor_controls.o \
./APP/red_leds.o \
./APP/serial_communication.o \
./APP/slipLessSteeringRatios.o \
./APP/timer.o \
./APP/wrap.o 


# Each subdirectory must supply rules for building sources it contributes
APP/%.o: ../APP/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C Compiler 5'
	armcc --cpu=Cortex-A9 --no_unaligned_access -Dsoc_cv_av -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\APP" -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\BSP" -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\BSP\OS" -I"C:\intelFPGA\17.0\embedded\ip\altera\hps\altera_hps\hwlib\include" -I"C:\intelFPGA\17.0\embedded\ip\altera\hps\altera_hps\hwlib\include\soc_cv_av" -I"C:\intelFPGA\17.0\embedded\ip\altera\hps\altera_hps\hwlib\include\soc_cv_av\socal" -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\HWLIBS" -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uC-CPU\ARM-Cortex-A" -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uC-CPU" -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uC-LIBS" -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uCOS-II\Ports" -I"C:\Users\jcrobert\Documents\Repositories\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uCOS-II\Source" --c99 --gnu -O0 -g --md --depend_format=unix_escaped --no_depend_system_headers --depend_dir="APP" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


