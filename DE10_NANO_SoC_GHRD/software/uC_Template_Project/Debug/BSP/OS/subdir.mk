################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/OS/bsp_os.c 

C_DEPS += \
./BSP/OS/bsp_os.d 

OBJS += \
./BSP/OS/bsp_os.o 


# Each subdirectory must supply rules for building sources it contributes
BSP/OS/%.o: ../BSP/OS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM C Compiler 5'
	armcc --cpu=Cortex-A9 --no_unaligned_access -Dsoc_cv_av -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\APP" -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\BSP" -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\BSP\OS" -I"C:\intelFPGA\17.0\embedded\ip\altera\hps\altera_hps\hwlib\include" -I"C:\intelFPGA\17.0\embedded\ip\altera\hps\altera_hps\hwlib\include\soc_cv_av" -I"C:\intelFPGA\17.0\embedded\ip\altera\hps\altera_hps\hwlib\include\soc_cv_av\socal" -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\HWLIBS" -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uC-CPU\ARM-Cortex-A" -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uC-CPU" -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uC-LIBS" -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uCOS-II\Ports" -I"C:\Users\kgmills\Desktop\TestDE10\SmartTankCrawler\DE10_NANO_SoC_GHRD\software\uC_Template_Project\uCOS-II\Source" --c99 --gnu -O0 -g --md --depend_format=unix_escaped --no_depend_system_headers --depend_dir="BSP/OS" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


