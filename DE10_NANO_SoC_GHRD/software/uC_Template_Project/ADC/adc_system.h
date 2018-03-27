/*
 * adc_ltc2308 configuration
 *
 */
#ifndef __ADC_SYSTEM_H__
#define __ADC_SYSTEM_H__

#define FPGA_TO_HPS_LW_ADDR(base)  ((void *) (((char *)  (ALT_LWFPGASLVS_ADDR))+ (base)))

#define ADC_LTC2308_BASE 0x81010
#define ADC_LTC2308_IRQ -1
#define ADC_LTC2308_IRQ_INTERRUPT_CONTROLLER_ID -1
#define ADC_LTC2308_NAME "/dev/adc_ltc2308"
#define ADC_LTC2308_SPAN 8
#define ADC_LTC2308_TYPE "adc_ltc2308"
#define ALT_MODULE_CLASS_adc_ltc2308 adc_ltc2308
#define SW_BASE 0x0

#endif
