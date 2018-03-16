/*
 * red_leds.h
 *
 *  Created on: Nov 3, 2017
 *      Author: nemtech
 */

#define 	RED_LEDS_BASE	0x00000000
#define 	HPS_FPGA_LEDS_BASE	( ALT_LWFPGASLVS_ADDR  + RED_LEDS_BASE)

void FPGA_LEDS_On (void);
void FPGA_LEDS_Off (void);
