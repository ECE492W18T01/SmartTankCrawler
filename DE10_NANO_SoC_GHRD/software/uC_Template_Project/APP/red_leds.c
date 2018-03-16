/*
 * red_leds.c
 *
 *  Created on: Nov 3, 2017
 *      Author: nemtech
 */

#include "socal/hps.h"
#include "socal/socal.h"
#include "hwlib.h"
#include "red_leds.h"

void 		FPGA_LEDS_On (void){
	uint32_t * fpga_leds = (uint32_t *)( HPS_FPGA_LEDS_BASE );

		alt_write_word(fpga_leds, 0x3ff);

}
void 		FPGA_LEDS_Off(void){
	uint32_t  * fpga_leds = (uint32_t *) ( HPS_FPGA_LEDS_BASE);

	alt_write_word(fpga_leds, 0x00);
}


