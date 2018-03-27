/*
 * fpga_to_hps.h
 *
 *  Created on: Mar 9, 2018
 *      Author: bofrim
 */

#ifndef APP_FPGA_TO_HPS_H_
#define APP_FPGA_TO_HPS_H_
#include <hps.h>
// Compute absolute address of any slave component attached to lightweight bridge
// base is address of component in QSYS window
// This computation only works for slave components attached to the lightweight bridge
// base should be ranged checked from 0x0 - 0x1fffff

#define FPGA_TO_HPS_LW_ADDR(base)  ((void *) (((char *)  (ALT_LWFPGASLVS_ADDR))+ (base)))

#endif /* APP_FPGA_TO_HPS_H_ */
