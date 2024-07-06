/*
 * epwm.h
 *
 *  Created on: Mar 25, 2024
 *      Author: nov4ou
 */

#ifndef APP_EPWM_EPWM_H_
#define APP_EPWM_EPWM_H_

#include "F2806x_Device.h"          // F2806x Headerfile
#include "F2806x_Examples.h"        // F2806x Examples Headerfile

void EPWM1_Init(Uint16 tbprd);
void EPWM1A_SetCompare(Uint16 value);
void EPWM1B_SetCompare(Uint16 value);

#endif /* APP_EPWM_EPWM_H_ */
