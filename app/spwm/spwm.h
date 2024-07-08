/*
 * spwm.h
 *
 *  Created on: Jul 5, 2024
 *      Author: nov4ou
 */

#ifndef APP_SPWM_SPWM_H_
#define APP_SPWM_SPWM_H_

#include "F2806x_Device.h"          // F2806x Headerfile
#include "F2806x_Examples.h"        // F2806x Examples Headerfile

#define PWM_TBPRD 4500 // 10kHz PWM frequency
#define PWM_FREQ 10000 // 10kHz PWM frequency
#define SINE_FREQ 49.51   // 50Hz sine wave frequency
// #define SINE_FREQ 34
#define MAX_CMPA 4500  // Maximum value for CMPA, should be same as TBPRD
#define PI 3.14159265358979

void InitPWM5();
void InitPWM6();
void InitPWM7();
void InitPWM8();
__interrupt void epwm5_timer_isr(void);
__interrupt void epwm6_timer_isr(void);
__interrupt void epwm7_timer_isr(void);
__interrupt void epwm8_timer_isr(void);

#endif /* APP_SPWM_SPWM_H_ */
