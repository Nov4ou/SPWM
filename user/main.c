/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h"   // F2806x Headerfile
#include "F2806x_Examples.h" // F2806x Examples Headerfile
#include "epwm.h"
#include "ina238.h"
#include "math.h"
#include "spwm.h"
#include "stdint.h"
#include "stdio.h"
#include "timer.h"

void LED_Init(void) {
  EALLOW;
  //    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;

  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;

  GpioDataRegs.GPASET.bit.GPIO0 = 1;

  EDIS;
}

void EPWM_Set_Compare(float out) {
  Uint8 value = 1800 * out / 100;
  EPWM1A_SetCompare(value);
}

int main() {
  InitSysCtrl();
  InitPieCtrl();
  IER = 0x0000;
  IFR = 0x0000;
  InitPieVectTable();

  EALLOW;
  PieVectTable.EPWM5_INT = &epwm5_timer_isr;
  PieVectTable.EPWM6_INT = &epwm6_timer_isr;
  // PieVectTable.EPWM6_INT = &epwm7_timer_isr;
  // PieVectTable.EPWM6_INT = &epwm8_timer_isr;
  EDIS;

  // InitEPwmTimer();
  LED_Init();
  InitPWM5();
  InitPWM6();
  InitPWM7();
  InitPWM8();
  IER |= M_INT3;
  PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
  PieCtrlRegs.PIEIER3.bit.INTx6 = 1;
  // PieCtrlRegs.PIEIER3.bit.INTx7 = 1;
  // PieCtrlRegs.PIEIER3.bit.INTx8 = 1;
  EINT; // Enable Global interrupt INTM
  ERTM; // Enable Global realtime interrupt DBGM

  // TIM0_Init(90, 100); // 10khz

  // only in test
  while (1) {
    //f
    // dasfdasfdsafdsafads/f\fas
  }
}
