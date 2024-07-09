/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h"   // F2806x Headerfile
#include "F2806x_Examples.h" // F2806x Examples Headerfile
#include "adc.h"
#include "epwm.h"
#include "ina238.h"
#include "math.h"
#include "spwm.h"
#include "stdint.h"
#include "stdio.h"
#include "timer.h"

extern Uint8 deadBandA1;
extern Uint8 deadBandA2;
extern Uint8 deadBandB1;
extern Uint8 deadBandB2;
extern float sinAmp;
extern float currentGraph[GRAPH_MAX];

float i_ref = 0.75;
float i_max = -10;
float i_rms = 0;

extern float Vol1;
extern float Vol2;
extern float Vol3;
extern float Current;

typedef struct {
  float kp, ki, kd;
  float error, lastError, prevError;
  float integral, maxIntegral;
  float output, maxOutput;
} PID;
PID currentLoop;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);

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
  PieVectTable.ADCINT1 = &adc_isr; // ADC ISR
  PieVectTable.EPWM5_INT = &epwm5_timer_isr;
  // PieVectTable.EPWM6_INT = &epwm6_timer_isr;
  EDIS;

  InitAdc(); // For this example, init the ADC
  AdcOffsetSelfCal();
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
  IER |= M_INT1;                     // Enable CPU Interrupt 1
  EINT;
  ERTM;
  ADC_Init();

  // InitEPwmTimer();
  LED_Init();
  InitPWM5();
  InitPWM6();
  IER |= M_INT3;
  PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
  // PieCtrlRegs.PIEIER3.bit.INTx6 = 1;
  EINT; // Enable Global interrupt INTM
  ERTM; // Enable Global realtime interrupt DBGM

  PID_Init(&currentLoop, 0.01, 0.01, 0, 10, 10);
  TIM0_Init(90, 100); // 10khz

  while (1) {
    //
    EPwm5Regs.DBRED = deadBandA1;
    EPwm5Regs.DBFED = deadBandA2;
    EPwm6Regs.DBRED = deadBandB1;
    EPwm6Regs.DBFED = deadBandB2;
  }
}

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut) {
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->maxIntegral = maxI;
  pid->maxOutput = maxOut;
  pid->error = 0;
  pid->lastError = 0;
  pid->integral = 0;
  pid->output = 0;
}

void PID_Calc(PID *pid, float reference, float feedback) {
  pid->lastError = pid->error;
  pid->error = reference - feedback;
  float dout = (pid->error - pid->lastError) * pid->kd;
  float pout = pid->error * pid->kp;
  pid->integral += pid->error * pid->ki;
  if (pid->integral > pid->maxIntegral)
    pid->integral = pid->maxIntegral;
  else if (pid->integral < -pid->maxIntegral)
    pid->integral = -pid->maxIntegral;
  pid->output = pout + dout + pid->integral;
  if (pid->output > pid->maxOutput)
    pid->output = pid->maxOutput;
  else if (pid->output < -pid->maxOutput)
    pid->output = -pid->maxOutput;
}

interrupt void TIM0_IRQn(void) {
  EALLOW;
  Uint8 i = 0;
  i_max = -10;
  for (i = 0; i < GRAPH_MAX; i++) {
    if (currentGraph[i] > i_max)
    i_max = currentGraph[i];
  }
  i_rms = i_max / 1.41421356;

  PID_Calc(&currentLoop, i_ref, i_rms);
  // sinAmp = 0.5 + currentLoop.output / 10;
  sinAmp = 1;

  if (sinAmp < 0.2)
    sinAmp = 0.2;
  if (sinAmp > 1)
    sinAmp = 1;

  PieCtrlRegs.PIEACK.bit.ACK1 = 1;
  EDIS;
}
