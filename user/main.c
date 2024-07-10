/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h" // F2806x Headerfile
#include "F2806x_EPwm.h"
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
float i_ref_rt;
float i_max = -10;
float i_rms = 0;
extern float i_vpp;
float filtered_i_rms = 0;

extern float Vol1;
extern float Vol2;
extern float Vol3;
extern float Current;
extern float error;
extern float biased_error;
extern float sineWave;

typedef struct {
  float x_est; // Estimated state value
  float P_est; // Estimated state covariance
  float Q;     // Process noise covariance
  float R;     // Measurement noise covariance
} KalmanFilter;
KalmanFilter I_rms;

typedef struct {
  float kp, ki, kd;
  float error, lastError, prevError;
  float integral, maxIntegral;
  float output, maxOutput;
} PID;
PID currentLoop;

void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float reference, float feedback);
void kalmanFilter_Init(KalmanFilter *kf);
float kalman_filter(KalmanFilter *kf, float z);

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
  i_vpp = i_ref * sqrt(2);
  LED_Init();
  InitPWM5();
  InitPWM6();
  EALLOW;
  SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1; // ePWM5
  SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1; // ePWM6
  EDIS;
  IER |= M_INT3;
  PieCtrlRegs.PIEIER3.bit.INTx5 = 1;
  // PieCtrlRegs.PIEIER3.bit.INTx6 = 1;
  EINT; // Enable Global interrupt INTM
  ERTM; // Enable Global realtime interrupt DBGM

  kalmanFilter_Init(&I_rms);
  
  // PID_Init(&currentLoop, 50, 0, 0, 1, 400);
  TIM0_Init(90, 100); // 0.1khz

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

void kalmanFilter_Init(KalmanFilter *kf) {
  // Initialize Kalman Filter
  kf->x_est = 0;  // Initial estimate of state value
  kf->P_est = 1;  // Initial estimate of state covariance
  kf->Q = 0.0001; // Process noise covariance
  kf->R = 0.01;   // Measurement noise covariance
}

float kalman_filter(KalmanFilter *kf, float z) {
  // Prediction step
  float x_pred = kf->x_est;
  float P_pred = kf->P_est + kf->Q;

  // Update step
  float K = P_pred / (P_pred + kf->R);
  kf->x_est = x_pred + K * (z - x_pred);
  kf->P_est = (1 - K) * P_pred;

  return kf->x_est;
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
  filtered_i_rms = kalman_filter(&I_rms, i_rms);

  // error =  3.8 + (i_ref_rt - Current) + i_ref * 1.41421356;
  // if (Current > i_vpp)
  //   Current = i_vpp;
  error = (i_ref_rt - Current) / i_vpp / 15;
  biased_error = error + i_vpp / 15 + 0.09;

  // sineWave = error * 400;
  // if (sineWave > 4499)
  //   sineWave = 0;

  sineWave = biased_error * 8000;
  if (sineWave > 4499)
    sineWave = 4499;
  if (sineWave < 0)
    sineWave = 0;

  // Update the duty cycle with the sine wave value
  // EPwm5Regs.CMPA.half.CMPA = (Uint16) sineWave;

  // EPwm6Regs.CMPA.half.CMPA = (Uint16) sineWave;
  // PID_Calc(&currentLoop, i_ref_rt, Current);
  // sineWave += (i_ref_rt - Current) * 3;
  // sineWave2 = sineWave / 30 * 0.5;

  // sineWave3 = -sineWave2;

  // sineWave2 += 0.5;
  // sineWave3 += 0.5;

  // EPwm5Regs.CMPA.half.CMPA = sineWave2;
  // EPwm6Regs.CMPA.half.CMPA = sineWave3;

  // sinAmp = 2000 + currentLoop.output;
  // // sinAmp = 0.5;

  // if (sinAmp < 100)
  //   sinAmp = 100;
  // if (sinAmp > 4000)
  //   sinAmp = 1;

  PieCtrlRegs.PIEACK.bit.ACK1 = 1;
  EDIS;
}
