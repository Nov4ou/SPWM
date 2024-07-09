/*
 * spwm.c
 *
 *  Created on: Jul 5, 2024
 *      Author: nov4ou
 */

#include "spwm.h"
#include "math.h"

Uint32 EPwm5TimerIntCount = 0;
Uint32 EPwm6TimerIntCount = 0;
Uint16 sineValue = 0;
Uint16 sineValue2 = 0;

float sinout;
Uint8 deadBandA1 = 80;
Uint8 deadBandA2 = 80;
Uint8 deadBandB1 = 80;
Uint8 deadBandB2 = 80;

float sineWave = 0;
float sineWave2 = 0;
float sineWave3 = 0;
float error = 0;

float sinAmp = 0.5;
extern float i_ref;
extern float i_ref_rt;
extern float Current;

typedef struct {
  float kp, ki, kd;
  float error, lastError, prevError;
  float integral, maxIntegral;
  float output, maxOutput;
} PID;
extern PID currentLoop;

extern void PID_Init(PID *pid, float p, float i, float d, float maxI,
                     float maxOut);
extern void PID_Calc(PID *pid, float reference, float feedback);

void InitPWM5() {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1; // ePWM5
  EDIS;

  InitEPwm5Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Pass through
  // Allow each timer to be sync'ed
  EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;
  EPwm5Regs.TBPHS.half.TBPHS = 0;
  EPwm5Regs.TBCTR = 0x0000; // Clear counter
  EPwm5Regs.TBPRD = PWM_TBPRD;
  EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and count down
  EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm5Regs.CMPA.half.CMPA = 500; // Set compare A value
  EPwm5Regs.CMPB = 500;           // Set Compare B value

  // Set actions
  // EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
  EPwm5Regs.AQCTLA.bit.CAU = AQ_SET;   // Clear PWM1A on event A, up count
  EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM on down count
  EPwm5Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm5Regs.DBRED = deadBandA1;
  EPwm5Regs.DBFED = deadBandA2;

  EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm5Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 0; // ePWM5
  EDIS;
}

void InitPWM6() {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1; // ePWM6
  EDIS;

  InitEPwm6Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Pass through
  // Allow each timer to be sync'ed
  EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;
  EPwm6Regs.TBPHS.half.TBPHS = 0;
  EPwm6Regs.TBCTR = 0x0000; // Clear counter
  EPwm6Regs.TBPRD = PWM_TBPRD;
  EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and count down
  EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm6Regs.CMPA.half.CMPA = 500; // Set compare A value
  EPwm6Regs.CMPB = 500;           // Set Compare B value

  // Set actions
  // EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
  EPwm6Regs.AQCTLA.bit.CAU = AQ_SET;   // Clear PWM1A on event A, up count
  EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM on down count
  EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm6Regs.DBRED = deadBandB1;
  EPwm6Regs.DBFED = deadBandB2;

  EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm6Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 0; // ePWM6
  EDIS;
}

__interrupt void epwm5_timer_isr(void) {
  EPwm5TimerIntCount++;
  static Uint16 index = 0;
  static const float step = 2 * PI * SINE_FREQ / PWM_FREQ;

  i_ref_rt = sin(step * index) * i_ref * 1.41421356;

  //   sineWave += (i_ref_rt - Current) * 3;
  // sineWave2 = sineWave / 30 * 0.5;

  // sineWave3 = -sineWave2;

  // sineWave2 += 0.5;
  // sineWave3 += 0.5;

  // if (sineWave2 < 0)
  //   sineWave2 = 0;
  // if (sineWave2 > 4500)
  //   sineWave2 = 4500;
  // if (sineWave2 < 0)
  //   sineWave2 = 0;
  // if (sineWave2 > 4500)
  //   sineWave2 = 4500;
  // EPwm5Regs.CMPA.half.CMPA = (Uint16)sineWave2;
  // EPwm6Regs.CMPA.half.CMPA = (Uint16)sineWave3;

  /************************** Open Loop ******************************/
  // // Calculate the current sine wave value
  // sineValue = (Uint16)((MAX_CMPA / 2) * (1 + sin(step * index) * sinAmp));
  // i_ref_rt = sin(step * index) * i_ref * 1.41421356;

  // sineValue2 = (Uint16)((MAX_CMPA / 2) * (1 - sin(step * index) * sinAmp));

  // // Update the duty cycle with the sine wave value
  // EPwm5Regs.CMPA.half.CMPA = sineValue;

  // EPwm6Regs.CMPA.half.CMPA = sineValue2;

  // // Increment the index and wrap around if necessary
  index++;
  if (index >= (PWM_FREQ / SINE_FREQ)) {
    index = 0;
  }
  /************************** Open Loop ******************************/

  /*************************** Close Loop *****************************/
  // PID_Calc(&currentLoop, i_ref_rt, Current);
  // sineValue = (currentLoop.output + 0.5) * (MAX_CMPA / 2);

  error = i_ref_rt - Current;
  sinout = error * 800;

  if (sinout < (-1 * MAX_CMPA / 2))
    sinout = -MAX_CMPA / 2;
  if (sinout > MAX_CMPA / 2)
    sinout = MAX_CMPA / 2;

  sinout += MAX_CMPA / 2;

  EPwm5Regs.CMPA.half.CMPA = sinout;
  EPwm6Regs.CMPA.half.CMPA = sinout;

  /*************************** Close Loop *****************************/

  //
  // Clear INT flag for this timer
  //
  EPwm5Regs.ETCLR.bit.INT = 1;

  //
  // Acknowledge this interrupt to receive more interrupts from group 3
  //
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// __interrupt void
// epwm6_timer_isr(void)
// {
//     EPwm6TimerIntCount++;
//     static Uint16 index2 = 0;
//     static const float step2 = 2 * PI * SINE_FREQ / PWM_FREQ;

//     // Calculate the current sine wave value
//     sineValue2 = (Uint16)((MAX_CMPA / 2) * (1 - sin(step2 * index2)));

//     // Update the duty cycle with the sine wave value
//     EPwm6Regs.CMPA.half.CMPA = sineValue2;

//     // Increment the index and wrap around if necessary
//     index2++;
//     if (index2 >= (PWM_FREQ / SINE_FREQ))
//     {
//         index2 = 0;
//     }

//     //
//     // Clear INT flag for this timer
//     //
//     EPwm6Regs.ETCLR.bit.INT = 1;

//     //
//     // Acknowledge this interrupt to receive more interrupts from group 3
//     //
//     PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
// }
