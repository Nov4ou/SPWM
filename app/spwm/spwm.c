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
Uint32 EPwm7TimerIntCount = 0;
Uint32 EPwm8TimerIntCount = 0;
Uint16 sineValue = 0;
Uint16 sineValue2 = 0;

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
  EPwm5Regs.DBRED = 10;
  EPwm5Regs.DBFED = 10;

  EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm5Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
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
  EPwm6Regs.DBRED = 10;
  EPwm6Regs.DBFED = 10;

  EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm6Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm6Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void InitPWM7() {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK = 1; // ePWM7
  EDIS;

  InitEPwm7Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Pass through
  // Allow each timer to be sync'ed
  EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;
  EPwm7Regs.TBPHS.half.TBPHS = 0;
  EPwm7Regs.TBCTR = 0x0000; // Clear counter
  EPwm7Regs.TBPRD = PWM_TBPRD;
  EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and count down
  EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm7Regs.CMPA.half.CMPA = 500; // Set compare A value
  EPwm7Regs.CMPB = 500;           // Set Compare B value

  // Set actions
  // EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
  EPwm7Regs.AQCTLA.bit.CAU = AQ_SET;   // Clear PWM1A on event A, up count
  EPwm7Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM on down count
  EPwm7Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  EPwm7Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm7Regs.DBRED = 10;
  EPwm7Regs.DBFED = 10;

  EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm7Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

void InitPWM8() {
  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // Disable TBCLK within the ePWM
  SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK = 1; // ePWM8
  EDIS;

  InitEPwm8Gpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // Stop all TB clock;
  EDIS;

  // Setup Sync
  EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE; // Pass through
  // Allow each timer to be sync'ed
  EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;
  EPwm8Regs.TBPHS.half.TBPHS = 0;
  EPwm8Regs.TBCTR = 0x0000; // Clear counter
  EPwm8Regs.TBPRD = PWM_TBPRD;
  EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and count down
  EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // Setup shadow register load on ZERO
  EPwm8Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm8Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm8Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm8Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

  // Set Compare values
  EPwm8Regs.CMPA.half.CMPA = 500; // Set compare A value
  EPwm8Regs.CMPB = 500;           // Set Compare B value

  // Set actions
  // EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
  EPwm8Regs.AQCTLA.bit.CAU = AQ_SET;   // Clear PWM1A on event A, up count
  EPwm8Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM on down count
  EPwm8Regs.AQCTLB.bit.ZRO = AQ_SET;   // Set PWM1B on Zero
  EPwm8Regs.AQCTLB.bit.CBU = AQ_CLEAR; // Clear PWM1B on event B, up count

  // Active Low PWMs - Setup Deadband
  EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm8Regs.DBRED = 10;
  EPwm8Regs.DBFED = 10;

  EPwm8Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
  EPwm8Regs.ETSEL.bit.INTEN = 1;            // Enable INT
  EPwm8Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Start all the timers synced
  EDIS;
}

interrupt void TIM0_IRQn(void) {
  EALLOW;

  PieCtrlRegs.PIEACK.bit.ACK1 = 1;
  EDIS;
}

__interrupt void epwm5_timer_isr(void) {
  EPwm5TimerIntCount++;
  static Uint16 index = 0;
  static const float step = 2 * PI * SINE_FREQ / PWM_FREQ;

  // Calculate the current sine wave value
  if (sin(step * index) >= 0)
    sineValue = (Uint16)(MAX_CMPA * sin(step * index));
  else
    sineValue = MAX_CMPA - (Uint16)(MAX_CMPA * fabs(sin(step * index)));

  // Update the duty cycle with the sine wave value
  EPwm5Regs.CMPA.half.CMPA = sineValue;

  // Increment the index and wrap around if necessary
  index++;
  if (index >= (PWM_FREQ / SINE_FREQ)) {
    index = 0;
  }

  //
  // Clear INT flag for this timer
  //
  EPwm5Regs.ETCLR.bit.INT = 1;

  //
  // Acknowledge this interrupt to receive more interrupts from group 3
  //
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm6_timer_isr(void) {
  EPwm6TimerIntCount++;
  static Uint16 index2 = 0;
  static const float step2 = 2 * PI * SINE_FREQ / PWM_FREQ;

  // Calculate the current sine wave value
  if (sin(step2 * index2) >= 0)
    sineValue2 = MAX_CMPA;
  else
    sineValue2 = 0;

  // Update the duty cycle with the sine wave value
  EPwm6Regs.CMPA.half.CMPA = sineValue2;

  // Increment the index and wrap around if necessary
  index2++;
  if (index2 >= (PWM_FREQ / SINE_FREQ)) {
    index2 = 0;
  }
  //
  // Clear INT flag for this timer
  //
  EPwm6Regs.ETCLR.bit.INT = 1;

  //
  // Acknowledge this interrupt to receive more interrupts from group 3
  //
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

// __interrupt void epwm7_timer_isr(void) {
//   EPwm7TimerIntCount++;

//   //
//   // Clear INT flag for this timer
//   //
//   EPwm7Regs.ETCLR.bit.INT = 1;

//   //
//   // Acknowledge this interrupt to receive more interrupts from group 3
//   //
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
// }

// __interrupt void epwm8_timer_isr(void) {
//   EPwm8TimerIntCount++;

//   //
//   // Clear INT flag for this timer
//   //
//   EPwm8Regs.ETCLR.bit.INT = 1;

//   //
//   // Acknowledge this interrupt to receive more interrupts from group 3
//   //
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
// }
