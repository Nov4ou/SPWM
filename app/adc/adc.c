#include "adc.h"

Uint16 LoopCount;
Uint16 ConversionCount;
Uint16 Voltage1[10];
Uint16 Voltage2[10];
Uint16 Voltage3[10];

float solarVol = 0;
float Vol1 = 0;
float Vol2 = 0;
float Vol3 = 0;
float Current = 0;

float currentGraph[GRAPH_MAX];
Uint8 indexTest = 0;

void ADC_Init()
{
    //
    // Configure ADC
    //
    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1; // Enable non-overlap mode

    //
    // ADCINT1 trips after AdcResults latch
    //
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    AdcRegs.INTSEL1N2.bit.INT1E     = 1;  // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;  // Disable ADCINT1 Continuous mode

    //
    // setup EOC1 to trigger ADCINT1 to fire
    //
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 1;

    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 5;  // set SOC0 channel select to ADCINA5
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 0xA;  // set SOC1 channel select to ADCINB2
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 0x3;  // set SOC1 channel select to ADCINA3
    //
    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;

    //
    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;

    //
    // set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts
    // first then SOC1
    //
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;

    //
    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;

    //
    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;
    EDIS;

    //
    // set SOC3 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    //
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;
    EDIS;

    //
    // Assumes ePWM1 clock is already enabled in InitSysCtrl();
    //
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;        // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;        // Select SOC from CMPA on upcount
    EPwm1Regs.ETPS.bit.SOCAPRD  = 1;        // Generate pulse on 1st event
    EPwm1Regs.CMPA.half.CMPA    = 0x0080;   // Set compare A value
    EPwm1Regs.TBPRD             = 0xFF00;   // Set period for ePWM1
    // EPwm1Regs.TBPRD             = 4500;   // Set period for ePWM1
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;        // count up and start

}

//
// adc_isr -
//
__interrupt void
adc_isr(void)
{
    Voltage1[ConversionCount] = AdcResult.ADCRESULT0;
    Voltage2[ConversionCount] = AdcResult.ADCRESULT1;
    Voltage3[ConversionCount] = AdcResult.ADCRESULT2;

    Vol1 = Voltage1[ConversionCount] * 3.3 / 4095;
    Vol2 = Voltage2[ConversionCount] * 3.3 / 4095;
    Vol3 = Voltage3[ConversionCount] * 3.3 / 4095;

    Current = (Vol2 - 1.506) * 38.5956 / 20;
    currentGraph[indexTest++] = Current;
    if (indexTest >= GRAPH_MAX)
      indexTest = 0;
    //
    // If 20 conversions have been logged, start over
    //
    if(ConversionCount == 9)
    {
        ConversionCount = 0;
    }
    else
    {
        ConversionCount++;
    }

    //
    // Clear ADCINT1 flag reinitialize for next SOC
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

    LoopCount++;
    return;
}
