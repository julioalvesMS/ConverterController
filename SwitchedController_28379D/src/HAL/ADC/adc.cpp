#include <src/HAL/ADC/adc.h>

#define ADC_TRIG_SOURCE     0xb // Triggered by PWM4A
#define ADC_PIN_VOUT        4   // ADC_C4
#define ADC_PIN_VIN         4   // ADC_B4
#define ADC_PIN_IL          4   // ADC_A4
#define ADC_PIN_IL_AVG      5   // ADC_C5

namespace ADC
{
    //
    // Configure - Write ADC configurations and power up the ADC for both
    //                ADC A and ADC B
    //
    void Configure(void)
    {
        EALLOW;

        //
        //write configurations
        //
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
        AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

        //
        //Set pulse positions to late
        //
        AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

        //
        //power up the ADC
        //
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

        //
        //delay for 1ms to allow ADC time to power up
        //
//        DELAY_US(1000);

        EDIS;
    }

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC(void)
    {
        Uint16 tempsensor_acqps;

        tempsensor_acqps = 139; //temperature sensor needs at least 700ns
                                //acquisition time

        //
        //Select the channels to convert and end of conversion flag
        //
        EALLOW;
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 13;  //SOC0 will convert internal
                                             //connection A13
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = tempsensor_acqps; //sample window is 100
                                                          //SYSCLK cycles
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    //trigger on ePWM1 SOCA/C
        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  //end of SOC0 will set INT1 flag
        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared
        EDIS;
    }

    //
    // ConfigureInterruption - Set the interruption handler and define where
    //                          to send the results from the conversion
    //
    void ConfigureInterruption(double* variables[])
    {
        int i;

        //
        // Set the interruption handler for the ADC PWM
        //
        EALLOW;
        PieVectTable.ADCA1_INT = &(IsrInterruption);
        EDIS;

        for (i=0;i<ADC_CONVERSIONS;i++)
        {
            resultDestination[i] = variables[i];
        }

        //
        // enable PIE interrupt
        //
        PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    }


    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void IsrInterruption(void)
    {
        //
        // Read ADC result
        //
        (*resultDestination[0]) = AdccResultRegs.ADCRESULT0; // Inductor Current
        (*resultDestination[1]) = AdccResultRegs.ADCRESULT1; // Load Voltage
        (*resultDestination[2]) = AdcbResultRegs.ADCRESULT0; // Input Voltage

        //
        // Prepare for next conversion
        //
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

        //
        // Check if overflow has occurred
        //
        if(1 == AdcbRegs.ADCINTOVF.bit.ADCINT1)
        {
            AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
            AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        }
        if(1 == AdccRegs.ADCINTOVF.bit.ADCINT1)
        {
            AdccRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
            AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        }

        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

        return;
    }
}
