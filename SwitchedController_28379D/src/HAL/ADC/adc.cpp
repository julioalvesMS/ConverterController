#include <src/HAL/ADC/adc.h>

#define ADC_TRIG_SOURCE         5   // Triggered by PWM4A

#define ADC_CHANNEL_IL          4   // ADC_A4
#define ADC_CHANNEL_VIN         4   // ADC_B4
#define ADC_CHANNEL_VOUT        4   // ADC_C4
#define ADC_CHANNEL_IL_AVG      5   // ADC_C5

namespace ADC_HAL
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
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4


        AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
        AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
        AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

        //
        //Set pulse positions to late
        //
        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

        //
        //power up the ADC
        //
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

        //
        //delay for 1ms to allow ADC time to power up
        //
        DELAY_US(1000);

        EDIS;
    }

    void SetupADC(void)
    {
        SetupADC_A();
        SetupADC_B();
        SetupADC_C();
    }

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_A(void)
    {
        Uint16 acqps;

        //
        // Determine minimum acquisition window (in SYSCLKS) based on resolution
        //
        if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
        {
            acqps = 14; //75ns
        }
        else //resolution is 16-bit
        {
            acqps = 63; //320ns
        }

        //
        //Select the channels to convert and end of conversion flag
        //
        EALLOW;
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_IL;  //SOC0 will convert the inductor current
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is 100 SYSCLK cycles
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ADC_TRIG_SOURCE; //trigger on ePWM1 SOCA/C
        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
        EDIS;
    }

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_B(void)
    {
        Uint16 acqps;

        //
        // Determine minimum acquisition window (in SYSCLKS) based on resolution
        //
        if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION)
        {
            acqps = 14; //75ns
        }
        else //resolution is 16-bit
        {
            acqps = 63; //320ns
        }

        //
        //Select the channels to convert and end of conversion flag
        //
        EALLOW;
        AdcbRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_VIN;    //SOC0 will convert pin A0
        AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;      //sample window is 100 SYSCLK cycles
        AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;        //trigger on ePWM1 SOCA/C
        AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      //end of SOC0 will set INT1 flag
        AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;        //enable INT1 flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //make sure INT1 flag is cleared
        EDIS;
    }

    //
    // SetupADC - Configure ADC EPWM acquisition window and trigger
    //
    void SetupADC_C(void)
    {
        Uint16 acqps;

        //
        // Determine minimum acquisition window (in SYSCLKS) based on resolution
        //
        if(ADC_RESOLUTION_12BIT == AdccRegs.ADCCTL2.bit.RESOLUTION)
        {
            acqps = 14; //75ns
        }
        else //resolution is 16-bit
        {
            acqps = 63; //320ns
        }

        //
        //Select the channels to convert and end of conversion flag
        //
        EALLOW;
        AdccRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_VOUT;    //SOC0 will convert pin A0
        AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps;      //sample window is 100 SYSCLK cycles
        AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;        //trigger on ePWM1 SOCA/C
        AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      //end of SOC0 will set INT1 flag
        AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;        //enable INT1 flag
        AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      //make sure INT1 flag is cleared
        EDIS;
    }

    //
    // ConfigureInterruption - Set the interruption handler and define where
    //                          to send the results from the conversion
    //
    void ConfigureInterruption(unsigned int* variables[])
    {
        int i;

        //
        // Set the interruption handler for the ADC PWM
        //
        EALLOW;
        PieVectTable.ADCA1_INT = &(IsrInterruption_A);
        PieVectTable.ADCB1_INT = &(IsrInterruption_B);
        PieVectTable.ADCC1_INT = &(IsrInterruption_C);
        EDIS;

        for (i=0;i<ADC_CONVERSIONS;i++)
        {
            resultDestination[i] = variables[i];
        }
    }


    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void IsrInterruption_A(void)
    {
        //
        // Read ADC result
        //
        (*resultDestination[0]) = AdcaResultRegs.ADCRESULT0; // Inductor Current
        (*resultDestination[1]) = AdccResultRegs.ADCRESULT0; // Load Voltage
        (*resultDestination[2]) = AdcbResultRegs.ADCRESULT0; // Input Voltage

        //
        // Prepare for next conversion
        //
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

        //
        // Check if overflow has occurred
        //
        if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
        {
            AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
            AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        }

        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

        return;
    }


    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void IsrInterruption_B(void)
    {
        //
        // Read ADC result
        //
        (*resultDestination[2]) = AdcbResultRegs.ADCRESULT0; // Input Voltage

        //
        // Prepare for next conversion
        //
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

        //
        // Check if overflow has occurred
        //
        if(1 == AdcbRegs.ADCINTOVF.bit.ADCINT1)
        {
            AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
            AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        }

        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

        return;
    }


    //
    // IsrInterruption - Interruption called on the end of ADC Conversion
    //
    __interrupt
    void IsrInterruption_C(void)
    {
        //
        // Read ADC result
        //
        (*resultDestination[1]) = AdccResultRegs.ADCRESULT0; // Load Voltage

        //
        // Prepare for next conversion
        //
        AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

        //
        // Check if overflow has occurred
        //
        if(1 == AdccRegs.ADCINTOVF.bit.ADCINT1)
        {
            AdccRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
            AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
        }

        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

        return;
    }
}
